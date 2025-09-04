/*
 * NSSL EFM Orientation Paddle Firmware
 * 
 * This firmware reads the IMU, axle position sensor, and GPS to create
 * a log of the instrument orientation at 10Hz and log it to an SD card.
 * Time is GPS synchronized to allow comparison with the data logged
 * in the spheres and the axle position serves as a check. The magnetometer
 * can also be used to check the synchronization with the sphere data.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include "ICM_20948.h"
#include "AS5600.h"
#include "pins.h"
#include "IWatchdog.h"
#include "TinyGPS++.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

//#define DEBUG_PRINT_ENABLE
#define WATCHDOG_ENABLE
// #define SHOW_DATA
// #define SKIP_GPS_WAIT // For testing without waiting for GPS fix

// ======================= Constants ======================= //
const uint32_t DEBUG_BAUD = 115200;
const uint32_t GPS_BAUD = 115200;
const uint32_t SPI_SPEED = SD_SCK_MHZ(18);
const uint32_t SAMPLE_INTERVAL = 100; // ms
const size_t LOG_BUFFER_LEN = 256;
const float ANGLE_OFFSET_DEG = 0.0; // Offset for AS5600 angle in degrees
const size_t MAX_LOG_LINES = 30;
const size_t WRITE_THRESHOLD = 10;
#define GPS_TIMEOUT_MS 60000

// ======================= Debug Macros ======================= //
#ifdef DEBUG_PRINT_ENABLE
  #define debugPrint(x) SerDebug.print(x)
  #define debugPrintln(x) SerDebug.println(x)
#else
  #define debugPrint(x)
  #define debugPrintln(x)
#endif

// ======================= Interfaces ======================= //
HardwareSerial SerDebug(PB11, PB10);
HardwareSerial SerGPS(PA3, PA2);
SdFat sd;
TinyGPSPlus gps;
File logFile;
ICM_20948_I2C imu;
AS5600 as5600;
SFE_UBLOX_GNSS myGNSS;

// ======================= Time Struct ======================= //
struct UtcStamp {
  uint16_t year; uint8_t month, day, hour, minute, second;
} utcAtLastPps;

// ======================= Runtime State ======================= //
/**
 * @brief Struct to hold runtime flags and timestamps for system state management.
 */
struct RuntimeStatus {
  volatile uint32_t ppsMillis = 0;              ///< Timestamp of last PPS signal
  volatile bool ppsSeen = false;                ///< PPS pulse seen flag
  uint32_t gpsEpochMillis = 0;                  ///< Millis at last GPS PPS sync
  uint32_t lastSample = 0;                      ///< Timestamp of last sensor sample
  uint32_t lastUTC = 0;                         ///< Last UTC value from GPS
  uint32_t gpsLastValidMillis = 0;              ///< Last millis() when GPS time was valid
  bool gpsLocked = false;                       ///< GPS has valid time and PPS sync
  bool gpsError = false;                        ///< GPS timeout occurred
  bool logError = false;                        ///< SD card write or open failure

  /**
   * @brief High-level system state.
   */
  enum SystemState {
    STATE_ACQUIRING_GPS, ///< Waiting for valid GPS
    STATE_LOGGING,       ///< Normal logging operation
    STATE_ERROR          ///< Error state due to sensor, GPS, or SD failure
  } currentState = STATE_ACQUIRING_GPS;
};
RuntimeStatus runtime;

char lineBuffer[MAX_LOG_LINES][LOG_BUFFER_LEN];
size_t bufferedLines = 0;

/**
 * @brief Initialize and configure the u-blox GPS module.
 * 
 * This function performs a complete setup of the u-blox GNSS module, including:
 * 
 * - Verifying I2C communication with the module
 * - Setting the dynamic model to AIRBORNE <4g> for high-altitude operation
 * - Setting the UART1 baud rate to 115200
 * - Reducing output rate to 1 Hz for power efficiency and sync with 1PPS
 * - Disabling unnecessary NMEA messages for bandwidth and parser performance
 * - Enabling only RMC (Recommended Minimum), ZDA (time), and GGA (position/altitude)
 * - Configuring the TIMEPULSE output on pin 0 to generate a 100 ms pulse at 1 Hz
 * - Saving all settings to battery-backed memory
 * 
 * This function is blocking and will halt execution permanently if the GPS module
 * is not detected on the I2C bus. All configuration changes are made using the
 * SFE_UBLOX_GNSS Arduino Library.
 * 
 * @note If GPS configuration fails, detailed debug messages are printed to SerDebug.
 */
bool setupGPS()
{
  if (!myGNSS.begin()) {
    debugPrintln("GPS not detected. Check I2C wiring.");
    return false;
  }

  debugPrintln("Configuring GPS...");

  // Set dynamic model to AIRBORNE <4g>
  if (!myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g)) {
    debugPrintln("Failed to set dynamic model");
    return false;
  }

  // Set UART1 baud rate to 115200
  myGNSS.setSerialRate(115200, COM_PORT_UART1); // No return value
  debugPrintln("Set UART1 baud rate to 115200.");


  // Set navigation rate to 1 Hz
  if (!myGNSS.setNavigationFrequency(1))
  {
    debugPrintln("Failed to set navigation frequency");
    return false;
  }

  // Disable all common NMEA messages
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);

  // Enable RMC and ZDA only
  myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  myGNSS.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);

  UBX_CFG_TP5_data_t timePulseSettings;
  memset(&timePulseSettings, 0, sizeof(UBX_CFG_TP5_data_t)); // Clear struct

  timePulseSettings.tpIdx = 0;                   // TIMEPULSE pin 0
  timePulseSettings.version = 0x01;
  timePulseSettings.flags.bits.active = 1;       // Enable output
  timePulseSettings.flags.bits.lockedOtherSet = 1; // Align to top of second
  timePulseSettings.flags.bits.isFreq = 1;       // Frequency mode
  timePulseSettings.flags.bits.isLength = 1;     // Length is valid
  //timePulseSettings.flags.bits.pulseDef = 0;     // Pulse is 'time high'
  timePulseSettings.freqPeriod = 1;              // 1 Hz
  timePulseSettings.freqPeriodLock = 1;
  timePulseSettings.pulseLenRatio = 100000;      // 100ms = 100,000 ns
  timePulseSettings.pulseLenRatioLock = 100000;

  if (!myGNSS.setTimePulseParameters(&timePulseSettings))
  {
    debugPrintln("Failed to configure time pulse!");
  }
  else
  {
    debugPrintln("Time pulse configured.");
  }

  // Save settings
  if (!myGNSS.saveConfiguration())
  {
    debugPrintln("Failed to save configuration!");
    return false;
  }
  else
  {
    debugPrintln("GPS configuration saved.");
  }

  debugPrintln("Setup complete. GPS will output RMC/ZDA/GGA at 115200 baud over UART1.");
  return true;
}

/**
 * @brief Increment a UTC stamp by one second.
 *
 * This helper advances the provided UtcStamp struct by exactly one
 * second, rolling over minutes and hours as needed. Day/month/year
 * rollover is not fully implemented here since logging at 10 Hz will
 * not skip across a day boundary, but can be added if required.
 *
 * @param u Reference to a UtcStamp structure to be incremented
 */
static void bumpOneSecond(UtcStamp &u) {
  if (++u.second <= 59) return;
  u.second = 0;
  if (++u.minute <= 59) return;
  u.minute = 0;
  if (++u.hour <= 23) return;
  u.hour = 0;
  // Day/month/year rollovers omitted for brevity; at 10 Hz you won’t skip a day.
}

/**
 * @brief Format a PPS-aligned UTC timestamp string.
 *
 * This function generates a timestamp string in the form
 * YYYYMMDDTHHMMSS.mmm using the UTC date/time latched at the
 * last GPS PPS event (utcAtLastPps) and the elapsed milliseconds
 * since that PPS edge.
 *
 * - The UTC date/time fields come from the PPS-aligned latch so
 *   the integer seconds are guaranteed to match the PPS boundary.
 * - The fractional milliseconds are computed from
 *   (now - runtime.gpsEpochMillis).
 * - If more than 1000 ms have elapsed, the seconds are carried
 *   forward using bumpOneSecond() until the millisecond remainder
 *   is less than 1000.
 *
 * @param[out] out     Character buffer to receive the timestamp string
 * @param[in]  outlen  Length of the output buffer
 * @param[in]  now     Current millis() value when the reading is taken
 */
void formatTimestamp(char *out, size_t outlen, uint32_t now) {
  // Milliseconds since the last PPS-aligned UTC boundary
  uint32_t dms = now - runtime.gpsEpochMillis;

  // Copy the latched UTC and add whole seconds if we’ve drifted >1000 ms
  UtcStamp u = utcAtLastPps;
  while (dms >= 1000) { dms -= 1000; bumpOneSecond(u); }

  // Format: YYYYMMDDTHHMMSS.mmm
  snprintf(out, outlen, "%04u%02u%02uT%02u%02u%02u.%03lu",
           (unsigned)u.year, (unsigned)u.month, (unsigned)u.day,
           (unsigned)u.hour, (unsigned)u.minute, (unsigned)u.second,
           (unsigned long)dms);
}

/**
 * @brief Checks if the GPS has a valid fix and accurate time.
 * 
 * This function verifies that:
 * - The GPS time is valid (gps.time.isValid() is true)
 * - The GPS location is valid (gps.location.isValid() is true)
 * 
 * If both conditions are met, the GPS is considered "locked" and time is reliable.
 * This avoids trusting GPS-estimated time from cold boot or almanac restore.
 * 
 * @return true if GPS has a valid fix and time; false otherwise.
 */
bool isGPSLocked() {
  const unsigned long maxAge = 2000; // milliseconds
  return gps.time.isValid() &&
         gps.location.isValid() &&
         gps.time.age() < maxAge &&
         gps.location.age() < maxAge;
}

/**
 * @brief Set RGB LED state using PWM values.
 *
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_LED_RED, r);
  analogWrite(PIN_LED_GREEN, g);
  analogWrite(PIN_LED_BLUE, b);
}

/**
 * @brief GPS PPS interrupt handler.
 */
void ppsISR() {
  runtime.ppsMillis = millis();
  runtime.ppsSeen = true;
}

/**
 * @brief Poll a limited number of GPS characters and sync time if valid.
 */
void pollGPS() {
  const int maxChars = 100;
  int count = 0;
  while (SerGPS.available() && count < maxChars) {
    char c = SerGPS.read();
    debugPrint(c); // Print GPS character for debugging
    if (gps.encode(c)) {
      if (isGPSLocked() && runtime.ppsSeen) {
        // Latch UTC at the PPS boundary we just saw
        utcAtLastPps.year   = gps.date.year();
        utcAtLastPps.month  = gps.date.month();
        utcAtLastPps.day    = gps.date.day();
        utcAtLastPps.hour   = gps.time.hour();
        utcAtLastPps.minute = gps.time.minute();
        utcAtLastPps.second = gps.time.second();

        runtime.gpsEpochMillis = runtime.ppsMillis;  // start of this UTC second (ms)
        runtime.gpsLocked = true;
        runtime.gpsLastValidMillis = millis();
        runtime.ppsSeen = false;
        debugPrintln("GPS synced to PPS");
      }
    }
    count++;
  }
// Reset lock if location or time becomes stale
if (millis() - runtime.gpsLastValidMillis > 3000) {
  runtime.gpsLocked = false;
}

}

/**
 * @brief Update LED color to reflect current system state.
 */
void updateLED() {
  switch (runtime.currentState) {
    case RuntimeStatus::STATE_ACQUIRING_GPS: setLED(0, 0, 255); break;
    case RuntimeStatus::STATE_LOGGING:       setLED(0, 255, 0); break;
    case RuntimeStatus::STATE_ERROR:         setLED(255, 0, 0); break;
    default:                                 setLED(0, 0, 0); break;
  }
}

/**
 * @brief Read sensor data from IMU and AS5600.
 *
 * @return true if all sensor reads succeeded
 */
bool readSensors(float& angleDeg, float& ax, float& ay, float& az,
                 float& gx, float& gy, float& gz,
                 float& mx, float& my, float& mz) {
  constexpr float BAD = -9999.0f;
  bool imuOk = false;
  bool angleOk = false;

  if (imu.dataReady())
  {
    imu.getAGMT();
    if (imu.status == ICM_20948_Stat_Ok)
    {
      ax = imu.accX(); ay = imu.accY(); az = imu.accZ();
      gx = imu.gyrX(); gy = imu.gyrY(); gz = imu.gyrZ();
      mx = imu.magX(); my = imu.magY(); mz = imu.magZ();
      imuOk = true;
    }
  }
  else
  {
    debugPrintln("IMU data not ready");
  }

  if (!imuOk) {
    ax = ay = az = gx = gy = gz = mx = my = mz = BAD;
    debugPrintln("IMU read failed");
  }

  uint16_t raw = as5600.rawAngle();
  if (raw != 0xFFFF)
  {
    angleDeg = raw * 360.0f / 4096.0f - ANGLE_OFFSET_DEG;
    if (angleDeg < 0) angleDeg += 360.0f;
    angleOk = true;
  }
  else
  {
    angleDeg = BAD;
    debugPrintln("AS5600 read failed");
  }
  return imuOk && angleOk;
}

/**
 * @brief Create a new unique log file.
 */
bool openLogFile() {
  char filename[16];
  for (int i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "OPLOG%03d.TXT", i);
    if (!sd.exists(filename)) {
      logFile = sd.open(filename, FILE_WRITE);
      logFile.print("Timestamp,Millis,Lat,Lon,Elev,Angle,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ");
      if (!logFile || !logFile.println("")) {
        runtime.logError = true;
        return false;
      }
      logFile.flush();
      return true;    
    }
  }
  runtime.logError = true;
  return false;
}

/**
 * @brief Flush buffer contents to SD, retry once on failure.
 */
void writeBufferedLines() {
  debugPrintln("Writing buffered log lines to SD");
  if (!logFile || bufferedLines == 0) return;
  for (size_t i = 0; i < bufferedLines; i++) {
    if (!logFile.println(lineBuffer[i])) {
      runtime.currentState = RuntimeStatus::STATE_ERROR;
      runtime.logError = true;
      logFile.close();
      if (sd.begin(PIN_SD_CS, SPI_SPEED) && openLogFile()) {
        logFile.println(lineBuffer[i]);
        logFile.flush();
      }
      break;
    } else {
      #ifdef SHOW_DATA
      debugPrintln(lineBuffer[i]);
      #endif
    }
  }
  logFile.flush();
  bufferedLines = 0;
}

/**
 * @brief Add a formatted line to the log buffer.
 */
void addLogLine(const char* line) {
  if (!logFile || bufferedLines >= MAX_LOG_LINES) return;
  strncpy(lineBuffer[bufferedLines], line, LOG_BUFFER_LEN);
  lineBuffer[bufferedLines][LOG_BUFFER_LEN - 1] = '\0';
  bufferedLines++;
}

/**
 * @brief Capture and store a sensor reading with timestamp.
 */
void takeReading(uint32_t now) {
  char timestamp[24];
  if (runtime.gpsLocked) {
    formatTimestamp(timestamp, sizeof(timestamp), now);
  } else {
    strcpy(timestamp, "00000000T000000.000");
  }

  float lat = gps.location.isValid() ? gps.location.lat() : -999.0;
  float lon = gps.location.isValid() ? gps.location.lng() : -999.0;
  float alt = gps.altitude.isValid() ? gps.altitude.meters() : -999.0;

  float angle, ax, ay, az, gx, gy, gz, mx, my, mz;
  bool sensorsOk = readSensors(angle, ax, ay, az, gx, gy, gz, mx, my, mz);

  // Convert float values to fixed-point (3 decimal places)
  int32_t iAngle = static_cast<int32_t>(angle * 1000.0f);
  int32_t iAx    = static_cast<int32_t>(ax    * 1000.0f);
  int32_t iAy    = static_cast<int32_t>(ay    * 1000.0f);
  int32_t iAz    = static_cast<int32_t>(az    * 1000.0f);
  int32_t iGx    = static_cast<int32_t>(gx    * 1000.0f);
  int32_t iGy    = static_cast<int32_t>(gy    * 1000.0f);
  int32_t iGz    = static_cast<int32_t>(gz    * 1000.0f);
  int32_t iMx    = static_cast<int32_t>(mx    * 1000.0f);
  int32_t iMy    = static_cast<int32_t>(my    * 1000.0f);
  int32_t iMz    = static_cast<int32_t>(mz    * 1000.0f);
  int32_t iLat = static_cast<int32_t>(lat * 1e7);   // 7 decimal places
  int32_t iLon = static_cast<int32_t>(lon * 1e7);
  int32_t iAlt = static_cast<int32_t>(alt * 1000);  // meters × 1000

  // Format into CSV line (no float formatting required)
  char line[LOG_BUFFER_LEN];
  snprintf(line, sizeof(line),
         "%s,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
         timestamp, now,
         iLat, iLon, iAlt,
         iAngle, iAx, iAy, iAz,
         iGx, iGy, iGz, iMx, iMy, iMz);

  addLogLine(line);

  if (bufferedLines >= WRITE_THRESHOLD) {
    writeBufferedLines();
  }

  
  if (!sensorsOk) {
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }
}

/**
 * @brief Arduino setup function.
 */
void setup() {

  // Cycle LED to indicate startup
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  setLED(255, 0, 0);
  delay(1000);
  setLED(0, 255, 0);
  delay(1000);
  setLED(0, 0, 255);
  delay(1000);
  setLED(0, 0, 0);
  delay(1000);
  updateLED();

  #ifdef WATCHDOG_ENABLE
  IWatchdog.begin(26000000); //max 26208000
  #endif

  SerDebug.begin(DEBUG_BAUD);
  SerGPS.begin(GPS_BAUD);
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  SPI.begin();

  if (!setupGPS())
  {
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }

  pinMode(PIN_GPS_PPS, INPUT); // PPS pin as input with pull-up
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  debugPrintln("Initializing sensors...");
  imu.begin(Wire, 1); // 1 for AD0 high
  if (imu.status != ICM_20948_Stat_Ok)
  {
    debugPrintln("IMU initialization failed");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }
  else debugPrintln("IMU initialized successfully");

  uint8_t whoami = imu.getWhoAmI();
  debugPrint("WHO_AM_I: 0x");
  debugPrintln(whoami);

  // Set up IMU Ranges
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm8; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (imu.status != ICM_20948_Stat_Ok)
  {
    debugPrintln("IMU range setting failed");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }

  if (!as5600.begin())
  {
    debugPrintln("Encoder initialization failed");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }
  else debugPrintln("Encoder initialized successfully");

  if (!sd.begin(PIN_SD_CS, SPI_SPEED))
  {
    debugPrintln("SD card initialization failed");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }
  else debugPrintln("SD card initialized successfully");

  if (!openLogFile())
  {
    debugPrintln("Failed to open log file");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }
  else debugPrintln("Log file opened successfully");

  if (runtime.currentState == RuntimeStatus::STATE_ERROR)
  {
    debugPrintln("Error state detected, stopping setup");
    updateLED();
    while(1){}
  }

  // Wait for GPS to lock
  #ifndef SKIP_GPS_WAIT
  debugPrintln("Waiting for GPS lock...");
  while(!isGPSLocked()) {
    pollGPS();
    #ifdef WATCHDOG_ENABLE
    IWatchdog.reload();
    #endif
  }
  debugPrintln("GPS lock acquired");
  runtime.currentState = RuntimeStatus::STATE_LOGGING;
  #endif
}

/**
 * @brief Main loop.
 */
void loop()
{
  uint32_t now = millis();

  // Check if its time to read
  if (now - runtime.lastSample >= SAMPLE_INTERVAL)
  {
    takeReading(now);
    runtime.lastSample = now;
  }

  pollGPS();

  // If we have lost fix, turn the state and LED back to BLUE but keep logging
// Update system state based on GPS lock unless in error state
if (runtime.currentState != RuntimeStatus::STATE_ERROR) {
  runtime.currentState = runtime.gpsLocked ? 
                         RuntimeStatus::STATE_LOGGING : 
                         RuntimeStatus::STATE_ACQUIRING_GPS;
}


  #ifdef WATCHDOG_ENABLE
  IWatchdog.reload();
  #endif
  updateLED();
}
