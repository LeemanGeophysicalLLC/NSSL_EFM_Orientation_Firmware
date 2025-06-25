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
#include "TinyGPS++.h"
#include "ICM_20948.h"
#include "AS5600.h"
#include "pins.h"

// #define DEBUG_PRINT_ENABLE
// #define WATCHDGOG_ENABLE //TODO Use
// #define SHOW_DATA

// ======================= Constants ======================= //
const uint32_t DEBUG_BAUD = 9600;
const uint32_t GPS_BAUD = 9600;
const uint32_t SPI_SPEED = SD_SCK_MHZ(18);
const uint32_t SAMPLE_INTERVAL = 100;      // ms
const size_t LOG_BUFFER_LEN = 256;
const float ANGLE_OFFSET_DEG = 12.0; // Offset for AS5600 angle in degrees
const size_t MAX_LOG_LINES = 30;
const size_t WRITE_THRESHOLD = 20;
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
HardwareSerial SerGPS(PA3, PA2);
HardwareSerial SerDebug(PB11, PB10);
TinyGPSPlus gps;
SdFat sd;
File logFile;
ICM_20948_I2C imu;
AS5600 as5600;

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
  const int maxChars = 10;
  int count = 0;
  while (SerGPS.available() && count < maxChars) {
    if (gps.encode(SerGPS.read())) {
      if (gps.time.isValid() && runtime.ppsSeen) {
        runtime.lastUTC = gps.time.value();
        runtime.gpsEpochMillis = runtime.ppsMillis;
        runtime.gpsLocked = true;
        runtime.gpsLastValidMillis = millis();
        runtime.ppsSeen = false;
        debugPrintln("GPS synced to PPS");
      }
    }
    count++;
  }
}

/**
 * @brief Generate a formatted timestamp string from GPS + PPS.
 *
 * @param buf Output buffer
 * @param len Length of output buffer
 */
void getTimestamp(char* buf, size_t len) {
  if (!runtime.gpsLocked || !gps.time.isValid() || !gps.date.isValid()) {
    strncpy(buf, "0000-00-00 00:00:00.000", len);
    return;
  }
  uint32_t msSincePPS = millis() - runtime.gpsEpochMillis;
  snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03lu",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second(),
           msSincePPS % 1000);
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
  debugPrintln("Taking sensor readings");
  float angle, ax, ay, az, gx, gy, gz, mx, my, mz;
  bool sensorsOk = readSensors(angle, ax, ay, az, gx, gy, gz, mx, my, mz);

  char timestamp[32];
  getTimestamp(timestamp, sizeof(timestamp));

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

  // Format into CSV line (no float formatting required)
  char line[LOG_BUFFER_LEN];
  snprintf(line, sizeof(line), "%s,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
           timestamp, now, iAngle, iAx, iAy, iAz,
           iGx, iGy, iGz, iMx, iMy, iMz);

  addLogLine(line);

  if (bufferedLines >= WRITE_THRESHOLD) {
    writeBufferedLines();
  }

  if (sensorsOk && runtime.gpsLocked) {
    runtime.currentState = RuntimeStatus::STATE_LOGGING;
  }
}

/**
 * @brief Arduino setup function.
 */
void setup() {
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

  SerDebug.begin(DEBUG_BAUD);
  SerGPS.begin(GPS_BAUD);
  Wire.begin();
  SPI.begin();

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

  #ifdef WATCHDOG_ENABLE
  IWDG->KR = 0xCCCC; // Start watchdog
  IWDG->KR = 0x5555; // Enable write access
  IWDG->PR = 6;      // Prescaler for ~1s timeout
  IWDG->RLR = 312;   // Reload value
  IWDG->KR = 0xAAAA; // Reload counter
  #endif
}

/**
 * @brief Main loop.
 */
void loop() {
  updateLED();
  pollGPS();

  if (millis() - runtime.gpsLastValidMillis > GPS_TIMEOUT_MS)
  {
    debugPrintln("GPS timeout");
    runtime.gpsError = true;
    runtime.currentState = RuntimeStatus::STATE_ERROR;
  }

  if (runtime.currentState == RuntimeStatus::STATE_ERROR && runtime.gpsLocked && !runtime.logError) {
    runtime.currentState = RuntimeStatus::STATE_LOGGING;
  }

  uint32_t now = millis();
  if (now - runtime.lastSample >= SAMPLE_INTERVAL) {
    debugPrintln("Time to read");
    //runtime.lastSample += SAMPLE_INTERVAL;
    runtime.lastSample = now; // Update lastSample to current time
    takeReading(now);
  }
  else
  {
    debugPrintln("Not time to read yet");
  }

  #ifdef WATCHDOG_ENABLE
  IWDG->KR = 0xAAAA; // Pet the watchdog
  #endif
}
