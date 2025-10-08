#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <SPI.h>
#include <SD.h>
#include <esp_timer.h>
#include <math.h>
#include <ctype.h>
#include <cstring>

#include "firmware/esp32_log_format.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// -------- Pins (XIAO ESP32-S3) --------
constexpr int MOTOR_EN_PIN  = 4;   // D3 (PWM)
constexpr int MOTOR_PHS_PIN = 3;   // D2 (direction)
constexpr int SERVO_PIN     = 43;  // D6

constexpr int I2C_SDA_PIN   = 5;   // D4
constexpr int I2C_SCL_PIN   = 6;   // D5

constexpr int PPS_GPIO           = 9;    // PPS pulse output to ESP32-CAM
constexpr int UART_SYNC_TX_PIN   = 17;   // UART TX for sync burst
constexpr int UART_SYNC_RX_PIN   = -1;   // RX unused
constexpr uint32_t UART_SYNC_BAUD = 115200;

constexpr int SD_CS_PIN          = 7;    // Adjust per wiring
constexpr uint32_t LOG_RATE_HZ   = 200;
constexpr uint32_t LOG_PERIOD_US = 1000000UL / LOG_RATE_HZ;
constexpr uint32_t PPS_PULSE_US  = 10;
constexpr size_t   LOG_QUEUE_DEPTH = 256;
constexpr uint32_t LOG_FLUSH_INTERVAL = 200;

// Battery sensing disabled by default; set to ADC pin if available
constexpr int VBAT_ADC_PIN = -1;

// -------- Globals --------
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
ESP32PWM motorPwm;
Servo steeringServo;
HardwareSerial& syncSerial = Serial1;

bool imuOk = false;
bool sdOk = false;

static BLEServer* bleServer = nullptr;
static BLECharacteristic* bleCmdChar = nullptr;
static BLECharacteristic* bleImuChar = nullptr;
static BLECharacteristic* bleStatsChar = nullptr;

static const char* BLE_DEVICE_NAME = "RC-DRIFT-XIAO";
static BLEUUID svcUUID("12345678-1234-5678-1234-56789abcdef0");
static BLEUUID cmdUUID("12345678-1234-5678-1234-56789abcdef1");
static BLEUUID imuUUID("12345678-1234-5678-1234-56789abcdef2");
static BLEUUID statsUUID("12345678-1234-5678-1234-56789abcdef3");

const uint32_t TIMEOUT_MS = 1200;
volatile bool bleClientConnected = false;
volatile uint32_t lastCmdMillis = 0;

static QueueHandle_t logQueue = nullptr;
static TaskHandle_t acquisitionTaskHandle = nullptr;
static TaskHandle_t loggingTaskHandle = nullptr;
static File logFile;

static volatile bool g_lastSampleValid = false;
static esp32_log_record_t g_lastSample = {};
static portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool g_sessionActive = false;
static volatile bool g_startRequested = false;
static volatile bool g_stopRequested = false;
static char g_pendingFilename[64] = {0};
static char g_runLabel[32] = {0};
static portMUX_TYPE sessionMux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t logSequence = 0;
static portMUX_TYPE seqMux = portMUX_INITIALIZER_UNLOCKED;

struct RuntimeStats {
  uint32_t sensorDrops = 0;
  uint32_t queueOverruns = 0;
  uint32_t recordsQueued = 0;
  uint32_t recordsWritten = 0;
};

static RuntimeStats g_stats;
static portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;

static volatile int currentThrottle = 0;
static volatile int currentSteering = 90;

// Forward declarations
void motorSetPercent(int percent);
void steeringWriteSafe(int angle);
void applyCommand(const String& cmd);
static void acquisitionTask(void* param);
static void loggingTask(void* param);
static void triggerPps(uint64_t timestampUs);
static void emitSyncBurst(uint64_t timestampUs);
static bool queueLogRecord(esp32_log_record_t record);
static bool writeRecord(esp32_log_record_t& record);
static bool isSessionActive();
static void statsRecordQueued(bool success);
static void statsRecordWritten();
static void statsSensorDrop();
static void updateBleStats();
static void requestSessionStart();
static void requestSessionStop();
static void setRunLabel(const String& label);
static uint16_t readBatteryMv();
static void copyString(char* dst, size_t len, const char* src);

// CRC helpers
static uint32_t crc32_ieee(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFF;
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void copyString(char* dst, size_t len, const char* src) {
  if (!dst || len == 0) {
    return;
  }
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, len - 1);
  dst[len - 1] = '\0';
}

// BLE callbacks
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    motorSetPercent(0);
    pServer->getAdvertising()->start();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String value = ch->getValue();
    if (value.length() > 0) {
      applyCommand(value);
      lastCmdMillis = millis();
    }
  }
};

// Utility accessors
static bool isSessionActive() {
  portENTER_CRITICAL(&sessionMux);
  bool active = g_sessionActive;
  portEXIT_CRITICAL(&sessionMux);
  return active;
}

static uint8_t nextSequence() {
  portENTER_CRITICAL(&seqMux);
  uint8_t seq = logSequence++;
  portEXIT_CRITICAL(&seqMux);
  return seq;
}

static void statsRecordQueued(bool success) {
  portENTER_CRITICAL(&statsMux);
  if (success) {
    g_stats.recordsQueued++;
  } else {
    g_stats.queueOverruns++;
  }
  portEXIT_CRITICAL(&statsMux);
}

static void statsRecordWritten() {
  portENTER_CRITICAL(&statsMux);
  g_stats.recordsWritten++;
  portEXIT_CRITICAL(&statsMux);
}

static void statsSensorDrop() {
  portENTER_CRITICAL(&statsMux);
  g_stats.sensorDrops++;
  portEXIT_CRITICAL(&statsMux);
}

static void getStatsSnapshot(RuntimeStats& out) {
  portENTER_CRITICAL(&statsMux);
  out = g_stats;
  portEXIT_CRITICAL(&statsMux);
}

static void makeLogFilename(char* out, size_t len) {
  uint64_t ts = static_cast<uint64_t>(esp_timer_get_time());
  portENTER_CRITICAL(&sessionMux);
  bool hasLabel = g_runLabel[0] != '\0';
  portEXIT_CRITICAL(&sessionMux);
  if (hasLabel) {
    snprintf(out, len, "/logs/%s_%llu.bin", g_runLabel, static_cast<unsigned long long>(ts));
  } else {
    snprintf(out, len, "/logs/run_%llu.bin", static_cast<unsigned long long>(ts));
  }
}

static void requestSessionStart() {
  if (!sdOk) {
    Serial.println("[log] SD card not initialized; start request ignored");
    return;
  }
  portENTER_CRITICAL(&sessionMux);
  if (g_sessionActive || g_startRequested) {
    portEXIT_CRITICAL(&sessionMux);
    Serial.println("[log] Session already active or pending");
    return;
  }
  portEXIT_CRITICAL(&sessionMux);
  char filename[sizeof(g_pendingFilename)] = {0};
  makeLogFilename(filename, sizeof(filename));
  portENTER_CRITICAL(&sessionMux);
  copyString(g_pendingFilename, sizeof(g_pendingFilename), filename);
  g_startRequested = true;
  portEXIT_CRITICAL(&sessionMux);
  Serial.printf("[log] Start requested -> %s\n", filename);
}

static void requestSessionStop() {
  portENTER_CRITICAL(&sessionMux);
  if (!g_sessionActive || g_stopRequested) {
    portEXIT_CRITICAL(&sessionMux);
    Serial.println("[log] No active session to stop");
    return;
  }
  g_stopRequested = true;
  portEXIT_CRITICAL(&sessionMux);
  Serial.println("[log] Stop requested");
}

static void setRunLabel(const String& label) {
  String trimmed = label;
  trimmed.trim();
  char sanitized[sizeof(g_runLabel)] = {0};
  size_t len = trimmed.length();
  if (len >= sizeof(sanitized)) {
    len = sizeof(sanitized) - 1;
  }
  for (size_t i = 0; i < len; ++i) {
    char c = trimmed[i];
    if (isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
      sanitized[i] = c;
    } else {
      sanitized[i] = '_';
    }
  }
  sanitized[len] = '\0';
  portENTER_CRITICAL(&sessionMux);
  strncpy(g_runLabel, sanitized, sizeof(g_runLabel));
  g_runLabel[sizeof(g_runLabel) - 1] = '\0';
  portEXIT_CRITICAL(&sessionMux);
  Serial.printf("[log] Run label set to '%s'\n", g_runLabel);
}

static void emitSyncBurst(uint64_t timestampUs) {
  uint8_t packet[12];
  packet[0] = 0xAA;
  packet[1] = 0x55;
  memcpy(&packet[2], &timestampUs, sizeof(timestampUs));
  uint16_t crc = crc16_ccitt(packet, 10);
  packet[10] = static_cast<uint8_t>(crc & 0xFF);
  packet[11] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  syncSerial.write(packet, sizeof(packet));
}

static void triggerPps(uint64_t timestampUs) {
  digitalWrite(PPS_GPIO, HIGH);
  delayMicroseconds(PPS_PULSE_US);
  digitalWrite(PPS_GPIO, LOW);
  emitSyncBurst(timestampUs);
  if (isSessionActive()) {
    esp32_log_record_t event = {};
    event.timestamp_us = timestampUs;
    event.flags = ESP32_LOG_FLAG_SYNC;
    queueLogRecord(event);
  }
}

static bool queueLogRecord(esp32_log_record_t record) {
  if (!logQueue) {
    return false;
  }
  if (xQueueSend(logQueue, &record, 0) == pdTRUE) {
    statsRecordQueued(true);
    return true;
  }
  statsRecordQueued(false);
  return false;
}

static bool writeRecord(esp32_log_record_t& record) {
  if (!logFile) {
    return false;
  }
  record.sequence = nextSequence();
  record.crc32 = crc32_ieee(reinterpret_cast<const uint8_t*>(&record), sizeof(record) - sizeof(record.crc32));
  size_t written = logFile.write(reinterpret_cast<const uint8_t*>(&record), sizeof(record));
  if (written != sizeof(record)) {
    Serial.println("[log] Write failure");
    return false;
  }
  static uint32_t flushCountdown = 0;
  if (++flushCountdown >= LOG_FLUSH_INTERVAL) {
    logFile.flush();
    flushCountdown = 0;
  }
  statsRecordWritten();
  return true;
}

static uint16_t readBatteryMv() {
  if (VBAT_ADC_PIN < 0) {
    return 0;
  }
  uint16_t millivolts = analogReadMilliVolts(VBAT_ADC_PIN);
  return millivolts;
}

static void updateBleStats() {
  if (!bleStatsChar) {
    return;
  }
  RuntimeStats snapshot;
  getStatsSnapshot(snapshot);
  bool active = isSessionActive();
  char payload[96];
  snprintf(payload, sizeof(payload),
           "active=%d,queued=%lu,written=%lu,dropped=%lu,overrun=%lu",
           active ? 1 : 0,
           static_cast<unsigned long>(snapshot.recordsQueued),
           static_cast<unsigned long>(snapshot.recordsWritten),
           static_cast<unsigned long>(snapshot.sensorDrops),
           static_cast<unsigned long>(snapshot.queueOverruns));
  bleStatsChar->setValue(reinterpret_cast<uint8_t*>(payload), strlen(payload));
  if (bleClientConnected) {
    bleStatsChar->notify();
  }
}

void applyCommand(const String& cmd) {
  int start = 0;
  while (start < static_cast<int>(cmd.length())) {
    while (start < static_cast<int>(cmd.length()) && isspace(static_cast<unsigned char>(cmd[start]))) start++;
    if (start >= static_cast<int>(cmd.length())) break;
    int end = start;
    while (end < static_cast<int>(cmd.length()) && !isspace(static_cast<unsigned char>(cmd[end]))) end++;
    String tok = cmd.substring(start, end);
    start = end;

    int colon = tok.indexOf(':');
    if (colon > 0) {
      String key = tok.substring(0, colon);
      String val = tok.substring(colon + 1);
      key.toUpperCase();
      val.trim();
      if (key == "T" || key == "THROTTLE") {
        int v = val.toInt();
        v = constrain(v, -100, 100);
        currentThrottle = v;
        motorSetPercent(currentThrottle);
      } else if (key == "S" || key == "STEER" || key == "STEERING") {
        int a = val.toInt();
        a = constrain(a, 0, 180);
        currentSteering = a;
        steeringWriteSafe(currentSteering);
      } else if (key == "LOG" || key == "SESSION") {
        String upper = val;
        upper.toUpperCase();
        if (upper == "START") {
          requestSessionStart();
        } else if (upper == "STOP") {
          requestSessionStop();
        }
      } else if (key == "RUN") {
        setRunLabel(val);
      }
    }
  }
}

static void acquisitionTask(void* param) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(1000 / LOG_RATE_HZ);
  uint64_t lastTimestamp = esp_timer_get_time();
  uint64_t nextPps = ((lastTimestamp / 1000000ULL) + 1ULL) * 1000000ULL;

  while (true) {
    vTaskDelayUntil(&lastWake, periodTicks);

    if (!imuOk) {
      continue;
    }

    sensors_event_t accel, mag, gyro, temp;
    if (!lsm.getEvent(&accel, &mag, &gyro, &temp)) {
      statsSensorDrop();
      continue;
    }

    uint64_t timestampUs = static_cast<uint64_t>(esp_timer_get_time());

    esp32_log_record_t record = {};
    record.timestamp_us = timestampUs;

    auto toMg = [](float acc) -> int16_t {
      float g = acc / 9.80665f;
      long mg = lroundf(g * 1000.0f);
      return static_cast<int16_t>(constrain(mg, -32767L, 32767L));
    };

    auto toCentiDps = [](float rad) -> int16_t {
      float dps = rad * 57.2957795f;
      long cds = lroundf(dps * 100.0f);
      return static_cast<int16_t>(constrain(cds, -32767L, 32767L));
    };

    record.accel_mg[0] = toMg(accel.acceleration.x);
    record.accel_mg[1] = toMg(accel.acceleration.y);
    record.accel_mg[2] = toMg(accel.acceleration.z);
    record.gyro_cds[0] = toCentiDps(gyro.gyro.x);
    record.gyro_cds[1] = toCentiDps(gyro.gyro.y);
    record.gyro_cds[2] = toCentiDps(gyro.gyro.z);
    record.throttle_pct = currentThrottle;
    record.steering_deg = currentSteering;
    record.vbat_mv = readBatteryMv();

    portENTER_CRITICAL(&sampleMux);
    g_lastSample = record;
    g_lastSampleValid = true;
    portEXIT_CRITICAL(&sampleMux);

    if (isSessionActive()) {
      queueLogRecord(record);
    }

    if (timestampUs >= nextPps) {
      triggerPps(timestampUs);
      nextPps += 1000000ULL;
    }
  }
}

static void loggingTask(void* param) {
  while (true) {
    char filename[sizeof(g_pendingFilename)] = {0};
    bool startReq = false;
    bool stopReq = false;

    portENTER_CRITICAL(&sessionMux);
    if (g_startRequested && !g_sessionActive) {
      g_startRequested = false;
      startReq = true;
      copyString(filename, sizeof(filename), g_pendingFilename);
    }
    if (g_stopRequested && g_sessionActive) {
      g_stopRequested = false;
      stopReq = true;
    }
    portEXIT_CRITICAL(&sessionMux);

    if (startReq) {
      logFile = SD.open(filename, FILE_WRITE);
      if (logFile) {
        portENTER_CRITICAL(&sessionMux);
        g_sessionActive = true;
        portEXIT_CRITICAL(&sessionMux);
        esp32_log_record_t event = {};
        event.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
        event.flags = ESP32_LOG_FLAG_SESSION_START;
        writeRecord(event);
        Serial.printf("[log] Session started -> %s\n", filename);
      } else {
        Serial.println("[log] Failed to open log file");
      }
    }

    if (stopReq && logFile) {
      esp32_log_record_t event = {};
      event.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
      event.flags = ESP32_LOG_FLAG_SESSION_STOP;
      writeRecord(event);
      logFile.flush();
      logFile.close();
      logFile = File();
      portENTER_CRITICAL(&sessionMux);
      g_sessionActive = false;
      portEXIT_CRITICAL(&sessionMux);
      Serial.println("[log] Session stopped");
    }

    esp32_log_record_t record;
    if (xQueueReceive(logQueue, &record, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (logFile) {
        writeRecord(record);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== RC Car BLE Control (XIAO ESP32-S3) ===");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN);
  pinMode(MOTOR_PHS_PIN, OUTPUT);
  motorPwm.attachPin(MOTOR_EN_PIN, 5000, 8);
  steeringWriteSafe(currentSteering);
  motorSetPercent(0);

  pinMode(PPS_GPIO, OUTPUT);
  digitalWrite(PPS_GPIO, LOW);

  pinMode(SD_CS_PIN, OUTPUT);
  SPI.begin();
  sdOk = SD.begin(SD_CS_PIN);
  if (sdOk) {
    SD.mkdir("/logs");
    Serial.println("[log] SD card ready");
  } else {
    Serial.println("[log] SD card init failed");
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("Initializing LSM9DS1 sensor...");
  if (lsm.begin()) {
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    imuOk = true;
    Serial.println("LSM9DS1 ready.");
  } else {
    imuOk = false;
    Serial.println("LSM9DS1 not found. Continuing without IMU.");
  }

  syncSerial.begin(UART_SYNC_BAUD, SERIAL_8N1, UART_SYNC_RX_PIN, UART_SYNC_TX_PIN);

  BLEDevice::init(BLE_DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(svcUUID);
  bleCmdChar = service->createCharacteristic(
      cmdUUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleCmdChar->setCallbacks(new CmdCallbacks());

  bleImuChar = service->createCharacteristic(
      imuUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  if (bleImuChar) {
    auto* cccd = new BLE2902();
    cccd->setNotifications(true);
    bleImuChar->addDescriptor(cccd);
  }

  bleStatsChar = service->createCharacteristic(
      statsUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  if (bleStatsChar) {
    auto* statsCccd = new BLE2902();
    statsCccd->setNotifications(true);
    bleStatsChar->addDescriptor(statsCccd);
    bleStatsChar->setValue("active=0");
  }

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(svcUUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.printf("BLE advertising as '%s'\n", BLE_DEVICE_NAME);

  logQueue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(esp32_log_record_t));
  if (!logQueue) {
    Serial.println("[log] Failed to allocate log queue");
  }

  xTaskCreatePinnedToCore(acquisitionTask, "imu_acq", 4096, nullptr, 3, &acquisitionTaskHandle, 0);
  xTaskCreatePinnedToCore(loggingTask, "log_writer", 4096, nullptr, 2, &loggingTaskHandle, 1);

  lastCmdMillis = millis();
}

void loop() {
  if (millis() - lastCmdMillis > TIMEOUT_MS) {
    if (currentThrottle != 0) {
      currentThrottle = 0;
      motorSetPercent(0);
      Serial.println("Safety stop (timeout)");
    }
    lastCmdMillis = millis();
  }

  static uint32_t lastImuNotify = 0;
  const uint32_t IMU_PERIOD_MS = 40;
  if (bleClientConnected && bleImuChar && (millis() - lastImuNotify >= IMU_PERIOD_MS)) {
    lastImuNotify = millis();
    esp32_log_record_t sample;
    bool haveSample = false;
    portENTER_CRITICAL(&sampleMux);
    if (g_lastSampleValid) {
      sample = g_lastSample;
      haveSample = true;
    }
    portEXIT_CRITICAL(&sampleMux);
    if (haveSample) {
      uint8_t buf[12];
      memcpy(&buf[0], &sample.accel_mg[0], 2);
      memcpy(&buf[2], &sample.accel_mg[1], 2);
      memcpy(&buf[4], &sample.accel_mg[2], 2);
      memcpy(&buf[6], &sample.gyro_cds[0], 2);
      memcpy(&buf[8], &sample.gyro_cds[1], 2);
      memcpy(&buf[10], &sample.gyro_cds[2], 2);
      bleImuChar->setValue(buf, sizeof(buf));
      bleImuChar->notify();
    }
  }

  static uint32_t lastStatsMillis = 0;
  if (millis() - lastStatsMillis >= 1000) {
    lastStatsMillis = millis();
    updateBleStats();
  }

  delay(5);
}

void steeringWriteSafe(int angle) {
  angle = constrain(angle, 0, 180);
  steeringServo.write(angle);
}

void motorSetPercent(int percent) {
  percent = constrain(percent, -100, 100);
  if (percent == 0) {
    motorPwm.write(0);
    return;
  }
  bool fwd = (percent > 0);
  digitalWrite(MOTOR_PHS_PIN, fwd ? HIGH : LOW);
  uint32_t duty = (static_cast<uint32_t>((1U << 8) - 1) * static_cast<uint32_t>(abs(percent))) / 100U;
  motorPwm.write(duty);
}
