#include <Arduino.h>
#include <esp_camera.h>
#include <FS.h>
#include <SD_MMC.h>
#include <driver/rtc_io.h>
#include <esp_timer.h>

#include <cmath>
#include <cstring>
#include <cstddef>

// Camera model pin mapping (AI-Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

constexpr int PPS_GPIO = 33;              // Rising edge PPS from ESP32-S3
constexpr int SYNC_UART_RX = 13;          // UART RX for timestamp packets (adjust as wired)
constexpr int SYNC_UART_TX = 12;          // Not used
constexpr uint32_t SYNC_BAUD = 115200;

constexpr uint32_t FRAME_INTERVAL_US = 33333;  // ~30 fps

static portMUX_TYPE syncMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t gLastSyncTimestamp = 0;

static portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t gLastPpsLocalTime = 0;
static volatile uint64_t gLastPpsSyncTime = 0;
static volatile bool gPpsHasSync = false;

static File metaFile;
static char sessionDir[64];
static uint32_t frameCounter = 0;
static uint64_t nextFrameDeadlineUs = 0;

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

void IRAM_ATTR handlePps() {
  uint64_t local = static_cast<uint64_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&ppsMux);
  gLastPpsLocalTime = local;
  gLastPpsSyncTime = gLastSyncTimestamp;
  gPpsHasSync = (gLastSyncTimestamp != 0);
  portEXIT_CRITICAL_ISR(&ppsMux);
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;      // unused for grayscale
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%X\n", err);
    while (true) {
      delay(1000);
    }
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_framesize(sensor, FRAMESIZE_QVGA);
    sensor->set_pixformat(sensor, PIXFORMAT_GRAYSCALE);
  }
}

void ensureMetaFile() {
  if (metaFile) {
    return;
  }
  char path[96];
  snprintf(path, sizeof(path), "%s/meta.csv", sessionDir);
  metaFile = SD_MMC.open(path, FILE_APPEND);
  if (!metaFile) {
    Serial.println("Failed to open metadata file");
    return;
  }
  if (metaFile.size() == 0) {
    metaFile.println("frame,pps_timestamp_us,local_capture_us,offset_us,path");
    metaFile.flush();
  }
}

bool writeFrame(const camera_fb_t* fb, uint64_t captureLocalUs, uint64_t ppsSyncUs, uint64_t ppsLocalUs, bool ppsValid) {
  char path[96];
  snprintf(path, sizeof(path), "%s/frame_%06lu.pgm", sessionDir, static_cast<unsigned long>(frameCounter));

  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open frame file");
    return false;
  }

  file.printf("P5\n%d %d\n255\n", fb->width, fb->height);
  file.write(fb->buf, fb->len);
  file.close();

  ensureMetaFile();
  if (metaFile) {
    uint64_t offset = (ppsValid && captureLocalUs >= ppsLocalUs) ? (captureLocalUs - ppsLocalUs) : 0;
    metaFile.printf("%lu,%llu,%llu,%llu,%s\n",
                    static_cast<unsigned long>(frameCounter),
                    static_cast<unsigned long long>(ppsValid ? ppsSyncUs : 0),
                    static_cast<unsigned long long>(captureLocalUs),
                    static_cast<unsigned long long>(offset),
                    path);
    metaFile.flush();
  }
  return true;
}

void initStorage() {
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD_MMC mount failed");
    while (true) {
      delay(1000);
    }
  }

  uint64_t tag = static_cast<uint64_t>(esp_timer_get_time());
  snprintf(sessionDir, sizeof(sessionDir), "/sdcard/run_%llu", static_cast<unsigned long long>(tag));
  if (!SD_MMC.exists(sessionDir)) {
    if (!SD_MMC.mkdir(sessionDir)) {
      Serial.println("Failed to create session directory");
      while (true) {
        delay(1000);
      }
    }
  }
}

void processSyncSerial() {
  enum class State { WaitHeader1, WaitHeader2, Body };
  static State state = State::WaitHeader1;
  static uint8_t buffer[12];
  static std::size_t index = 0;

  while (Serial1.available()) {
    int byte = Serial1.read();
    if (byte < 0) {
      continue;
    }
    uint8_t value = static_cast<uint8_t>(byte);

    switch (state) {
      case State::WaitHeader1:
        if (value == 0xAA) {
          buffer[0] = value;
          state = State::WaitHeader2;
        }
        break;
      case State::WaitHeader2:
        if (value == 0x55) {
          buffer[1] = value;
          index = 2;
          state = State::Body;
        } else {
          state = State::WaitHeader1;
        }
        break;
      case State::Body:
        buffer[index++] = value;
        if (index >= sizeof(buffer)) {
          uint16_t crc = buffer[10] | (static_cast<uint16_t>(buffer[11]) << 8);
          uint16_t computed = crc16_ccitt(buffer, 10);
          if (crc == computed) {
            uint64_t timestamp = 0;
            std::memcpy(&timestamp, &buffer[2], sizeof(uint64_t));
            portENTER_CRITICAL(&syncMux);
            gLastSyncTimestamp = timestamp;
            portEXIT_CRITICAL(&syncMux);
          }
          state = State::WaitHeader1;
          index = 0;
        }
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32-CAM PPS Logger");

  initStorage();
  setupCamera();

  pinMode(PPS_GPIO, INPUT_PULLUP);
  attachInterrupt(PPS_GPIO, handlePps, RISING);

  Serial1.begin(SYNC_BAUD, SERIAL_8N1, SYNC_UART_RX, SYNC_UART_TX, false);

  frameCounter = 0;
  nextFrameDeadlineUs = static_cast<uint64_t>(esp_timer_get_time());
}

void loop() {
  processSyncSerial();

  uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
  if (nowUs < nextFrameDeadlineUs) {
    delayMicroseconds(500);
    return;
  }
  nextFrameDeadlineUs += FRAME_INTERVAL_US;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to acquire frame buffer");
    return;
  }

  uint64_t captureLocalUs = static_cast<uint64_t>(esp_timer_get_time());
  uint64_t ppsSyncUs = 0;
  uint64_t ppsLocalUs = 0;
  bool ppsValid = false;

  portENTER_CRITICAL(&ppsMux);
  ppsSyncUs = gLastPpsSyncTime;
  ppsLocalUs = gLastPpsLocalTime;
  ppsValid = gPpsHasSync;
  portEXIT_CRITICAL(&ppsMux);

  if (writeFrame(fb, captureLocalUs, ppsSyncUs, ppsLocalUs, ppsValid)) {
    ++frameCounter;
  }

  esp_camera_fb_return(fb);
}
