#ifndef ESP32_LOG_FORMAT_H
#define ESP32_LOG_FORMAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Packed telemetry record written by the ESP32-S3 datalogger.
 *
 * All numeric fields use little-endian encoding. Timestamps originate from the
 * ESP32 hardware timer running at 1 MHz (microsecond resolution). Accelerometer
 * samples are stored in milli-g (mg), gyro samples in centi-deg/s (0.01 dps).
 */
typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;   /**< Microseconds since timer start */
    int16_t accel_mg[3];     /**< Body-frame acceleration XYZ in milli-g */
    int16_t gyro_cds[3];     /**< Body-frame angular rate XYZ in 0.01 deg/s */
    int16_t throttle_pct;    /**< Commanded throttle percent (-100..100) */
    uint16_t steering_deg;   /**< Commanded steering angle in degrees (0..180) */
    uint16_t vbat_mv;        /**< Battery voltage in millivolts */
    uint8_t flags;           /**< Flag bits (see ESP32_LOG_FLAG_*) */
    uint8_t sequence;        /**< Monotonic sequence counter (wraps at 255) */
    uint32_t crc32;          /**< CRC-32 (IEEE 802.3) over first 28 bytes */
} esp32_log_record_t;

#define ESP32_LOG_FLAG_SYNC               0x01  /**< PPS edge marker */
#define ESP32_LOG_FLAG_SESSION_START      0x02  /**< Session start marker */
#define ESP32_LOG_FLAG_SESSION_STOP       0x04  /**< Session stop marker */
#define ESP32_LOG_FLAG_FAULT              0x08  /**< Fault condition recorded */
#define ESP32_LOG_FLAG_DROPPED_SENSOR     0x10  /**< IMU sample dropped before write */
#define ESP32_LOG_FLAG_BUFFER_OVERRUN     0x20  /**< Logger buffer overflow */

#define ESP32_LOG_RECORD_SIZE_BYTES       32

/* Compile-time validation that packing is preserved */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(esp32_log_record_t) == ESP32_LOG_RECORD_SIZE_BYTES,
               "esp32_log_record_t must remain 32 bytes");
#endif

#ifdef __cplusplus
}
#endif

#endif /* ESP32_LOG_FORMAT_H */
