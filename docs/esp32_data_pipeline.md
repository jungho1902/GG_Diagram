# ESP32 RC Car Data & Sync Architecture

## Signal Synchronisation
- **Master clock**: XIAO ESP32-S3 general-purpose timer 0 at 1 MHz produces the authoritative `timestamp_us` used in every log record.
- **PPS output**: a dedicated GPIO (`GPIO9` recommended) toggles high for 10 µs once per second. The rising edge feeds the ESP32-CAM and an optional debug LED.
- **UART sync burst**: immediately after each PPS rising edge, the ESP32-S3 pushes a 12-byte packet on `UART1` (`0xAA55` header + 64-bit timestamp + CRC-16`). The ESP32-CAM latches the packet so each captured frame stores the microsecond timer value associated with the PPS edge.
- **ESP32-CAM framing**: frames embed the last PPS timestamp plus a local microsecond delta (camera XTAL reference). The PC applies a linear correction using successive PPS pairs to bound skew below 1 ms.

## Firmware Task Split (ESP32-S3)
- **Core 0 (APP CPU)**
  - High-priority timer ISR snapshots IMU samples at 200 Hz.
  - Queues packed `esp32_log_record_t` structs into a lock-free ring buffer.
  - Maintains PPS GPIO toggling and UART sync burst emission.
- **Core 1 (PRO CPU)**
  - Responsible for SD-card writes using a double-buffer strategy (DMA-capable SPI).
  - Manages BLE command/control (teleoperation, health query characteristic).
  - Streams low-rate telemetry (5 Hz) for operator feedback.

## ESP32-CAM Responsibilities
- Receives PPS GPIO edge on `GPIO33` and UART sync packets on `UART0` (115200 baud).
- Captures ROI=QVGA grayscale frames at 30 fps triggered using internal timer aligned to PPS.
- Stores raw frames plus JSON sidecar `{frame_index, pps_timestamp_us, local_offset_us}`.
- Optionally uplinks 160×120 JPEG thumbnails over Wi-Fi for health monitoring.

## Firmware Control Interface (BLE)
- **Service UUID** `12345678-1234-5678-1234-56789abcdef0` exposes three characteristics:
  - `cmd` (write / write without response) — accepts whitespace-separated tokens such as `T:-30`, `S:110`, `LOG:START`, `LOG:STOP`, `RUN:wet_track`.
  - `imu` (read / notify) — 12-byte payload packing accel (mg) and gyro (0.01 dps) for quick-look telemetry at ~25 Hz.
  - `stats` (read / notify) — ASCII summary `active=1,queued=...,` reflecting session state and logging health (queue overruns, sensor drops).
- Safety watchdog halts the motor (sets throttle to 0%) if no command is received for 1.2 s.
- Run labels persist until changed; they are applied to the next `LOG:START` request to form `/logs/<label>_<timestamp>.bin`.

## Log Record Format
- Defined in `firmware/esp32_log_format.h`; each 32-byte packet contains:
  - `timestamp_us` (uint64) — ESP32 1 MHz timer tick.
  - `accel_mg[3]`, `gyro_cds[3]` — accelerometer (milli-g) and gyro (centi-deg/s).
  - `throttle_pct`, `steering_deg`, `vbat_mv` — actuator commands and battery telemetry.
  - `flags`, `sequence`, `crc32` — event markers and CRC-32 guard (IEEE 802.3 polynomial).
- CRC covers the first 28 bytes and is recomputed in the PC ingestion pipeline (`tools/pipeline/log_reader.py`).
- Session events (`SESSION_START`, `SESSION_STOP`, `SYNC`) generate dedicated records with zeroed numeric fields and the relevant flag bit set.

## ESP32-CAM Capture Firmware
- Sketch: `firmware/ESP32_CAM_PPS_Logger.ino`.
- Captures QVGA grayscale frames (~30 fps) using `esp_camera`, timestamps them against the most recent PPS/UART sync packet from the ESP32-S3, and stores raw `.pgm` files plus `meta.csv` (frame index, PPS timestamp, local capture time, offset).
- PPS rising edge (GPIO33) records local timer ticks; UART sync packets (`0xAA55` + 64-bit timestamp + CRC16) deliver the master timestamp. Capture metadata includes both values so the PC pipeline can reconstruct absolute frame times.
- Output directory: `/sdcard/run_<boot_tag>/frame_XXXXXX.pgm` and `/sdcard/run_<boot_tag>/meta.csv`.
- Configure `SYNC_UART_RX` if your wiring differs; defaults assume `Serial1` RX on GPIO13.

## PC Processing Stack
- **Ingest**: `tools/run_pipeline.py` converts binary logs to Parquet/CSV (`dataset.parquet`, `dataset.csv`).
- **Core processing**: `gg_pipeline_cli` (build via `cmake --build build --target gg_pipeline_cli`) consumes the CSV, applies bias/lever-arm corrections, runs quasi-steady filtering, alpha-shape envelope extraction, Allan deviation, validation metrics (R², friction check, optional Hausdorff), pseudo-VIO export (if camera metadata present), and Monte Carlo perturbations with 95% CI. Outputs `results.json`, `steady_points.csv`, `envelope.csv`, `trajectory.csv` (optional), and `processed_points.csv` when requested.
- **CLI invocation example**:
  ```bash
  ./build/gg_pipeline_cli \
      --input runs/test/dataset.csv \
      --output-json runs/test/results.json \
      --envelope-csv runs/test/envelope.csv \
      --steady-csv runs/test/steady_points.csv \
      --alpha 0.25 \
      --accel-bias 0.02 0.01 -0.05 \
      --lever-arm 0.12 0.02 0.05 \
      --friction-mu 1.2 \
      --camera-meta runs/test/raw/cam/meta.csv \
      --camera-root runs/test/raw/cam \
      --use-vio --vio-trajectory runs/test/processed/trajectory.csv \
      --monte-carlo 500 --mc-accel-sigma 0.05 --mc-gyro-sigma 0.005 --mc-lever-sigma 0.01
  ```
- `tools/run_pipeline.py` wraps the above: `tools/run_pipeline.py LOG.bin --out runs/test --alpha 0.25 --friction-mu 1.2 --monte-carlo 500 --use-vio`.
- GUI와 연동: `gg_gui` 실행 후 *Processed Results* 섹션에 `runs/test` 경로를 입력하면 `results.json`/CSV/trajectory를 기반으로 G-G 다이어그램, 검증 지표, 몬테카를로 통계를 즉시 재시각화할 수 있다.

## Event Marking
- **Session start**: BLE command issues `SESSION_START` flag; first logged record after acknowledgement sets `ESP32_LOG_FLAG_SESSION_START`.
- **Session stop**: either BLE stop or safety timeout marks `ESP32_LOG_FLAG_SESSION_STOP`.
- **Fault conditions**: watchdogs (IMU timeout, SD write lag > 10 ms, buffer high-water) set `ESP32_LOG_FLAG_FAULT` along with a fault code stored in an auxiliary TLV record.

## Data Flow Summary
```
IMU (I2C @ 1 MHz) ─► Core0 acquisition ─► Ring buffer ─► Core1 SD writer ─► SD card (binary)
                                        └─► BLE telemetry (5 Hz summary)
PPS Timer ─► GPIO pulse ─► ESP32-CAM EXTI ─► Frame timestamping
          └─► UART sync ─► ESP32-CAM serial parser
```

This layout isolates hard real-time acquisition from communications, keeping jitter well under the 5 ms budget while giving the PC deterministic timestamps across both IMU and camera data streams.
