# Hardware Test Checklist (ESP32 RC Car + ESP32-CAM)

## Pre-Test Preparation
- **Main firmware**: compile `ESP32_RC_CAR_BLE.ino` (Arduino IDE / PlatformIO) with correct pin map (`SD_CS_PIN`, battery ADC). Verify serial output shows `SD card ready` and `LSM9DS1 ready`.
- **Camera firmware**: flash `ESP32_CAM_PPS_Logger.ino`, confirm PPS (GPIO33) and UART RX wiring, and check `/sdcard/run_*/meta.csv` is created on boot.
- **SD cards**: format FAT32, create `/logs` on ESP32-S3 card; ESP32-CAM card auto-creates `/sdcard/run_<boot_tag>/`.
- **Power/battery**: charge traction battery, inspect wiring (motor enable, servo), keep kill switch accessible.
- **Calibration blobs**: load latest `calib_v*.yaml` into SPIFFS (bias/scale, lever-arm) and sync the same values with the PC processing config (pass via CLI arguments or YAML).

## Wiring Sanity Checks
- PPS GPIO9 -> ESP32-CAM GPIO33 (verify clean 3.3 V pulse on scope).
- UART1 TX (GPIO17) -> ESP32-CAM UART0 RX; confirm idle high.
- Motor enable/direction lines connected, servo operates freely.
- LSM9DS1 I2C lines secure; measure 3.3 V supply.

## Boot & Connectivity
- Power on car; monitor serial output at 115200 baud.
- Ensure BLE advertisement `RC-DRIFT-XIAO` visible (phone or PC scanner).
- Run `tools/session_cli.py --scan` (see below) to locate MAC address; note for control script.

## Dry-Run Checks (no motion)
1. Send `RUN:testbench`, `LOG:START`, `LOG:STOP` over BLE; confirm `/logs` gains new file.
2. Download file via USB or SD reader; run `tools/run_pipeline.py <file> --out runs/smoke --monte-carlo 10` to validate CRC and pipeline.
3. Confirm PPS records present in log (flags bit 0x01); run `./build/gg_pipeline_cli --input runs/smoke/dataset.csv --output-json runs/smoke/results.json --steady-csv ...` and ensure JSON contains `steady_point_count > 0`.
4. Check BLE stats characteristic via `tools/session_cli.py --device <MAC> --stats`: `active=0` when idle, queue/overrun counters remain zero.

## Dynamic Test Flow
1. Position car on stands; use CLI to send throttle/steering pulses (e.g., `T:20`, `S:90`) ensuring wheels respond.
2. When ready, issue `RUN:<label>` (e.g., `RUN:dry_lane1`) then `LOG:START` and begin drive sequence.
3. Monitor `stats` notifications for buffer overruns; if triggered, reduce log rate or investigate SD speed.
4. After run, send `LOG:STOP`, power down, remove SD card, copy log to PC under `runs/<date>/<label>/raw/`.
5. Execute full pipeline: `tools/run_pipeline.py RAW.bin --out runs/<date>/<label> --alpha 0.25 --friction-mu 1.2 --monte-carlo 500 --use-vio`; review `results.json` (validation/MC stats), `steady_points.csv`, `envelope.csv`, `trajectory.csv`, and archive outputs alongside camera frames.

## Post-Test Tasks
- Archive raw logs + pipeline outputs with metadata (track temp, tyre notes).
- Update calibration if IMU bias drift observed; reflash SPIFFS as needed.
- File Git issues for any firmware anomalies (queue overruns, sensor drops, PPS missing).
- Recharge batteries, inspect drivetrain, prep for next session.
