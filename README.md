# G-G Diagram Processing & Visualization

A modular C++17 pipeline for post-processing race car IMU data into a G-G diagram, instrumented with a Dear ImGui GUI. The implementation tracks Steps 1–3 of the `GG_Diagram.txt` IEEEtran specification (sensor preparation, inertial processing, and alpha-shape envelope extraction) and adds diagnostics (sampling quality, Allan deviation, Monte Carlo perturbations) to support calibration and validation.

> `GG_Diagram.txt` supersedes the older HTML draft. Build it with pdfLaTeX to review the full academic specification alongside the code.

---

## Architecture Overview

```
IMU Data ─► AttitudeEstimator ─► GgProcessor ─► QuasiSteadyStateFilter ─► GgEnvelope
               │                     │                      │
               │                     │                      └─► Analytics (quality, sync, Allan)
               │                     └─► Processed history (Gx/Gy, velocity, yaw)
               └─► Monte Carlo perturbations (optional)
```
- **AttitudeEstimator** – Mahony-style AHRS with accelerometer feedback for estimating orientation quaternions.
- **GgProcessor** – Applies bias removal, transforms acceleration to CG, performs lever-arm correction, integrates velocity (with optional damping), and normalizes into g-units.
- **QuasiSteadyStateFilter** – Sliding window jerk/angular-accel/RMS derivative filter to isolate quasi-steady-state samples.
- **GgEnvelope** – Alpha-shape boundary (tunable `alpha` parameter, 0 ⇒ convex hull) over steady-state Gx/Gy points per Step 3 of `GG_Diagram.txt`.
- **Analytics** – Sampling quality, cross-correlation time offset, Allan deviation, Monte Carlo dispersion.

GUI panels provide control over parameters, analytics, Monte Carlo tests, and export features.

---

## Build & Run

```bash
cmake -S . -B build
cmake --build build --target gg_gui gg_pipeline_cli
./build/gg_gui
```
- Requires a C++17 compiler, GLFW3, GLEW, OpenGL, and ImGui (fetched automatically via CMake FetchContent).
- Optional dataset CSV must contain rows of `timestamp, ax, ay, az, gx, gy, gz` (seconds, m/s², rad/s), with header lines automatically ignored.

### ESP32 Data Pipeline Helpers
- `tools/session_cli.py --scan` — discover the ESP32 car BLE advertisement.
- `tools/session_cli.py --device <MAC> --label dry_run --start-log` — start/stop logging and issue tele-op commands (combine with `--stop-log`, `--throttle`, `--steering`). Requires `pip install bleak`.
- `cmake --build build --target gg_pipeline_cli` — build the headless processor/Monte Carlo executable.
- `tools/run_pipeline.py LOG.bin --out runs/test --alpha 0.25 --friction-mu 1.2 --monte-carlo 500 --use-vio` — ingest raw binary, auto-detect camera metadata, run the C++ pipeline (quality metrics, validation checks, pseudo-VIO export, Monte Carlo with 95% CI), and drop results under `runs/test`.
- `tools/export_csv.py LOG.bin --out run.csv` — convert binary (or `--parquet dataset.parquet`) to CSV for legacy tools or the GUI pipeline.
- `tools/generate_report.py runs/test/results.json --steady runs/test/steady_points.csv --envelope runs/test/envelope.csv --trajectory runs/test/trajectory.csv` — build a Markdown run report summarizing analytics, validation, and Monte Carlo results.
- Ingested Parquet files align with the C++ GUI expectations: columns are `timestamp_us`, `ax_mps2`, `ay_mps2`, `az_mps2`, `gx_rads`, `gy_rads`, `gz_rads`, plus actuator and flag metadata.

---

## GUI Walkthrough

### 1. Simulation Controls
- **Start/Stop Simulation** – Toggle synthetic IMU generator.
- **Reset Data** – Clears history, analytics, Monte Carlo results, and resets states.
- **Use Dataset Playback** – Switch between synthetic stream and loaded CSV.
- **Dataset CSV Path / Load Dataset** – Parse and load IMU dataset; automatically resets pipeline for dataset playback.
- **Processed Results** – Provide a `runs/<label>` directory (containing `results.json`, `steady_points.csv`, etc.) to visualize CLI outputs inside the GUI, including envelope, validation metrics, and Monte Carlo summaries.
- **Playback Speed** – Adjust playback speed multiplier for dataset mode.
- **Alpha Detail (1/g)** – Controls the alpha-shape boundary tightness: 0 = convex hull, increasing values preserve concave features from Step 3 of `GG_Diagram.txt`.
- Displays current Gx/Gy (g-units), processed count, and accepted steady-state count.

### 2. Parameters Panel
- Adjust IMU bias, lever arm, gravity vector, and velocity damping; analytics and Monte Carlo parameters; execute Monte Carlo runs; toggle Monte Carlo sample overlay on the plot.

### 3. G-G Diagram Panel
- Draws processed, steady-state, Monte Carlo points, and both base/Monte Carlo envelopes. Axis ticks/labels autoupdate with the configured range and tick count.

### 4. Analytics Panel
- Computes quality metrics, time synchronization, Allan deviation. Shows loaded validation metrics (R², friction, Hausdorff) and Monte Carlo CI summaries when processed results are loaded.

### 5. SVG Export
- Produces an 800×800 SVG with axes, ticks, labels, processed/steady/Monte Carlo samples, and envelopes.

---

## Data Pipeline Overview

1. **Firmware** – `ESP32_RC_CAR_BLE.ino` logs IMU + actuator data at 200 Hz with PPS/UART sync; `ESP32_CAM_PPS_Logger.ino` captures QVGA frames + timestamp metadata.
2. **Ingest** – `tools/run_pipeline.py` decodes binary logs, outputs Parquet/CSV, and calls `gg_pipeline_cli` for validation + Monte Carlo + pseudo-VIO summary.
3. **Visualization** – `gg_gui` loads either raw CSV datasets or processed result folders (`results.json` + CSVs) for interactive exploration.
4. **Reporting** – `tools/generate_report.py` converts CLI outputs into a Markdown report.

---

## Remaining Scope (per `GG_Diagram.txt`)
- Integrate a real VIO/EKF stack (OpenVINS or equivalent) with calibration YAML.
- Capture calibration data (camera intrinsics/extrinsics, IMU noise) and load into the pipeline.
- Collect validation datasets (friction coefficients, repeated runs) and finalize ISO GUM uncertainty reporting.
- Automate report generation with figures (plots, envelopes, Allan charts) and PDF export.

---

## Changelog (오늘 작업)
- CLI가 camera metadata, 검증 지표(R², 마찰, Hausdorff), pseudo-VIO, Monte Carlo 95% CI를 `results.json`에 추가하도록 확장됨.
- GUI에 처리 결과 디렉터리 로드 기능이 들어가 `gg_gui`에서 CLI/보고서와 동일 정보를 즉시 재현 가능.
- `tools/run_pipeline.py`, `tools/generate_report.py` 등 보조 스크립트 업데이트.

