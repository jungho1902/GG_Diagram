# G-G Diagram Processing & Visualization

A modular C++17 pipeline for post-processing race car IMU data into a G-G diagram, instrumented with a Dear ImGui GUI. The implementation tracks Steps 1–3 of the `GG_Diagram.txt` IEEEtran specification (sensor preparation, inertial processing, and alpha-shape envelope extraction) and adds diagnostics (sampling quality, Allan deviation, Monte Carlo perturbations) to support calibration and validation. The tightly-coupled vision/EKF portions in Step 2 and the full validation/uncertainty stack from Step 3 remain to be integrated; see _Remaining Specification Items_.

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
cmake --build build
./build/gg_gui
```
- Requires a C++17 compiler, GLFW3, GLEW, OpenGL, and ImGui (fetched automatically via CMake FetchContent).
- Optional dataset CSV must contain rows of `timestamp, ax, ay, az, gx, gy, gz` (seconds, m/s², rad/s), with header lines automatically ignored.

## GG_Diagram.txt 컴파일

`GG_Diagram.txt`는 IEEEtran 템플릿 기반 LaTeX 문서입니다. pdfLaTeX로 빌드하면 코드 구현과 동일한 용어 체계를 갖춘 참조 문서를 얻을 수 있습니다.

```bash
pdflatex GG_Diagram.txt
bibtex GG_Diagram
pdflatex GG_Diagram.txt
pdflatex GG_Diagram.txt
```

- Windows 환경에서는 TeX Live 혹은 MiKTeX 설치 후 동일한 명령을 사용할 수 있습니다.
- 출력물은 Step 1–3 전 과정을 상세히 설명하므로 코드 수정 시 반드시 최신 문서와 대조하세요.

---

## GUI Walkthrough

### 1. Simulation Controls
- **Start/Stop Simulation** – Toggle synthetic IMU generator.
- **Reset Data** – Clears history, analytics, Monte Carlo results, and resets states.
- **Use Dataset Playback** – Switch between synthetic stream and loaded CSV.
- **Dataset CSV Path / Load Dataset** – Parse and load IMU dataset; automatically resets pipeline for dataset playback.
- **Playback Speed** – Adjust playback speed multiplier for dataset mode.
- **Alpha Detail (1/g)** – Controls the alpha-shape boundary tightness: 0 = convex hull, increasing values preserve concave features from Step 3 of `GG_Diagram.txt`.
- Displays current Gx/Gy (g-units), processed count, and accepted steady-state count.

### 2. Parameters Panel
Groups of editable parameters (all changes apply immediately):

| Group | Fields | Notes |
| --- | --- | --- |
| IMU Bias | `Accel Bias (XYZ)` | m/s² offsets added to raw accelerometer readings.
|  | `Gyro Bias (XYZ)` | rad/s offsets added to gyro input.
| Lever Arm | `Lever Arm (XYZ)` | IMU position relative to CG (body frame, meters).
| Gravity | `Gravity Vector (XYZ)` | Default [0,0,-9.80665]; edit for local gravity.
| Integration | `Velocity Damping` | 1/s exponential decay applied to longitudinal velocity integration.
| Quality/Analytics | `Accel Saturation`, `Gyro Saturation` | Thresh used in quality metrics.
|  | `Time Sync Max Lag` | Max lag in cross-correlation (samples).
|  | `Allan Max Tau`, `Allan Steps` | Averaging interval sweep for Allan deviation.
| Monte Carlo | `Samples`, `Accel Bias Sigma`, `Gyro Bias Sigma`, `Lever Arm Sigma`, `Random Seed` | Configure Gaussian perturbations; set samples > 0 to enable runs.
| Monte Carlo Control | `Run Monte Carlo`, `Show samples` | Executes simulation and toggles accepted-point overlay.
| Export | `SVG Output File` | Path used by “Export SVG” button.

Every parameter change refreshes processed history (when possible) and clears analytics/Monte Carlo status messages so new results reflect the current configuration.

### 3. G-G Diagram Panel
- Gray dots: all processed Gx/Gy samples.
- Blue dots: steady-state samples passing the filter.
- Red polyline: alpha-shape envelope using the current alpha setting (falls back to convex hull at 0).
- Optional yellow dots: Monte Carlo steady-state samples (if show toggle active).
- Gold polyline: Monte Carlo alpha-shape envelope (shares the same alpha parameter).
- Gridlines and axis ticks with numeric labels (units in g).

### 4. Analytics Panel
- **Compute Analytics** button runs quality metrics, time offset estimation, and Allan deviation on the active dataset (synthetic or CSV).
- Quality Metrics table: sample count, mean Δt, nominal Hz, jitter, missing-data rate, saturation rates.
- Time Synchronization: best lag converted to seconds and peak correlation.
- Allan Deviation tables: `tau [s], σx, σy, σz` for accelerometer and gyroscope axes.
- Monte Carlo summary: envelope point count, accepted sample count, and latest status text.

### 5. SVG Export
- Builds an 800×800 SVG with axes, ticks, labels, processed/steady/Monte Carlo samples, and both base/Monte Carlo alpha-shape envelopes (공유 알파 파라미터 사용).
- Uses current `tickCount_` and data overlays from GUI state.
- Command: click “Export SVG” in Parameters panel; path from `SVG Output File` field.

---

## 스펙 매핑 개요

- **Step 1 – 사전 계측/보정**: IMU 바이어스·레버암·중력 벡터 편집과 Monte Carlo 퍼터베이션이 코드에 반영되어 있습니다.
- **Step 2 – 관성/시각 융합**: 현재 구현은 Mahony 기반 AHRS 및 레버암 보정까지만 포함하며, `GG_Diagram.txt` Step 2 후반(비전/VIO + EKF)은 미구현 상태입니다.
- **Step 3 – G-G 다이어그램 생성/검증**: 알파셰이프 경계, quasi-steady 필터, 샘플링 품질·Allan·Monte Carlo 분석이 구현되어 있습니다. 시스템 수준 검증 지표와 ISO GUM 보고서는 진행 중입니다.

---

## 데이터 준비 가이드

`GG_Diagram.txt` Step 1 권장 절차를 따르면 다음 자료를 준비해두세요.

1. **IMU 설치/보정**: 센서 축과 차량 바디 프레임 정렬, 레버암 측정, 바이어스 추정치를 README 기본값 또는 별도 설정 파일에 기록.
2. **데이터 로깅**: `timestamp, ax, ay, az, gx, gy, gz` 형식(초, m/s², rad/s)을 유지합니다. 헤더나 주석은 자동으로 무시되지만 결측치는 사전에 제거하는 것이 좋습니다.
3. **환경 메타데이터**: 중력 벡터(위도·고도 반영), 노면 상태, 타이어 온도 등 Step 3 검증에서 활용할 정보를 별도 로그로 남기세요.
4. **비전 데이터(선택)**: Step 2 확장을 위해서는 동기화된 카메라 로그와 내·외부 캘리브레이션 파일이 필요합니다.

---

## Analytics Pipeline Details

### Sampling Quality & Synchronization
- **Mean Δt / Nominal rate**: average sampling interval and derived frequency.
- **Jitter**: standard deviation of Δt.
- **Missing data rate**: proportion of intervals > 1.5× mean.
- **Saturation rate**: fraction of samples exceeding user thresholds (per axis max magnitude).
- **Time offset**: discrete cross-correlation between lateral acceleration and `Vx * yaw_rate` (quasi steady-state lateral acceleration model) up to `maxLagSamples`. Output is lag (s) and correlation peak.

### Allan Deviation
- Uses log-spaced averaging intervals (`tau`) up to user-defined `allanMaxTau` with `allanSteps` points.
- Applies classic two-sample variance for `τ` windows: σ(τ) = sqrt(½(k-1) Σ (avgᵢ₊₁ - avgᵢ)²).
- Separate tables for accelerometer and gyroscope axes.

### Monte Carlo Sensitivity (Step 3 – partial)
- Runs `samples` iterations, each perturbing IMU bias/lever arm by Gaussian noise (σ’s configurable).
- Recomputes pipeline per perturbation, collecting accepted points and envelope.
- Aggregated hull shown in GUI and SVG; sample overlay optional.
- **Note**: full uncertainty reporting (confidence intervals on maxima, etc.) is not yet implemented.

---

## Feature Summary

| Feature | Status | Notes |
| --- | --- | --- |
| Synthetic IMU generator | ✅ | Deterministic scenario with launch/turn/brake phases.
| Attitude & CG acceleration pipeline | ✅ | Supports bias, lever arm, gravity editing.
| Quasi-steady-state filtering | ✅ | Jerk / angular accel / RMS thresholds exposed.
| Envelope extraction (alpha-shape) | ✅ | Tunable alpha per Step 3; 0 ⇒ convex hull, >0 retains concavity; SVG export updated.
| Dataset ingestion (CSV) | ✅ | Unit: seconds, m/s², rad/s. Header ignored.
| Sampling quality metrics | ✅ | Mean Δt, jitter, gap / saturation rates.
| Time offset estimation | ✅ | Cross-correlation of lateral accel vs vx·yaw.
| Allan deviation | ✅ | Adjustable τ range and resolution.
| Monte Carlo perturbations | ✅ | Bias/lever-arm Gaussian perturbations; results plotted/exported.
| SVG export | ✅ | Includes overlays, grid, and Monte Carlo data.
| Vision-based fusion, EKF | ❌ | Requires camera data, calibration, OpenCV/Eigen.
| Tire limit comparison | ❌ | Needs tire data and validation framework.
| ISO GUM / full uncertainty reporting | ⚠️ Partial | Monte Carlo samples available, but CI calculations/reporting pending.
| Automated report generation | ❌ | To be designed once full pipeline integrated.

---

## Remaining Specification Items & Requirements

### Step 2) Vision & EKF Integration (Not Implemented)
- Needs synchronized camera logs, calibration (intrinsic/extrinsic), and libs (OpenCV, Eigen, possibly Sophus/GTSAM).
- Includes keypoint extraction, Essential matrix, relative pose, IMU-camera state propagation, EKF updates.

### Step 3) System-Level Validation
- Requires additional sensors/metrics: reference velocity/yaw, tire tests.
- Includes kinematic consistency (`a_y ≈ v_x * r`), run-to-run repeatability (Hausdorff distance), tire friction comparison.

### Step 3) Uncertainty Analysis (Partial)
- ISO GUM propagation, covariance estimation from Allan noise parameters.
- Monte Carlo with full pipeline re-execution and reporting (CI for max Gx/Gy, etc.). Current Monte Carlo handles perturbations but lacks statistical summarization.

### Reporting & Data Imports
- Need structured configuration (JSON/INI) for lever arm, noise covariances, tire data.
- Automated tables/plots for final reports.

To progress on these sections, provide: synchronized IMU+camera datasets, calibration files, noise parameter goals, tire benchmarks, and agree on library stack. Once inputs and priorities are set, the remaining stages can be implemented incrementally.

---

## Suggested Next Steps
1. **Vision/EKF Scope** – Decide target VIO approach (custom EKF vs. existing frameworks), supply sensor calibrations and dataset examples.
2. **Validation Data** – Collect tire limit measurements, repeated test runs for Hausdorff distance, reference velocity sources for kinematic checks.
3. **Uncertainty Framework** – Define parameters and distributions, desired outputs (CI, histograms) for Monte Carlo reports.
4. **Config/Reporting** – Standardize configuration files and automated report generation once the above data flow is established.
5. **Testing & Documentation** – Add regression tests for pipeline permutations, expand README with dataset instructions once the pipeline is complete.

---

For any additional functionality (vision fusion, ISO GUM calculations, automated reports), please supply the relevant datasets, calibration specs, and preferred tooling. This README will be updated as those components are implemented.
