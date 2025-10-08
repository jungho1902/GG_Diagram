# G-G Diagram Processing & Visualization

A modular C++17 pipeline for post-processing race car IMU data into a G-G diagram, instrumented with a Dear ImGui GUI. The implementation follows Sections 5, 8, and 9 of the provided HTML specification and adds diagnostics (sampling quality, Allan deviation, Monte Carlo perturbations) to support calibration and validation. The vision/EKF portions (Sections 6–7) and full uncertainty/reporting stack (Sections 10–11) are not yet integrated; details are listed in the _Remaining Specification Items_ section.

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
- **GgEnvelope** – Monotone chain convex hull of steady-state Gx/Gy points.
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

---

## GUI Walkthrough

### 1. Simulation Controls
- **Start/Stop Simulation** – Toggle synthetic IMU generator.
- **Reset Data** – Clears history, analytics, Monte Carlo results, and resets states.
- **Use Dataset Playback** – Switch between synthetic stream and loaded CSV.
- **Dataset CSV Path / Load Dataset** – Parse and load IMU dataset; automatically resets pipeline for dataset playback.
- **Playback Speed** – Adjust playback speed multiplier for dataset mode.
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
- Red polyline: convex hull (base envelope).
- Optional yellow dots: Monte Carlo steady-state samples (if show toggle active).
- Gold polyline: Monte Carlo aggregate hull.
- Gridlines and axis ticks with numeric labels (units in g).

### 4. Analytics Panel
- **Compute Analytics** button runs quality metrics, time offset estimation, and Allan deviation on the active dataset (synthetic or CSV).
- Quality Metrics table: sample count, mean Δt, nominal Hz, jitter, missing-data rate, saturation rates.
- Time Synchronization: best lag converted to seconds and peak correlation.
- Allan Deviation tables: `tau [s], σx, σy, σz` for accelerometer and gyroscope axes.
- Monte Carlo summary: envelope point count, accepted sample count, and latest status text.

### 5. SVG Export
- Builds an 800×800 SVG with axes, ticks, labels, processed/steady/Monte Carlo samples, base hull, and Monte Carlo hull.
- Uses current `tickCount_` and data overlays from GUI state.
- Command: click “Export SVG” in Parameters panel; path from `SVG Output File` field.

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

### Monte Carlo Sensitivity (Partial Section 11)
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
| Envelope extraction (convex hull) | ✅ | Visual + CSV/SVG outputs.
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

### 6–7) Vision & EKF Integration (Not Implemented)
- Needs synchronized camera logs, calibration (intrinsic/extrinsic), and libs (OpenCV, Eigen, possibly Sophus/GTSAM).
- Includes keypoint extraction, Essential matrix, relative pose, IMU-camera state propagation, EKF updates.

### 10) System-Level Validation
- Requires additional sensors/metrics: reference velocity/yaw, tire tests.
- Includes kinematic consistency (`a_y ≈ v_x * r`), run-to-run repeatability (Hausdorff distance), tire friction comparison.

### 11) Uncertainty Analysis (Partial)
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

