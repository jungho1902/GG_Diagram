# Vision-Inertial Integration Plan

## Library Selection
- **OpenVINS** chosen as the baseline VIO estimator.
  - Mature C++ library with BSD license, supports offline dataset replay.
  - Provides tight coupling of IMU+camera with parameterizable noise/initialization.
  - Dependencies: Eigen3, OpenCV, `yaml-cpp`, optional ROS2 wrappers. We integrate the standalone C++ core to avoid ROS.
- **Justification**: matches requirements from `GG_Diagram.txt` Step 2 (tightly coupled EKF, lever-arm handling). Alternatives (VINS-Fusion, Kimera-VIO) require heavier ROS tooling.

## Calibration Strategy
1. **Intrinsic calibration**: use OpenCV `calib3d` tooling (chessboard) to produce `camera_intrinsics.yaml`. Store in `calibration/esp32_cam_intrinsics.yaml`.
2. **Extrinsic calibration (IMU ↔ camera)**: perform hand-eye calibration with AprilTag board and record synchronized motion. Use OpenVINS `ov_calibration` utility; export as `calibration/imu_cam_extrinsics.yaml`.
3. **IMU noise parameters**: derive from static Allan variance test; store in `calibration/imu_noise.yaml` (already partially covered by analytics module).
4. **Consolidated config**: new file `calibration/openvins_config.yaml` consumed by the pipeline CLI; references the three YAML blobs above.

## Data Pipeline Updates
- Extend binary ingestion to also import ESP32-CAM `meta.csv` and link frames via PPS timestamp.
- New directory layout per run:
  ```
  runs/<date>/<label>/
    raw/
      imu.bin
      cam/
        frame_XXXXXX.pgm
        meta.csv
    processed/
      dataset.parquet
      dataset.csv
      openvins/
        trajectory.csv
        states.json
  ```
- Provide synchronization module that merges IMU and frame timestamps before feeding OpenVINS.

## Integration Steps
1. **Build OpenVINS core as external project** (FetchContent) producing `libopenvins.a`.
2. **Create `vision/vio_runner.cpp`** that:
   - Parses combined dataset.
   - Configures OpenVINS with calibration YAML.
   - Produces state estimates (`timestamp`, `position`, `velocity`, Euler angles, biases).
   - Saves to `processed/openvins/trajectory.csv`.
3. **Adapt `gg_pipeline_cli`** to accept optional VIO output path; when present, read CG accelerations from VIO-estimated attitude & velocity instead of pure IMU integration.
4. **Validation metrics** per Step 3:
   - Compute `R²` between measured lateral acceleration and `v_x * yaw_rate` using VIO velocity/yaw.
   - Compute Hausdorff distance between envelopes of multiple runs (store baseline in run metadata, compare with new run).
   - Enforce friction circle constraint using measured `μ_s` (from calibration metadata) and flag violations.
5. **ISO GUM Monte Carlo**:
   - Parameter priors from calibration YAML (mean ± std).
   - Wrap run pipeline to sample parameters, run OpenVINS + dynamics, and produce 95% confidence intervals for `Gx_max`, `Gy_max`, envelope area.

## Reporting
- Generate Markdown report per run summarizing:
  - Run metadata (date, track, tyres).
  - Quality metrics, VIO consistency (R², Hausdorff, friction check).
  - Monte Carlo stats with CIs.
  - Plots: G-G diagram, velocity/yaw comparison, Allan deviation (links to existing SVG exports).
- Use Pandoc (optional) to convert Markdown → PDF.

## Next Actions
1. Vendor OpenVINS, create CMake target, and validate build.
2. Implement dataset merger & VIO replay runner.
3. Extend CLI to incorporate validation metrics + ISO GUM style MC.
4. Build reporting script.
