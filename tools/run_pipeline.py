#!/usr/bin/env python3
"""Entry point for running ingest + core processing + Monte Carlo."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import pandas as pd

sys.path.append(str(Path(__file__).parent))

from pipeline.ingest import IngestConfig, ingest  # type: ignore  # noqa: E402


def extend_cmd(cmd: list[str], flag: str, values) -> None:
    if values is None:
        return
    if isinstance(values, (list, tuple)):
        if not values:
            return
        cmd.extend([flag, *map(str, values)])
    else:
        cmd.extend([flag, str(values)])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="Binary log file produced by the ESP32 datalogger")
    parser.add_argument("--out", type=Path, default=Path("runs/output"), help="Output directory for processed artifacts")
    parser.add_argument("--alpha", type=float, default=0.0, help="Alpha value for envelope extraction")
    parser.add_argument("--accel-bias", type=float, nargs=3, metavar=("AX", "AY", "AZ"), help="Accelerometer bias (m/s^2)")
    parser.add_argument("--gyro-bias", type=float, nargs=3, metavar=("GX", "GY", "GZ"), help="Gyro bias (rad/s)")
    parser.add_argument("--lever-arm", type=float, nargs=3, metavar=("LX", "LY", "LZ"), help="Lever arm IMU->CG (m)")
    parser.add_argument("--velocity-damping", type=float, default=0.0, help="Velocity damping coefficient (1/s)")
    parser.add_argument("--monte-carlo", type=int, default=0, help="Monte Carlo sample count (0 disables)")
    parser.add_argument("--mc-seed", type=int, default=42, help="Monte Carlo RNG seed")
    parser.add_argument("--mc-accel-sigma", type=float, default=0.0, help="Monte Carlo accel bias sigma (m/s^2)")
    parser.add_argument("--mc-gyro-sigma", type=float, default=0.0, help="Monte Carlo gyro bias sigma (rad/s)")
    parser.add_argument("--mc-lever-sigma", type=float, default=0.0, help="Monte Carlo lever arm sigma (m)")
    parser.add_argument("--no-crc", action="store_true", help="Skip CRC validation during ingest")
    parser.add_argument("--friction-mu", type=float, default=0.0, help="Static friction coefficient for validation")
    parser.add_argument("--camera-meta", type=Path, help="Path to ESP32-CAM meta.csv (auto-detected if omitted)")
    parser.add_argument("--camera-root", type=Path, help="Root directory containing camera frames")
    parser.add_argument("--use-vio", action="store_true", help="Enable pseudo VIO output")
    parser.add_argument("--vio-trajectory", type=Path, help="Output CSV path for VIO trajectory")
    parser.add_argument("--compare-envelope", type=Path, help="Reference envelope CSV for Hausdorff distance")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    parquet_path = args.out / "dataset.parquet"

    ingest_config = IngestConfig(
        log_path=args.log,
        output_path=parquet_path,
        apply_crc_validation=not args.no_crc,
    )
    ingest(ingest_config)
    print(f"[ingest] wrote {parquet_path}")

    frame = pd.read_parquet(parquet_path)
    csv_path = args.out / "dataset.csv"
    frame.to_csv(csv_path, index=False)
    print(f"[convert] wrote {csv_path}")

    repo_root = Path(__file__).resolve().parent.parent
    exe_name = "gg_pipeline_cli.exe" if os.name == "nt" else "gg_pipeline_cli"
    cli_path = repo_root / "build" / exe_name
    if not cli_path.exists():
        raise SystemExit(f"Pipeline CLI not found: {cli_path}. Build the project (cmake --build build) first.")

    cmd: list[str] = [
        str(cli_path),
        "--input", str(csv_path),
        "--output-json", str(args.out / "results.json"),
        "--envelope-csv", str(args.out / "envelope.csv"),
        "--steady-csv", str(args.out / "steady_points.csv"),
        "--alpha", str(args.alpha),
    ]

    extend_cmd(cmd, "--accel-bias", args.accel_bias)
    extend_cmd(cmd, "--gyro-bias", args.gyro_bias)
    extend_cmd(cmd, "--lever-arm", args.lever_arm)
    if args.velocity_damping:
        extend_cmd(cmd, "--velocity-damping", args.velocity_damping)
    if args.friction_mu:
        extend_cmd(cmd, "--friction-mu", args.friction_mu)
    if args.monte_carlo:
        extend_cmd(cmd, "--monte-carlo", args.monte_carlo)
        extend_cmd(cmd, "--mc-seed", args.mc_seed)
        extend_cmd(cmd, "--mc-accel-sigma", args.mc_accel_sigma)
        extend_cmd(cmd, "--mc-gyro-sigma", args.mc_gyro_sigma)
        extend_cmd(cmd, "--mc-lever-sigma", args.mc_lever_sigma)
    extend_cmd(cmd, "--processed-csv", args.out / "processed_points.csv")

    # Camera dataset auto-detection
    camera_meta = args.camera_meta
    if camera_meta is None:
        candidate = args.log.parent / "cam" / "meta.csv"
        if candidate.exists():
            camera_meta = candidate
    if camera_meta is None:
        candidate = args.log.parent / "meta.csv"
        if candidate.exists():
            camera_meta = candidate

    if camera_meta:
        extend_cmd(cmd, "--camera-meta", camera_meta)
        camera_root = args.camera_root if args.camera_root else camera_meta.parent
        extend_cmd(cmd, "--camera-root", camera_root)
        if args.use_vio:
            cmd.append("--use-vio")
            output_traj = args.vio_trajectory if args.vio_trajectory else args.out / "trajectory.csv"
            extend_cmd(cmd, "--vio-trajectory", output_traj)

    if args.compare_envelope:
        extend_cmd(cmd, "--compare-envelope", args.compare_envelope)

    print(f"[pipeline_cli] {' '.join(map(str, cmd))}")
    subprocess.run(cmd, check=True)
    print(f"[results] Generated {args.out / 'results.json'}")


if __name__ == "__main__":
    main()
