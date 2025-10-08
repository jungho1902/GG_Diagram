"""Monte Carlo wrapper that drives the C++ CLI for uncertainty analysis."""

from __future__ import annotations

import dataclasses
import json
import os
import subprocess
from pathlib import Path
from typing import Optional

import pandas as pd

from .ingest import IngestConfig, ingest


@dataclasses.dataclass
class MonteCarloConfig:
    base_log: Path
    output_dir: Path
    samples: int = 1000
    random_seed: int = 42
    accel_bias_sigma_mps2: float = 0.05
    gyro_bias_sigma_rads: float = 0.005
    lever_sigma_m: float = 0.01
    accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lever_arm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_damping: float = 0.0
    alpha: float = 0.0
    skip_crc: bool = False


def _cli_path() -> Path:
    repo_root = Path(__file__).resolve().parent.parent
    exe_name = "gg_pipeline_cli.exe" if os.name == "nt" else "gg_pipeline_cli"
    path = repo_root / "build" / exe_name
    if not path.exists():
        raise FileNotFoundError(f"Pipeline CLI not found at {path}. Build the project first.")
    return path


def run(config: MonteCarloConfig) -> Path:
    """Ingest the log, run the C++ pipeline with Monte Carlo enabled, and return the output dir."""
    config.output_dir.mkdir(parents=True, exist_ok=True)

    parquet_path = config.output_dir / "dataset.parquet"
    ingest_cfg = IngestConfig(
        log_path=config.base_log,
        output_path=parquet_path,
        apply_crc_validation=not config.skip_crc,
    )
    ingest(ingest_cfg)

    frame = pd.read_parquet(parquet_path)
    csv_path = config.output_dir / "dataset.csv"
    frame.to_csv(csv_path, index=False)

    cli = _cli_path()
    cmd = [
        str(cli),
        "--input", str(csv_path),
        "--output-json", str(config.output_dir / "results.json"),
        "--envelope-csv", str(config.output_dir / "envelope.csv"),
        "--steady-csv", str(config.output_dir / "steady_points.csv"),
        "--alpha", str(config.alpha),
        "--monte-carlo", str(config.samples),
        "--mc-seed", str(config.random_seed),
        "--mc-accel-sigma", str(config.accel_bias_sigma_mps2),
        "--mc-gyro-sigma", str(config.gyro_bias_sigma_rads),
        "--mc-lever-sigma", str(config.lever_sigma_m),
    ]

    if any(abs(v) > 0 for v in config.accel_bias):
        cmd.extend(["--accel-bias", *map(str, config.accel_bias)])
    if any(abs(v) > 0 for v in config.gyro_bias):
        cmd.extend(["--gyro-bias", *map(str, config.gyro_bias)])
    if any(abs(v) > 0 for v in config.lever_arm):
        cmd.extend(["--lever-arm", *map(str, config.lever_arm)])
    if config.velocity_damping:
        cmd.extend(["--velocity-damping", str(config.velocity_damping)])

    subprocess.run(cmd, check=True)

    summary_path = config.output_dir / "results.json"
    if summary_path.exists():
        data = json.loads(summary_path.read_text())
        config.output_dir.joinpath("summary_mc.json").write_text(
            json.dumps(
                {
                    "samples": config.samples,
                    "seed": config.random_seed,
                    "max_gx": data.get("monte_carlo", {}).get("max_gx"),
                    "max_gy": data.get("monte_carlo", {}).get("max_gy"),
                    "envelope_area": data.get("monte_carlo", {}).get("envelope_area"),
                },
                indent=2,
            )
        )

    return config.output_dir
