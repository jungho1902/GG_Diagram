"""Ingestion pipeline converting raw ESP32 logs into structured datasets."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Iterable

import pandas as pd

from .log_reader import iter_records


@dataclasses.dataclass
class IngestConfig:
    """Configuration for a single ingestion run."""

    log_path: Path
    output_path: Path
    metadata_path: Path | None = None
    apply_crc_validation: bool = True


def to_dataframe(records) -> pd.DataFrame:
    """Convert an iterable of :class:`LogRecord` objects into a DataFrame."""
    rows = []
    for record in records:
        rows.append(
            {
                "timestamp_us": record.timestamp_us,
                "ax_mps2": record.accel_mg[0] * 9.80665 / 1000.0,
                "ay_mps2": record.accel_mg[1] * 9.80665 / 1000.0,
                "az_mps2": record.accel_mg[2] * 9.80665 / 1000.0,
                "gx_rads": record.gyro_cds[0] * (3.141592653589793 / 18000.0),
                "gy_rads": record.gyro_cds[1] * (3.141592653589793 / 18000.0),
                "gz_rads": record.gyro_cds[2] * (3.141592653589793 / 18000.0),
                "throttle_pct": record.throttle_pct,
                "steering_deg": record.steering_deg,
                "vbat_mv": record.vbat_mv,
                "flags": record.flags,
                "sequence": record.sequence,
            }
        )
    return pd.DataFrame(rows)


def ingest(config: IngestConfig) -> Path:
    """Run the ingestion process and return the output path."""
    with config.log_path.open("rb") as stream:
        frame = to_dataframe(iter_records(stream, validate_crc=config.apply_crc_validation))

    frame.to_parquet(config.output_path, index=False)
    return config.output_path


def batch_ingest(configs: Iterable[IngestConfig]) -> list[Path]:
    """Ingest multiple logs in sequence."""
    outputs = []
    for config in configs:
        outputs.append(ingest(config))
    return outputs
