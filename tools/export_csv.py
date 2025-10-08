#!/usr/bin/env python3
"""Convert ESP32 binary logs (or Parquet output) into CSV for legacy tools."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent))

import pandas as pd

from pipeline.ingest import IngestConfig, ingest  # type: ignore  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Binary log (.bin) or Parquet file")
    parser.add_argument("--out", type=Path, required=True, help="Destination CSV path")
    parser.add_argument("--parquet", action="store_true", help="Treat input as already-ingested Parquet")
    parser.add_argument("--no-crc", action="store_true", help="Skip CRC validation when ingesting binary logs")
    args = parser.parse_args()

    if args.parquet:
        frame = pd.read_parquet(args.source)
    else:
        tmp_parquet = args.out.with_suffix(".tmp.parquet")
        config = IngestConfig(log_path=args.source, output_path=tmp_parquet, apply_crc_validation=not args.no_crc)
        ingest(config)
        frame = pd.read_parquet(tmp_parquet)
        tmp_parquet.unlink(missing_ok=True)

    frame.to_csv(args.out, index=False)
    print(f"Wrote CSV to {args.out}")


if __name__ == "__main__":
    main()
