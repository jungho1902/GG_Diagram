#!/usr/bin/env python3
"""Decode ESP32 RC car binary logs into structured Python objects."""

from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

# Ensure the sibling pipeline package is importable when running from the repo root
sys.path.append(str(Path(__file__).parent))

from pipeline.log_reader import iter_records  # type: ignore


def summarize(path: Path) -> None:
    """Print a quick summary for a binary log file."""
    with path.open("rb") as f:
        records = list(iter_records(f))

    if not records:
        print("[empty log]")
        return

    start = records[0].timestamp_us
    end = records[-1].timestamp_us
    duration = (end - start) / 1_000_000.0
    print(f"records      : {len(records)}")
    print(f"duration     : {duration:.3f} s")
    print(f"first seq    : {records[0].sequence}")
    print(f"last seq     : {records[-1].sequence}")
    print(f"first flags  : 0x{records[0].flags:02X}")
    print(f"last flags   : 0x{records[-1].flags:02X}")
    print(f"battery range: {min(r.vbat_mv for r in records)}-{max(r.vbat_mv for r in records)} mV")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path, help="binary log file to decode")
    parser.add_argument("--no-crc", action="store_true", help="skip CRC validation")
    parser.add_argument("--summary", action="store_true", help="only display aggregate statistics")
    parser.add_argument("--limit", type=int, default=0, help="print at most N records")
    args = parser.parse_args()

    if args.summary:
        summarize(args.path)
        return

    remaining = args.limit if args.limit else None
    with args.path.open("rb") as f:
        for record in iter_records(f, validate_crc=not args.no_crc):
            print(dataclasses.asdict(record))
            if remaining is not None:
                remaining -= 1
                if remaining <= 0:
                    break


if __name__ == "__main__":
    main()
