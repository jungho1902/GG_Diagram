"""Binary log reader utilities shared across the processing pipeline."""

from __future__ import annotations

import dataclasses
import struct
import zlib
from typing import BinaryIO, Iterator

RECORD_SIZE = 32
_STRUCT = struct.Struct("<QhhhhhhhHHBBI")

@dataclasses.dataclass
class LogRecord:
    """In-memory representation of a single telemetry sample."""

    timestamp_us: int
    accel_mg: tuple[int, int, int]
    gyro_cds: tuple[int, int, int]
    throttle_pct: int
    steering_deg: int
    vbat_mv: int
    flags: int
    sequence: int
    crc32: int

    def accel_g(self) -> tuple[float, float, float]:
        return tuple(axis / 1000.0 for axis in self.accel_mg)

    def gyro_dps(self) -> tuple[float, float, float]:
        return tuple(axis / 100.0 for axis in self.gyro_cds)


def iter_records(stream: BinaryIO, *, validate_crc: bool = True) -> Iterator[LogRecord]:
    """Yield :class:`LogRecord` objects parsed from ``stream``."""

    while True:
        chunk = stream.read(RECORD_SIZE)
        if not chunk:
            break
        if len(chunk) != RECORD_SIZE:
            raise ValueError("truncated record encountered")

        unpacked = _STRUCT.unpack(chunk)
        (timestamp_us,
         ax, ay, az,
         gx, gy, gz,
         throttle_pct,
         steering_deg,
         vbat_mv,
         flags,
         sequence,
         crc32) = unpacked

        if validate_crc:
            expected = zlib.crc32(chunk[:-4]) & 0xFFFFFFFF
            if expected != crc32:
                raise ValueError(
                    f"crc mismatch at seq {sequence}: expected 0x{expected:08X}, "
                    f"found 0x{crc32:08X}"
                )

        yield LogRecord(
            timestamp_us=timestamp_us,
            accel_mg=(ax, ay, az),
            gyro_cds=(gx, gy, gz),
            throttle_pct=throttle_pct,
            steering_deg=steering_deg,
            vbat_mv=vbat_mv,
            flags=flags,
            sequence=sequence,
            crc32=crc32,
        )


def decode_file(path, *, validate_crc: bool = True):
    """Convenience wrapper returning a list of records from a file path."""
    with open(path, "rb") as stream:
        return list(iter_records(stream, validate_crc=validate_crc))
