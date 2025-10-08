#!/usr/bin/env python3
"""BLE control utility for the ESP32 RC car."""

from __future__ import annotations

import argparse
import asyncio
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    from bleak import BleakClient, BleakScanner
except ImportError as exc:  # pragma: no cover - import guard
    print("Bleak is required: pip install bleak", file=sys.stderr)
    raise SystemExit(1) from exc

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
CMD_UUID = "12345678-1234-5678-1234-56789abcdef1"
STATS_UUID = "12345678-1234-5678-1234-56789abcdef3"


async def scan(timeout: float) -> None:
    devices = await BleakScanner.discover(timeout=timeout)
    if not devices:
        print("No BLE devices found.")
        return
    for dev in devices:
        print(f"{dev.address}  {dev.name or '<unknown>'}  RSSI={dev.rssi}")


def build_command(throttle: int | None, steering: int | None, label: str | None, start: bool, stop: bool, raw: Iterable[str]) -> str:
    tokens = []
    if throttle is not None:
        tokens.append(f"T:{throttle}")
    if steering is not None:
        tokens.append(f"S:{steering}")
    if label is not None:
        tokens.append(f"RUN:{label}")
    if start:
        tokens.append("LOG:START")
    if stop:
        tokens.append("LOG:STOP")
    tokens.extend(raw)
    return " ".join(tokens)


async def send_command(address: str, command: str, notify_stats: bool) -> None:
    if not command:
        print("No command to send.")
        return
    print(f"Connecting to {address}...")
    async with BleakClient(address) as client:
        print("Connected. Writing command:", command)
        await client.write_gatt_char(CMD_UUID, command.encode("utf-8"), response=True)
        if notify_stats:
            try:
                data = await client.read_gatt_char(STATS_UUID)
                print("Stats:", data.decode("utf-8"))
            except Exception as exc:  # pragma: no cover - optional
                print("Failed to read stats:", exc)


async def main_async(args: argparse.Namespace) -> None:
    if args.scan:
        await scan(args.timeout)
        return
    if not args.device:
        raise SystemExit("--device or --scan required")
    command = build_command(
        throttle=args.throttle,
        steering=args.steering,
        label=args.label,
        start=args.start_log,
        stop=args.stop_log,
        raw=args.raw or (),
    )
    await send_command(args.device, command, args.stats)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan", action="store_true", help="Scan for nearby BLE devices and exit")
    parser.add_argument("--device", type=str, help="MAC address of the ESP32 device")
    parser.add_argument("--timeout", type=float, default=5.0, help="Scan timeout or connection timeout")
    parser.add_argument("--throttle", type=int, help="Send throttle command (-100..100)")
    parser.add_argument("--steering", type=int, help="Send steering command (0..180)")
    parser.add_argument("--label", type=str, help="Set run label before logging")
    parser.add_argument("--start-log", action="store_true", help="Send LOG:START")
    parser.add_argument("--stop-log", action="store_true", help="Send LOG:STOP")
    parser.add_argument("--stats", action="store_true", help="Read stats characteristic after command")
    parser.add_argument("raw", nargs="*", help="Additional raw tokens to send")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:  # pragma: no cover - manual stop
        print("Interrupted.")


if __name__ == "__main__":
    main()
