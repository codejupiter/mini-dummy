#!/usr/bin/env python3
"""Send Cartesian G0 targets to the mini-dummy RP2040 firmware over USB-CDC."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

PICO_USB_VENDOR_ID = 0x2E8A


@dataclass(frozen=True)
class Target:
    x_mm: float
    y_mm: float
    z_mm: float

    def command(self) -> str:
        return f"G0 {self.x_mm:.3f} {self.y_mm:.3f} {self.z_mm:.3f}"


def available_ports() -> list[list_ports.ListPortInfo]:
    return list(list_ports.comports())


def print_ports() -> None:
    ports = available_ports()
    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        vid_pid = (
            f"{port.vid:04X}:{port.pid:04X}"
            if port.vid is not None and port.pid is not None
            else "unknown VID:PID"
        )
        print(f"{port.device:24} {vid_pid:9} {port.description}")


def detect_pico_port() -> str:
    candidates = [port for port in available_ports() if port.vid == PICO_USB_VENDOR_ID]
    if not candidates:
        raise RuntimeError(
            "No Raspberry Pi Pico USB serial device found. "
            "Use --list, then pass --port explicitly."
        )
    if len(candidates) > 1:
        devices = ", ".join(port.device for port in candidates)
        raise RuntimeError(
            f"Multiple Pico devices found ({devices}). Select one with --port."
        )
    return candidates[0].device


class PicoController:
    def __init__(self, port: str, baud: int, timeout: float) -> None:
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.1,
            write_timeout=timeout,
        )
        self._response_timeout = timeout
        time.sleep(0.35)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "PicoController":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def request(self, command: str) -> str:
        payload = f"{command.strip()}\n".encode("ascii")
        self._serial.write(payload)
        self._serial.flush()

        deadline = time.monotonic() + self._response_timeout
        while time.monotonic() < deadline:
            response = self._serial.readline().decode("utf-8", errors="replace").strip()
            if response:
                return response
        raise TimeoutError(f"No response to {command!r} within {self._response_timeout}s")

    def move_to(self, target: Target) -> str:
        return self.request(target.command())


def parse_target(tokens: list[str]) -> Target:
    if tokens and tokens[0].upper() == "G0":
        tokens = tokens[1:]
    if len(tokens) != 3:
        raise ValueError("Expected three coordinates: X Y Z")
    return Target(*(float(value) for value in tokens))


def run_repl(controller: PicoController) -> None:
    print("mini-dummy controller — enter X Y Z (millimetres), 'status', or 'quit'")
    while True:
        try:
            raw = input("mini-dummy> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if not raw:
            continue
        if raw.lower() in {"quit", "exit"}:
            return
        if raw.lower() == "status":
            print(controller.request("M114"))
            continue
        if raw.lower() == "ping":
            print(controller.request("PING"))
            continue

        try:
            target = parse_target(raw.split())
            print(controller.move_to(target))
        except (ValueError, TimeoutError, serial.SerialException) as error:
            print(f"error: {error}", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send Cartesian coordinates to mini-dummy over USB-CDC."
    )
    parser.add_argument("coordinates", nargs="*", metavar="XYZ")
    parser.add_argument("--port", help="Serial device; auto-detected when omitted")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="CDC compatibility baud rate (default: 115200)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Response timeout in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List serial devices and exit",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.list:
        print_ports()
        return 0

    if len(args.coordinates) not in {0, 3}:
        print("error: provide either zero coordinates or exactly X Y Z", file=sys.stderr)
        return 2

    try:
        port = args.port or detect_pico_port()
        print(f"Connecting to {port}...", file=sys.stderr)
        with PicoController(port, args.baud, args.timeout) as controller:
            if args.coordinates:
                response = controller.move_to(parse_target(args.coordinates))
                print(response)
                return 0 if response.startswith("OK") else 2
            run_repl(controller)
            return 0
    except (RuntimeError, ValueError, TimeoutError, serial.SerialException) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
