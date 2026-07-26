#!/usr/bin/env python3
"""Field USB storage client for RocketV10 and GroundStationV10."""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import termios
import time
import tty
import zlib
from pathlib import Path


FRAME = struct.Struct("<4sBBHIIHHI")
FRAME_MAGIC = b"RVXF"


class SerialPort:
    def __init__(self, path: str, timeout: float = 8.0):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.timeout = timeout
        self.buffer = bytearray()
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def __enter__(self) -> "SerialPort":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def drain_input(self, seconds: float = 0.25) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], 0.02)
            if not ready:
                continue
            try:
                os.read(self.fd, 65536)
            except BlockingIOError:
                pass
        self.buffer.clear()

    def write_line(self, line: str) -> None:
        data = (line.rstrip("\r\n") + "\n").encode("ascii")
        position = 0
        while position < len(data):
            _, ready, _ = select.select([], [self.fd], [], self.timeout)
            if not ready:
                raise TimeoutError("serial write timed out")
            position += os.write(self.fd, data[position:])

    def _fill(self, needed: int, deadline: float) -> None:
        while len(self.buffer) < needed:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("serial read timed out")
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                raise TimeoutError("serial read timed out")
            chunk = os.read(self.fd, 65536)
            if not chunk:
                raise EOFError("serial device disconnected")
            self.buffer.extend(chunk)

    def read_exact(self, length: int, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        self._fill(length, deadline)
        result = bytes(self.buffer[:length])
        del self.buffer[:length]
        return result

    def read_line(self, timeout: float | None = None) -> str:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self.buffer[:newline])
                del self.buffer[: newline + 1]
                return raw.rstrip(b"\r").decode("utf-8", "replace")
            self._fill(len(self.buffer) + 1, deadline)


def command_for(backend: str, operation: str, target: str | None = None) -> str:
    backend = backend.upper()
    if operation == "list":
        return f"{backend} LIST"
    if operation == "info":
        return f"{backend} INFO {target}"
    raise ValueError(operation)


def print_response(port: SerialPort, command: str, end_marker: str | None) -> int:
    port.drain_input()
    port.write_line(command)
    while True:
        line = port.read_line()
        print(line)
        if line.startswith("ERR "):
            return 1
        if end_marker and line.startswith(end_marker):
            return 0
        if not end_marker:
            return 0


def parse_begin(line: str) -> dict[str, str]:
    parts = line.split()
    if len(parts) < 4 or parts[:2] != ["XFER", "BEGIN"]:
        raise RuntimeError(f"unexpected transfer response: {line}")
    return dict(zip(parts[2::2], parts[3::2]))


def download(
    port: SerialPort,
    backend: str,
    target: str,
    output: Path,
    resume: bool,
    requested_length: int,
) -> None:
    backend = backend.upper()
    if output.exists() and not resume:
        raise FileExistsError(f"{output} exists; use --resume or remove it")
    offset = output.stat().st_size if output.exists() else 0
    command = f"{backend} READ {target} {offset} {requested_length}"
    port.drain_input()
    port.write_line(command)

    while True:
        line = port.read_line(timeout=15.0)
        if line.startswith("ERR "):
            raise RuntimeError(line)
        if line.startswith("XFER BEGIN "):
            begin = parse_begin(line)
            break
        if line:
            print(line)

    transfer_length = int(begin["LENGTH"])
    file_size = int(begin["SIZE"])
    expected_offset = int(begin["OFFSET"])
    if expected_offset != offset:
        raise RuntimeError(f"device offset {expected_offset} != local offset {offset}")

    mode = "ab" if offset else "wb"
    received = 0
    expected_sequence = 0
    running_crc = 0
    started = time.monotonic()
    with output.open(mode) as destination:
        while received < transfer_length:
            header = port.read_exact(FRAME.size, timeout=15.0)
            magic, version, frame_backend, flags, sequence, frame_offset, length, _, crc = (
                FRAME.unpack(header)
            )
            if magic != FRAME_MAGIC or version != 1:
                raise RuntimeError("invalid transfer frame header")
            if frame_backend != (1 if backend == "NAND" else 2):
                raise RuntimeError("wrong transfer backend")
            if sequence != expected_sequence or frame_offset != offset + received:
                raise RuntimeError("transfer sequence/offset discontinuity")
            payload = port.read_exact(length, timeout=15.0)
            if zlib.crc32(payload) & 0xFFFFFFFF != crc:
                raise RuntimeError(f"CRC mismatch at offset {frame_offset}")
            destination.write(payload)
            running_crc = zlib.crc32(payload, running_crc)
            received += length
            expected_sequence += 1
            if flags & 1 and received != transfer_length:
                raise RuntimeError("premature final frame")
            elapsed = max(time.monotonic() - started, 0.001)
            percent = 100.0 if transfer_length == 0 else received * 100.0 / transfer_length
            print(
                f"\r{received}/{transfer_length} bytes ({percent:5.1f}%) "
                f"{received / elapsed / 1024:.0f} KiB/s",
                end="",
                flush=True,
            )
        destination.flush()
        os.fsync(destination.fileno())
    print()

    end_line = port.read_line(timeout=10.0)
    while not end_line.startswith("XFER END "):
        if end_line.startswith("ERR ") or end_line.startswith("XFER ERROR"):
            raise RuntimeError(end_line)
        end_line = port.read_line(timeout=10.0)
    fields = dict(zip(end_line.split()[2::2], end_line.split()[3::2]))
    device_crc = int(fields["CRC32"], 16)
    if int(fields["BYTES"]) != received or device_crc != (running_crc & 0xFFFFFFFF):
        raise RuntimeError("final byte count or CRC mismatch")
    print(
        f"Saved {output} ({output.stat().st_size}/{file_size} bytes), "
        f"segment CRC32 {running_crc & 0xFFFFFFFF:08X}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="/dev/cu.usbmodem...")
    sub = parser.add_subparsers(dest="operation", required=True)
    sub.add_parser("status")
    list_parser = sub.add_parser("list")
    list_parser.add_argument("backend", choices=("nand", "sd"))
    info_parser = sub.add_parser("info")
    info_parser.add_argument("backend", choices=("nand", "sd"))
    info_parser.add_argument("target", help="NAND index or SD filename")
    get_parser = sub.add_parser("download")
    get_parser.add_argument("backend", choices=("nand", "sd"))
    get_parser.add_argument("target", help="NAND index or SD filename")
    get_parser.add_argument("output", type=Path)
    get_parser.add_argument("--resume", action="store_true")
    get_parser.add_argument("--length", type=int, default=0, help="0 means to EOF")
    args = parser.parse_args()

    try:
        with SerialPort(args.port) as port:
            if args.operation == "status":
                return print_response(port, "STORAGE STATUS", "STORAGE STATUS END")
            if args.operation == "list":
                marker = f"{args.backend.upper()} LIST END"
                return print_response(
                    port, command_for(args.backend, "list"), marker
                )
            if args.operation == "info":
                return print_response(
                    port, command_for(args.backend, "info", args.target), None
                )
            download(
                port,
                args.backend,
                args.target,
                args.output,
                args.resume,
                args.length,
            )
            return 0
    except (OSError, EOFError, TimeoutError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
