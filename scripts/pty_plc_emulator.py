#!/usr/bin/env python3
import argparse
import json
import os
import select
import sys
import tty


ASCII_STX = 0x02
ASCII_ETX = 0x03


def extract_frames(buffer: bytearray):
    frames = []
    while True:
        try:
            stx = buffer.index(ASCII_STX)
        except ValueError:
            buffer.clear()
            break

        if stx > 0:
            del buffer[:stx]

        try:
            etx = buffer.index(ASCII_ETX, 1)
        except ValueError:
            break

        payload = bytes(buffer[1:etx])
        del buffer[:etx + 1]
        frames.append(payload)
    return frames


def parse_tx_payload(payload: bytes):
    text = payload.decode("ascii", errors="replace")
    if not text:
        return {"raw": text}

    # Expected format built by the driver:
    # <syst>tx <cmd> <val> <len><crc>
    parts = text.split()
    if len(parts) < 4 or len(parts[0]) < 3:
        return {"raw": text}

    token = parts[0]
    if token[1:] != "tx":
        return {"raw": text}

    try:
        cmd = int(parts[1])
        val = int(parts[2])
    except ValueError:
        return {"raw": text}

    return {
        "raw": text,
        "syst": token[0],
        "verb": "tx",
        "cmd": cmd,
        "val": val,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal PTY peer for indi-gapers virtual serial tests")
    parser.add_argument("device", help="PTY device path, e.g. /dev/pts/27")
    parser.add_argument("--echo", action="store_true", help="Echo back any received bytes")
    parser.add_argument(
        "--capture-jsonl",
        help="Optional path to a JSONL file where each received PLC frame is appended",
    )
    args = parser.parse_args()

    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY)
    # PTY defaults can be canonical; force raw mode to read binary STX/ETX frames immediately.
    tty.setraw(fd)

    capture_fp = None
    if args.capture_jsonl:
        capture_fp = open(args.capture_jsonl, "a", encoding="utf-8")

    frame_buffer = bytearray()
    try:
        while True:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if fd not in ready:
                continue
            data = os.read(fd, 1024)
            if not data:
                continue

            frame_buffer.extend(data)
            for frame in extract_frames(frame_buffer):
                if capture_fp is not None:
                    json.dump(parse_tx_payload(frame), capture_fp)
                    capture_fp.write("\n")
                    capture_fp.flush()

            if args.echo:
                os.write(fd, data)
    except KeyboardInterrupt:
        return 0
    finally:
        if capture_fp is not None:
            capture_fp.close()
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
