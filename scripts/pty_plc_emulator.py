#!/usr/bin/env python3
import argparse
import os
import select
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal PTY peer for indi-gapers virtual serial tests")
    parser.add_argument("device", help="PTY device path, e.g. /dev/pts/27")
    parser.add_argument("--echo", action="store_true", help="Echo back any received bytes")
    args = parser.parse_args()

    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY)
    try:
        while True:
            ready, _, _ = select.select([fd], [], [], 0.5)
            if fd not in ready:
                continue
            data = os.read(fd, 1024)
            if not data:
                continue
            if args.echo:
                os.write(fd, data)
    except KeyboardInterrupt:
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
