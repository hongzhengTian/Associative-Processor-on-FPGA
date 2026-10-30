#!/usr/bin/env python3
"""Convert a raw binary image to $readmemh format, one 32-bit little-endian
word per line."""
import sys


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit("usage: bin2hex.py <input.bin> <output.hex>")
    data = open(sys.argv[1], "rb").read()
    data += b"\0" * (-len(data) % 4)
    with open(sys.argv[2], "w") as out:
        for i in range(0, len(data), 4):
            out.write("%08x\n" % int.from_bytes(data[i:i + 4], "little"))


if __name__ == "__main__":
    main()
