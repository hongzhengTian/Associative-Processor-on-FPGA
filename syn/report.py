#!/usr/bin/env python3
"""Summarize Yosys `stat` output of several synthesis logs as a Markdown table."""
import re
import sys

# cell name prefix -> column
GROUPS = [
    ("LUT", re.compile(r"^LUT[1-6]$")),
    ("LUTRAM", re.compile(r"^RAM(32|64|128|256|512)")),
    ("FF", re.compile(r"^FD[CPRSE]+$")),
    ("CARRY", re.compile(r"^CARRY[48]$")),
    ("MUXF", re.compile(r"^MUXF[789]$")),
    ("BRAM", re.compile(r"^RAMB(18|36)")),
    ("URAM", re.compile(r"^URAM")),
    ("DSP", re.compile(r"^DSP")),
]


def last_stat(text: str) -> dict:
    """Cell counts of the last statistics block in a log."""
    block = text.split("Printing statistics.")[-1]
    cells = {}
    for name, count in re.findall(r"^\s+(\w+)\s+(\d+)\s*$", block, re.M):
        cells[name] = cells.get(name, 0) + int(count)
    return cells


def main() -> None:
    print("| target | " + " | ".join(g for g, _ in GROUPS) + " |")
    print("|---|" + "---:|" * len(GROUPS))
    for path in sys.argv[1:]:
        cells = last_stat(open(path).read())
        sums = []
        for _, pat in GROUPS:
            sums.append(sum(n for c, n in cells.items() if pat.match(c)))
        name = path.rsplit("/", 1)[-1].removesuffix(".log")
        print(f"| {name} | " + " | ".join(str(s) for s in sums) + " |")


if __name__ == "__main__":
    main()
