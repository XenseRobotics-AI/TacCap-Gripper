#!/usr/bin/env python3
"""Compatibility entry point; use frequency_benchmark.py for new commands."""
import runpy
from pathlib import Path

if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).with_name("frequency_benchmark.py")), run_name="__main__")
