from __future__ import annotations
import sys
from pathlib import Path
from .device import run

def main() -> None:
    dir_path = Path(__file__).resolve().parent
    run(str(dir_path))

if __name__ == "__main__":
    main()