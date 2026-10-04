"""CLI: generate a roadmap problem from a continuous-space problem.

Mirrors the reference tool `roadmap_generation`:

    psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt
    psipp-generate-roadmap config.yaml
"""
from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config",
        nargs="?",
        default="config/roadmap_generation.yaml",
        help="YAML configuration file (default: config/roadmap_generation.yaml)",
    )
    parser.parse_args()
    raise SystemExit("psipp-generate-roadmap: not implemented yet")


if __name__ == "__main__":
    main()
