"""CLI: solve roadmap problems and report planner statistics.

Mirrors the reference tool `planner_benchmark`:

    psipp-plan < roadmap_problem.txt > plan.txt
    psipp-plan config.yaml
"""
from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config",
        nargs="?",
        default="config/planner_benchmark.yaml",
        help="YAML configuration file (default: config/planner_benchmark.yaml)",
    )
    parser.parse_args()
    raise SystemExit("psipp-plan: not implemented yet")


if __name__ == "__main__":
    main()
