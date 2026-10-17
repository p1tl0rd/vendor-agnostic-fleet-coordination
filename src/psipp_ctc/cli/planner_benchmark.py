"""CLI: solve roadmap problems and report planner statistics.

Two invocation forms:

    psipp-plan < roadmap_problem.txt > plan.txt
    psipp-plan config.yaml

The first form passes the problem on stdin and uses the inline
default config (which mirrors the C++'s
``config/planner_benchmark.yaml``: stdin source, PSIPP planner, stdout
output). The second form loads an arbitrary YAML config.

YAML schema (the same fields the C++'s ``planner_benchmark.cpp``
consumes):

    Problems:
      <name>:
        type: Explicit            # Geometric | Explicit | Turning | Grid
        file: <path or "stdin">
        # optional:
        number_of_agents: <int>   # truncates ``tasks`` after load
    Planners:
      <name>:
        type: PSIPP               # CCBS not ported
        # optional:
        time_limit: 30.0          # seconds
    Output: <path or "stdout">    # optional; plans go here on success
    Iterations: <int>             # default 1 (matches the C++)

The output is the ``Benchmark.display_all_results()`` table on
stderr. The driver also writes ``"   problem   time(ms)\\n"``
followed by one line per problem to stderr to mirror the C++'s
problem-load timing report.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict

import yaml

from psipp_ctc.benchmark import Benchmark
from psipp_ctc.config import make_planner, make_problem_instance


#: Inline default config. stdin is the problem source, ``PSIPP``
#: is the planner, and ``stdout`` is the output destination.
_DEFAULT_CONFIG: Dict[str, Any] = {
    "Problems": {
        "Stdin": {
            "type": "Explicit",
            "file": "stdin",
        },
    },
    "Planners": {
        "PSIPP": {
            "type": "PSIPP",
        },
    },
    "Output": "stdout",
}


def _load_config(path: str) -> Dict[str, Any]:
    """Read a YAML config from ``path``.

    Uses ``yaml.safe_load`` (already a project dependency). For the
    schema above every value is a primitive or a ``dict``/``list``,
    which is the read shape the consumer expects.
    """
    if path == "default":
        return _DEFAULT_CONFIG
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "config",
        nargs="?",
        default="default",
        help=(
            "YAML configuration file. Default is stdin problem, "
            "PSIPP planner, stdout output."
        ),
    )
    args = parser.parse_args()

    config = _load_config(args.config)

    benchmark = Benchmark()
    # ``iterations`` defaults to 1, matching the C++; the YAML key
    # is allowed as an extension for parity with the
    # incrementally-experiments driver.
    benchmark.iterations = int(config.get("Iterations", 1) or 1)
    if "time_limit" in config and config.get("time_limit") is not None:
        benchmark.time_limit = float(config["time_limit"])

    # Problem-load timing report.
    sys.stderr.write("   problem   time(ms)\n")
    for name, prob_config in (config.get("Problems") or {}).items():
        t0 = time.monotonic()
        problem = make_problem_instance(prob_config)
        elapsed_ms = int((time.monotonic() - t0) * 1000)
        sys.stderr.write(f"{name:>10s} {elapsed_ms:10d}\n")
        benchmark.add_problem_instance(problem, name)

    for name, plan_config in (config.get("Planners") or {}).items():
        benchmark.add_planner(make_planner(plan_config), name)

    output_file = config.get("Output", "") or ""
    if output_file:
        benchmark.run_all(output_file)
    else:
        benchmark.run_all()
    benchmark.display_all_results()


if __name__ == "__main__":
    main()
