"""CLI: generate a roadmap problem from a continuous-space problem.

Two invocation forms:

    psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt
    psipp-generate-roadmap config.yaml

The first form passes the spatial problem on stdin and uses the
inline default config (which mirrors the C++'s
``config/roadmap_generation.yaml``: kPRM(100, 10), stdin input, stdout
output). The second form loads an arbitrary YAML config.

YAML schema (the same fields the C++'s ``roadmap_generation.cpp``
consumes):

    Generator:
      type: kPRM            # PRM | PRMStar | kPRM | kPRMStar | GRID | CDT
      # type-specific params, e.g. number_of_vertices, radius,
      # number_of_neighbors, grid_size, K, number_of_additionals,
      # use_corner_points
    Input:
      type: Text            # Text | MAPF_Benchmark
      file: <path or "stdin">
      # MAPF_Benchmark-only:
      map: <path>
      agent_size: <float>
      scenario: <path>      # optional
    Output: <path or "stdout">
    Iterations: <int>       # default 1

The driver runs the generator ``iterations`` times; each call
writes its output to ``output_file + str(count)`` (or just
``output_file`` when ``iterations == 1``). After each call it
invokes ``check_roadspace_validity`` on the generated roadmap;
failures raise ``ValueError`` and abort the run.
"""
from __future__ import annotations

import argparse
import sys
from typing import Any, Dict

import yaml

from psipp_ctc.config import (
    make_generator,
    make_spatial_problem_instance,
)
from psipp_ctc.io import print_roadmap_problem


#: Inline default config. stdin is the spatial problem source,
#: kPRM(100, 10) is the generator, ``stdout`` is the output
#: destination.
_DEFAULT_CONFIG: Dict[str, Any] = {
    "Generator": {
        "type": "kPRM",
        "number_of_vertices": 100,
        "number_of_neighbors": 10,
    },
    "Input": {
        "type": "Text",
        "file": "stdin",
    },
    "Output": "stdout",
}


def _load_config(path: str) -> Dict[str, Any]:
    """Read a YAML config from ``path``.

    The C++ uses ``YAML::LoadFile``; the Python port uses
    ``yaml.safe_load``. Equivalent for the schema above.
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
            "YAML configuration file. Default is kPRM(100, 10) "
            "generator, stdin spatial problem, stdout output."
        ),
    )
    args = parser.parse_args()

    config = _load_config(args.config)
    generator = make_generator(config["Generator"])
    spatial = make_spatial_problem_instance(config["Input"])
    iterations = int(config.get("Iterations", 1) or 1)
    output_file = config["Output"]

    for count in range(iterations):
        generator.generate(spatial)
        suffix = "" if iterations == 1 else str(count)
        print_roadmap_problem(generator.problem_instance, output_file + suffix)
        # ``check_roadspace_validity`` is the post-generation
        # validation. The Python version raises ``ValueError`` on
        # failure; we let it propagate so the CLI exits with a
        # non-zero status.
        spatial.check_roadspace_validity(generator.problem_instance)


if __name__ == "__main__":
    main()
