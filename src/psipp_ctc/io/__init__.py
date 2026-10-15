"""Parsers and writers for problem/plan file formats.

Stage 1a: the ROADMAP text format (`base/input.hpp`, lines 12-79) is in
`roadmap.py`. Stage 1b: the SPATIAL polygon format plus the MAPF_Benchmark
.map and .scen readers are in `spatial.py`. The round-trip printer for
spatial problems is NOT ported (see `spatial.py` module docstring -- the
reference's `printTextFile` is not the inverse of `loadTextFile`, and is
driver-local to `simplify_grid`).
"""

from psipp_ctc.io.roadmap import (
    load_roadmap_problem,
    print_roadmap_problem,
)
from psipp_ctc.io.spatial import (
    load_grip_map,
    load_mapfb_grid,
    load_mapfb_scenario,
    load_spatial_problem,
)

__all__ = [
    "load_roadmap_problem",
    "print_roadmap_problem",
    "load_spatial_problem",
    "load_mapfb_grid",
    "load_grip_map",
    "load_mapfb_scenario",
]
