"""Parsers and writers for problem file formats.

* ``roadmap.py`` -- the ROADMAP text format (vertices, edges, tasks, radius).
* ``spatial.py`` -- the SPATIAL polygon format, MAPF_Benchmark ``.map`` and
  ``.scen`` readers, and ``load_scenario`` which wires a ``.scen`` file
  into a ``SpatialProblemInstance``.

The round-trip printer for spatial problems is not implemented (see
``spatial.py`` module docstring -- the reference's ``print_text_file`` is
not the inverse of ``load_text_file``, and is driver-local to
``simplify_grid``).
"""

from psipp_ctc.io.roadmap import (
    load_roadmap_problem,
    print_roadmap_problem,
)
from psipp_ctc.io.spatial import (
    load_grip_map,
    load_mapfb_grid,
    load_mapfb_scenario,
    load_scenario,
    load_spatial_problem,
)

__all__ = [
    "load_grip_map",
    "load_mapfb_grid",
    "load_mapfb_scenario",
    "load_roadmap_problem",
    "load_scenario",
    "load_spatial_problem",
    "print_roadmap_problem",
]
