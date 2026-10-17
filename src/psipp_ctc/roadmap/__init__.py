"""Roadmap generation.

Three generators, taking a continuous-space spatial problem and
returning a roadmap (graph of vertices and edges) that the planner
solves on:

  - :func:`grid.generate_grid` -- uniform 2D grid, K-connectivity.
  - :func:`probabilistic_roadmap.generate_kprm` -- k-Nearest PRM.
  - :func:`probabilistic_roadmap.generate_prm` -- radius PRM.
  - :func:`constrained_triangulation.generate_cdt` -- CDT
    (constrained Delaunay triangulation; with ``scipy.spatial.Delaunay``
    + a shapely offset filter, see the module docstring for the
    CGAL-deviation discussion).

The geometry backend is :mod:`psipp_ctc.roadmap.space_2d` (shapely +
scipy.spatial.Delaunay). See its module docstring for the backend
decision and the numerical-divergence risk.

All generators take a :class:`SpatialProblemInstance` and return a
:class:`ProblemInstanceWithGeometry` whose ``edges`` is an adjacency
list (``edges[v] = [target, ...]`` -- the contract every planner
and the text I/O format depend on).
"""

from psipp_ctc.roadmap.constrained_triangulation import generate_cdt
from psipp_ctc.roadmap.grid import DEFAULT_K, generate_grid
from psipp_ctc.roadmap.probabilistic_roadmap import (
    SMALL_VALUE,
    generate_kprm,
    generate_prm,
)

__all__ = [
    "DEFAULT_K",
    "SMALL_VALUE",
    "generate_cdt",
    "generate_grid",
    "generate_kprm",
    "generate_prm",
]
