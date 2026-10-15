"""Stage 3 -- the geometry backend for roadmap generation.

Port of
  PSIPP-CTC_src/src/base/geometry_2D_for_generator.hpp  (Space2D, 194 lines)
  PSIPP-CTC_src/src/base/spatial.hpp                  (SpatialProblemInstance, 168 lines)

This module wraps the Python geometry backend and exposes the same six
operations the C++ reference uses during roadmap generation:

  - check_vertex_inside(space, point, agent_geometry) -- point-in-polygon
    with a clearance margin
  - check_edge_inside(space, p0, p1, agent_geometry) -- a disc travelling
    along the segment (p0, p1) stays inside the space
  - get_bound(space, axis) -- bounding-box of the outer ring
  - get_random_points(space, agent_geometry, n) -- uniform rejection sampling
  - get_separate_random_points(space, agent_geometry, n) -- as above, plus
    pairwise 2*radius separation
  - calculate_free_volume(space, agent_geometry) -- Minkowski-shrunk area

Backend decision: shapely + scipy.spatial.Delaunay
--------------------------------------------------
The C++ uses `boost::geometry` (for polygon ops + offset) and
`CGAL::Constrained_Delaunay_triangulation_2` (for CDT). Both are banned by
AGENTS.md. The chosen Python substitutes are:

  shapely -- the de-facto Python 2D geometry library (MIT, GEOS-backed).
  Covers all the polygon operations Space2D needs: containment, distance
  to boundary, area, length, inward offset, line/polygon intersection.

  scipy.spatial.Delaunay -- unconstrained Delaunay. The CDT generator
  builds an unconstrained triangulation and then filters edges through
  the whole-segment ``isInside`` test (the "clearance filter" in the
  brief). The filter is what makes every surviving edge collision-free
  for a disc; the offset/constraint behaviour of CGAL's CDT is not
  load-bearing for the output. See ``constrained_triangulation.py`` for
  the detail.

Numerical divergence risk (recorded here per the brief)
-------------------------------------------------------
- shapely's GEOS uses a different floating-point model than
  ``boost::geometry``: inward offsets of the same polygon can produce
  vertex sequences that differ at sub-pixel scale. For CDT this means
  the input offset border is approximately the same shape but not
  bit-identical. The clearance filter dominates the output, so the
  *set of surviving edges* is structurally similar but the
  triangulation topology can differ.
- kPRM is unseeded in the C++ (`std::random_device` +
  `std::default_random_engine`), so byte-parity is impossible regardless
  of backend; only structural parity (vertex count, every task start
  connected to its goal) is meaningful.
- The Grid generator is purely index-based; its output is the most
  reproducible of the three.
- These divergences are documented in the relevant tests; the gate is
  "every task's start and goal are connected in the output graph",
  not byte equality. See ``tests/test_roadmap_*.py``.

Three EPS values, three classes (PORTING.md §4 trap 5)
-----------------------------------------------------
  Geometry2D::EPS         = 0       (this module, collision tolerance)
  ProblemInstance::EPS    = 1e-8    (problem.py, data-model tolerance)
  SpatialProblemInstance::EPS = 1e-6  (problem.py, clearance margin used
                                    by isInside as ``agent_geometry - EPS``)

Do not unify them. The spatial EPS is a clearance margin, not a
comparison tolerance.
"""

from __future__ import annotations

import math
import random
from typing import List, Tuple

from shapely.geometry import LineString, Point as ShapelyPoint, Polygon

from psipp_ctc.problem import Point

# C++ Geometry2D::EPS = 0 and INF = inf
GEOMETRY_EPS: float = 0.0
INF: float = math.inf


# ---------------------------------------------------------------------------
# Polygon adapter -- the Space2D "Space" is a shapely Polygon (outer + holes)
# ---------------------------------------------------------------------------


def _to_shapely_point(p) -> ShapelyPoint:
    """Coerce a ``psipp_ctc.problem.Point`` (or a 2-tuple) to shapely Point."""
    if isinstance(p, Point):
        return ShapelyPoint(p.x, p.y)
    return ShapelyPoint(p[0], p[1])


def _make_polygon(outer: List[Point], inners: List[List[Point]]) -> Polygon:
    """Build a shapely Polygon from the C++ outer / inner ring representation.

    Both ``outer`` and each ``inners[i]`` are closed rings (first point
    repeated at the end), as produced by ``load_grip_map``. shapely's
    Polygon handles closed rings, but the duplicate first/last point is
    not required -- the constructor closes the ring automatically.
    """
    outer_coords = [(p.x, p.y) for p in outer]
    # Drop the closing duplicate; shapely closes automatically.
    if len(outer_coords) >= 2 and outer_coords[0] == outer_coords[-1]:
        outer_coords = outer_coords[:-1]
    inner_coords = []
    for ring in inners:
        rc = [(p.x, p.y) for p in ring]
        if len(rc) >= 2 and rc[0] == rc[-1]:
            rc = rc[:-1]
        inner_coords.append(rc)
    return Polygon(outer_coords, inner_coords)


# ---------------------------------------------------------------------------
# Space2D -- the six operations (mirror geometry_2D_for_generator.hpp:51-191)
# ---------------------------------------------------------------------------


def check_vertex_inside(space: Polygon, point, agent_geometry: float) -> bool:
    """Mirror of Space2D::checkVertexInside (geometry_2D_for_generator.hpp:51-68).

    Returns True iff ``point`` lies strictly inside ``space`` AND a disc of
    radius ``agent_geometry`` centred at ``point`` does not cross any
    boundary segment. The clearance is enforced by testing the distance
    from the point to the polygon's boundary.

    Notes on boundary behaviour: shapely's ``Polygon.contains`` is strict
    (boundary returns False), matching the C++ ``boost::geometry::within``
    semantics. The clearance check is then ``boundary.distance(point) >=
    agent_geometry``. shapely's ``Polygon.distance`` for an interior point
    returns 0 (the polygon contains the point), so we use ``boundary`` --
    a MultiLineString of every ring segment -- which gives the true
    distance to the closest edge.
    """
    pt = _to_shapely_point(point)
    if not space.contains(pt):
        return False
    # boundary is a MultiLineString; .distance gives the min distance to
    # any of its edges.
    return space.boundary.distance(pt) >= agent_geometry


def check_edge_inside(space: Polygon, p0, p1, agent_geometry: float) -> bool:
    """Mirror of Space2D::checkEdgeInside (geometry_2D_for_generator.hpp:70-98).

    Returns True iff a disc of radius ``agent_geometry`` can travel along
    the segment from ``p0`` to ``p1`` without crossing any boundary.

    The C++ does this in two phases:

    1. For each inner ring, project the FIRST vertex of the ring onto the
       segment and reject if the projection lies strictly inside the
       segment AND is within ``agent_geometry`` laterally. (This is a
       simplified clearance test against holes; only the first vertex of
       each hole is checked, not the whole ring. Ported verbatim.)
    2. Build a rectangle of width ``2 * agent_geometry`` around the
       segment, and reject if any boundary segment intersects the
       rectangle. (Catches outer-boundary proximity and inner-ring
       clearance via geometry rather than per-vertex projection.)
    """
    vec_x = p1.x - p0.x
    vec_y = p1.y - p0.y
    length = math.hypot(vec_x, vec_y)
    if length == 0.0:
        # Degenerate segment; fall back to vertex check.
        return check_vertex_inside(space, p0, agent_geometry)
    u0_x = vec_x / length
    u0_y = vec_y / length
    u1_x = -u0_y
    u1_y = u0_x

    # Phase 1: first-vertex-of-each-inner clearance (port of the C++).
    # shapely's ``interiors`` is a sequence of LinearRings; the first
    # coordinate of each is the first vertex of that hole.
    for inner in space.interiors:
        coords = list(inner.coords)
        if not coords:
            continue
        first = coords[0]
        vx = first[0] - p0.x
        vy = first[1] - p0.y
        ip0 = u0_x * vx + u0_y * vy
        ip1 = u1_x * vx + u1_y * vy
        if 0.0 < ip0 < length and abs(ip1) < agent_geometry:
            return False

    # Phase 2: rectangle-of-width-2r intersection.
    d_x = agent_geometry * u1_x
    d_y = agent_geometry * u1_y
    rect_pts = [
        (p0.x - d_x, p0.y - d_y),
        (p1.x - d_x, p1.y - d_y),
        (p1.x + d_x, p1.y + d_y),
        (p0.x + d_x, p0.y + d_y),
    ]
    rect_boundary = LineString(rect_pts)
    # ``intersects`` is True if the two geometries share any point
    # (including a single shared endpoint). Matches the C++ semantics
    # of ``boost::geometry::intersects``.
    if rect_boundary.intersects(space.boundary):
        return False
    return True


def get_bound(space: Polygon, axis: int) -> Tuple[float, float]:
    """Mirror of Space2D::getBound (geometry_2D_for_generator.hpp:100-112).

    Returns ``(min, max)`` of the outer-ring coordinate along ``axis``
    (0 = x, 1 = y). The C++ skips the first vertex of the outer ring
    (the closing duplicate of the last), but the min and max of a closed
    ring are the same with or without the duplicate. We use the full
    coordinate sequence.

    Note: this returns the BBox of the outer ring only. The C++ does
    not look at the inners for the bound; Grid.cpp uses this bound to
    position grid cells across the entire BBox and the per-cell
    ``isInside`` test discards cells that fall in holes.
    """
    xs = [x for x, _ in space.exterior.coords]
    ys = [y for _, y in space.exterior.coords]
    if axis == 0:
        return (min(xs), max(xs))
    elif axis == 1:
        return (min(ys), max(ys))
    else:
        raise ValueError(f"axis must be 0 (x) or 1 (y); got {axis}")


def get_random_points(
    space: Polygon, agent_geometry: float, number: int
) -> List[Point]:
    """Mirror of Space2D::getRandomPoints (geometry_2D_for_generator.hpp:114-141).

    Rejection sampling: pick (x, y) uniformly in the BBox minus a
    ``agent_geometry`` margin on every side, accept iff
    ``check_vertex_inside`` is True. The C++ uses
    ``std::random_device + std::default_random_engine`` (unseeded);
    Python uses the stdlib ``random`` module with no seed set, also
    unseeded. Two runs of either reference produce different samples
    (PORTING.md §6 trap 7).
    """
    (min_x, max_x), (min_y, max_y) = get_bound(space, 0), get_bound(space, 1)
    lo_x = min_x + agent_geometry
    hi_x = max_x - agent_geometry
    lo_y = min_y + agent_geometry
    hi_y = max_y - agent_geometry
    out: List[Point] = []
    while len(out) < number:
        x = random.uniform(lo_x, hi_x)
        y = random.uniform(lo_y, hi_y)
        if check_vertex_inside(space, Point(x, y), agent_geometry):
            out.append(Point(x, y))
    return out


def get_separate_random_points(
    space: Polygon, agent_geometry: float, number: int
) -> List[Point]:
    """Mirror of Space2D::getSepareteRandomPoints (:142-178).

    As ``get_random_points`` but also requires each new point to be at
    distance >= ``2 * agent_geometry`` from every previously accepted
    point. Used to seed random samples that won't immediately cluster.
    """
    (min_x, max_x), (min_y, max_y) = get_bound(space, 0), get_bound(space, 1)
    lo_x = min_x + agent_geometry
    hi_x = max_x - agent_geometry
    lo_y = min_y + agent_geometry
    hi_y = max_y - agent_geometry
    out: List[Point] = []
    sep = 2.0 * agent_geometry
    sep_sq = sep * sep
    while len(out) < number:
        x = random.uniform(lo_x, hi_x)
        y = random.uniform(lo_y, hi_y)
        candidate = Point(x, y)
        if not check_vertex_inside(space, candidate, agent_geometry):
            continue
        ok = True
        for prev in out:
            if (prev.x - candidate.x) ** 2 + (prev.y - candidate.y) ** 2 < sep_sq:
                ok = False
                break
        if ok:
            out.append(candidate)
    return out


def calculate_free_volume(space: Polygon, agent_geometry: float) -> float:
    """Mirror of Space2D::calculateFreeVolume (geometry_2D_for_generator.hpp:182-191).

    The free area available to a disc of radius ``agent_geometry`` to
    move inside ``space`` -- the polygon's area minus a strip of width
    ``agent_geometry`` along every boundary segment, plus the round-cap
    term ``pi * r^2 * (1 + num_inners)`` (which compensates for the
    corner over-subtraction in the strip model).

    The result can be slightly negative for very small or very concave
    polygons (the formula is an approximation). The C++ does not clamp.
    """
    area = space.area
    boundary_length = space.length  # shapely: outer + inner total length
    area -= agent_geometry * boundary_length
    area -= math.pi * agent_geometry * agent_geometry * (1 + len(space.interiors))
    return area
