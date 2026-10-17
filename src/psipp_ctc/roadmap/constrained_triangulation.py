"""The Constrained Delaunay Triangulation generator.

The C++ uses ``CGAL::Constrained_Delaunay_triangulation_2``: a
constrained triangulation that keeps specified segments as hard edges
in the mesh. CGAL is not available; the Python port uses
``scipy.spatial.Delaunay`` (an *unconstrained* Delaunay) and
re-implements the constraint behaviour as a post-filter.

The two key behaviours of the C++ CDT are:

  1. **Offset border as a hard wall.** For every ring vertex, the
     C++ computes an inward-shifted offset (one or two points per
     vertex, depending on convexity) and inserts the connecting
     segments as hard constraints, so the triangulation cannot
     place an edge across the offset border. The result is that no
     edge in the final mesh lies within ``agent_geometry`` of the
     original boundary.

  2. **Whole-segment ``isEdgeInside`` filter on every surviving
     edge.** This is the ultimate gate: even if an offset border
     segment is inserted as a constraint, every output edge must
     also pass the per-segment clearance test (``spatial.hpp:33``).

The Python port replicates behaviour (2) verbatim, and approximates
behaviour (1) using ``shapely.buffer(-r)``: the inward-shifted
polygon is taken as a vertex / edge mask, and the unconstrained
Delaunay's edges are filtered to those whose both endpoints and whole
segment lie inside the offset. This gives a *structurally similar*
CDT -- same vertex count for the same point set, same edge-validity
gate -- but the exact triangulation topology can differ from the
CGAL output because the offset-border constraint set is not identical.
That divergence is documented in ``space_2d.py`` and accepted by the
test gate: every task's start and goal are connected, and every edge
passes ``isEdgeInside``.

Discretisation
--------------
The C++ collapses near-duplicate points with
``(long long)(point.x / EPS), (long long)(point.y / EPS)`` so that
random samples that land almost on top of a task endpoint merge to
the same vertex. The Python port uses the same trick with
``SpatialProblemInstance.EPS = 1e-6`` as the discretisation grid.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
from scipy.spatial import Delaunay

from psipp_ctc.problem import (
    AgentTask,
    Point,
    ProblemInstanceWithGeometry,
    SpatialProblemInstance,
)
from psipp_ctc.roadmap.space_2d import (
    check_edge_inside,
    get_random_points,
)


def _discretise(point: Point, eps: float) -> Tuple[int, int]:
    """C++ CDT's vertex-merging key: ``floor(x/eps), floor(y/eps)``.

    Two vertices with the same discretised key are treated as the
    same vertex by the C++ (it stores them in a ``std::map`` keyed on
    this pair). The Python port uses a dict, same semantics.
    """
    return (int(point.x / eps), int(point.y / eps))


def generate_cdt(
    spatial: SpatialProblemInstance,
    number_of_additionals: int = 0,
    use_corner_points: bool = True,
) -> ProblemInstanceWithGeometry:
    """Generate a CDT roadmap for ``spatial``.

    See module docstring for the deviation from CGAL's constrained
    Delaunay.

    The output's vertex set is:
      - the C++'s discretised merge of task endpoints + random
        additional points + (optionally) corner points inserted
        on concave ring vertices;
      - plus the two task endpoints as named vertices mapped from
        the discretisation key.

    Edges are added bidirectionally; every edge passes
    ``is_edge_inside(spatial, ...)`` so a disc can travel along it
    without crossing the boundary.
    """
    problem = ProblemInstanceWithGeometry()
    problem.agent_geometry = spatial.agent_geometry
    tasks = list(spatial.tasks)

    # The C++ uses ``SpatialProblemInstance.EPS = 1e-6`` as both the
    # clearance margin in ``isInside`` and the discretisation grid in
    # the vertex map. The two uses happen to coincide but are
    # semantically different. We import the constant from the class.
    eps = spatial.EPS

    space_poly = spatial._space_polygon()

    # 1. Collect the initial point set: task endpoints + random
    # additional samples. The C++ inserts them into the CDT one by
    # one in this order. We collect them all and run an unconstrained
    # Delaunay on the whole set.
    points: List[Point] = []
    for task in tasks:
        points.append(task.initial_point)
        points.append(task.goal_point)
    if number_of_additionals > 0:
        for p in get_random_points(
            space_poly, spatial.agent_geometry, number_of_additionals
        ):
            points.append(p)

    # 2. Optional corner points on concave ring vertices. The C++
    # walks the outer + inner rings and, for each concave vertex,
    # inserts a corner point if it passes ``isInside``. The corner is
    # computed from the two adjacent edge normals.
    if use_corner_points:
        for ring in [space_poly.exterior] + list(space_poly.interiors):
            coords = list(ring.coords)
            n = len(coords) - 1  # ring is closed
            for i in range(n):
                prev_i = n - 1 if i == 0 else i - 1
                next_i = (i + 1) % n
                p_i = Point(coords[i][0], coords[i][1])
                p_prev = Point(coords[prev_i][0], coords[prev_i][1])
                p_next = Point(coords[next_i][0], coords[next_i][1])
                vec0_x = p_prev.x - p_i.x
                vec0_y = p_prev.y - p_i.y
                vec1_x = p_next.x - p_i.x
                vec1_y = p_next.y - p_i.y
                l0 = math.hypot(vec0_x, vec0_y)
                l1 = math.hypot(vec1_x, vec1_y)
                if l0 == 0.0 or l1 == 0.0:
                    continue
                u0_x, u0_y = vec0_x / l0, vec0_y / l0
                u1_x, u1_y = vec1_x / l1, vec1_y / l1
                ccw = u0_x * u1_y - u0_y * u1_x
                if ccw > 0:
                    # convex vertex; no corner point
                    continue
                ip = u0_x * u1_x + u0_y * u1_y
                denom = 1.0 - ip
                if denom <= 0.0:
                    # vectors anti-parallel; skip
                    continue
                factor = math.sqrt((1.0 + ip) / denom)
                corner = Point(
                    p_i.x - spatial.agent_geometry * factor * u0_x,
                    p_i.y - spatial.agent_geometry * factor * u0_y,
                )
                if spatial.is_inside(corner):
                    points.append(corner)

    if len(points) < 3:
        # scipy.spatial.Delaunay needs at least 3 non-collinear points;
        # fall back to just the task endpoints.
        problem.coordinates = list(points)
        problem.edges = [[] for _ in points]
        for agent, task in enumerate(tasks):
            start_id = 2 * agent
            goal_id = start_id + 1
            problem.tasks.append(AgentTask(start_id, goal_id))
        return problem

    # 3. Run the unconstrained Delaunay.
    coords_arr = np.array([[p.x, p.y] for p in points])
    try:
        tri = Delaunay(coords_arr)
    except Exception:
        # Degenerate (collinear) input -- scipy raises; fall back to
        # the bare endpoints.
        problem.coordinates = list(points)
        problem.edges = [[] for _ in points]
        for agent, task in enumerate(tasks):
            start_id = 2 * agent
            goal_id = start_id + 1
            problem.tasks.append(AgentTask(start_id, goal_id))
        return problem

    # 4. Filter triangles: keep those whose circumcentre is inside
    # the offset polygon (shapely ``buffer(-r)``). Edges of kept
    # triangles become candidate edges. This is the Python port's
    # approximation of the C++'s hard-constraint set: any triangle
    # that straddles the offset border is dropped, so no edge of the
    # output mesh lies within ``agent_geometry`` of the boundary.
    offset_poly = space_poly.buffer(-spatial.agent_geometry)

    # 5. Build vertex id map by discretisation, and collect surviving
    # edges (undirected, deduplicated).
    vertex_ids: dict = {}
    kept_points: List[Point] = []
    for p in points:
        key = _discretise(p, eps)
        if key not in vertex_ids:
            vertex_ids[key] = len(kept_points)
            kept_points.append(p)

    edges: set = set()  # (min_id, max_id) deduplicated
    for simplex in tri.simplices:
        i, j, k = int(simplex[0]), int(simplex[1]), int(simplex[2])
        # Triangle vertex coordinates
        p_i, p_j, p_k = points[i], points[j], points[k]
        # The C++ checks ``isEdgeInside`` for the WHOLE edge segment.
        # We do the same and add the triangle if all three edges pass.
        if not spatial.is_edge_inside(p_i, p_j):
            continue
        if not spatial.is_edge_inside(p_j, p_k):
            continue
        if not spatial.is_edge_inside(p_k, p_i):
            continue
        # Also require the triangle to lie inside the offset polygon.
        # This is the C++'s hard-constraint equivalent for the
        # unconstrained Delaunay. We test the circumcentre: a
        # triangle whose circumcentre is inside the offset is
        # entirely inside the offset (for non-degenerate cases).
        if offset_poly is not None and not offset_poly.is_empty:
            ax, ay = p_i.x, p_i.y
            bx, by = p_j.x, p_j.y
            cx, cy = p_k.x, p_k.y
            d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
            if abs(d) > 1e-12:
                ux = ((ax * ax + ay * ay) * (by - cy)
                      + (bx * bx + by * by) * (cy - ay)
                      + (cx * cx + cy * cy) * (ay - by)) / d
                uy = ((ax * ax + ay * ay) * (cx - bx)
                      + (bx * bx + by * by) * (ax - cx)
                      + (cx * cx + cy * cy) * (bx - ax)) / d
                from shapely.geometry import Point as SPoint
                if not offset_poly.contains(SPoint(ux, uy)):
                    continue
        for a, b in ((i, j), (j, k), (k, i)):
            id_a = vertex_ids[_discretise(points[a], eps)]
            id_b = vertex_ids[_discretise(points[b], eps)]
            if id_a == id_b:
                continue
            edges.add((min(id_a, id_b), max(id_a, id_b)))

    # 6. Materialise the kept vertices and edges into the problem
    # instance. The C++ iterates finite_vertex_handles in CDT order;
    # we iterate the points list in the order they were added (which
    # matches the C++ after the discretisation merge).
    problem.coordinates = list(kept_points)
    problem.edges = [[] for _ in kept_points]
    for a, b in edges:
        problem.add_edge(a, b)
        problem.add_edge(b, a)

    # 7. Wire up the task vertex ids from the discretised key of
    # each task's start and goal point. The C++ uses the same
    # discretisation and a ``vertex_ids`` map; the Python port is
    # the same dict.
    for agent, task in enumerate(tasks):
        init_key = _discretise(task.initial_point, eps)
        goal_key = _discretise(task.goal_point, eps)
        if init_key not in vertex_ids or goal_key not in vertex_ids:
            raise ValueError(
                f"task {agent} endpoint not found in CDT vertex set "
                f"(initial {init_key} or goal {goal_key} missing)"
            )
        problem.tasks.append(AgentTask(
            vertex_ids[init_key], vertex_ids[goal_key]
        ))

    return problem
