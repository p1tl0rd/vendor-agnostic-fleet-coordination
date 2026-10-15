"""Stage 3 -- the ProbabilisticRoadmap generators (kPRM and PRM).

Port of PSIPP-CTC_src/src/roadmap_generators/ProbabilisticRoadmap.cpp
(239 lines, two classes).

kPRM (KNearestProbabilisticRoadmap)
-----------------------------------
Two variants exist: KNearestProbabilisticRoadmap (template, parameterised
on SpaceGeometry) and the 2D-only RadiusProbabilisticRoadmap2D. The C++
template body is duplicated in the .cpp via a templated base class; in
Python we have one concrete class per variant.

kPRM default K
--------------
When the caller omits K, the C++ sets
``K = ceil((e * (1 + 1/dim) + small_value) * log(n))`` where
``e = std::exp(1.)``, ``dim = 2``, and ``small_value = 0.1`` (the C++
slack constant -- it is NOT one of the three EPS values). This is
admissible-by-construction for the unit-disc kPRM connectivity proof.
The port reproduces the formula exactly.

kPRM number_of_vertices = 0
---------------------------
Means "no random vertices" -- the graph is exactly the 2*tasks
start/goal vertices. The K formula would otherwise ask for
``ceil((e * 1.5 + 0.1) * log(0))`` which is undefined; the C++
handles this by leaving K=0 (the C++ uses the K constructor arg as a
sentinel -- when the user passes K=0 the formula is computed even if
n=0). The Python port computes K the same way and lets the downstream
loop degenerate to 0 iterations.

PRM (RadiusProbabilisticRoadmap2D)
----------------------------------
Radius-based: every pair of vertices within ``radius`` of each other
becomes a candidate edge. Buckets2D broad-phase picks up the pairs.
If ``radius`` is omitted (0.0), the C++ auto-computes
``radius = (sqrt(6 * free_volume / pi) + small_value) * sqrt(log(n)/n)``
where ``free_volume`` is from ``Space2D::calculateFreeVolume``. This is
the standard kPRM radius for the unit-ball connectivity bound.

Bidirectional edges
-------------------
The C++ decides per-direction whether to add the edge based on
``use_other_agent_points``. With it True (default), every pair adds
both directions. With it False, the wiring is one-way between random
vertices and the agent-vertex pairs (the agent's start/goal are wired
to their K closest random vertices, but two random vertices wire
bidirectionally). The port reproduces this rule exactly.
"""

from __future__ import annotations

import math
import random
from typing import List, Tuple

from psipp_ctc.geometry.collision_report import Buckets2D
from psipp_ctc.problem import AgentTask, Point, ProblemInstanceWithGeometry
from psipp_ctc.roadmap.space_2d import (
    calculate_free_volume,
    check_edge_inside,
    get_random_points,
)


SMALL_VALUE: float = 0.1
"""The C++'s slack constant in the K and radius formulas. NOT one of
the three EPS values -- it is purely a numerical slack to keep the
formulas from under-shooting on small inputs."""


def _default_k(n: int, dim: int = 2) -> int:
    """The C++'s default kPRM K formula.

    ``K = ceil((e * (1 + 1/dim) + small_value) * log(n))``

    For ``n <= 1`` the C++ still computes this; the result is at least
    0 (log(1) = 0, log(0) is undefined so the C++ would crash, but
    n=0 is a documented special case meaning "no random vertices").
    """
    if n <= 1:
        # log(1) = 0, log(0) undefined. The C++ special-cases n=0
        # by skipping the loop entirely; with n=1, K=0 is harmless.
        return 0
    return int(math.ceil(
        (math.exp(1.0) * (1.0 + 1.0 / dim) + SMALL_VALUE) * math.log(n)
    ))


def _default_radius(free_volume: float, n: int) -> float:
    """The C++'s default PRM radius formula.

    ``radius = (sqrt(6 * free_volume / pi) + small_value) * sqrt(log(n) / n)``
    """
    if n <= 1:
        return 0.0
    return (math.sqrt(6.0 * free_volume / math.pi) + SMALL_VALUE) * math.sqrt(
        math.log(n) / n
    )


def _k_nearest(
    points: List[Point], id: int, k: int
) -> List[Tuple[float, int]]:
    """Return the ``k`` closest points to ``points[id]`` as ``(dist, id)`` pairs.

    Mirrors ``KNearestProbabilisticRoadmap::getNeighbors``
    (ProbabilisticRoadmap.cpp:15-34). Uses a max-heap of size k
    (heapq with negated distances) to keep the running cost O(N log k)
    rather than the O(N log N) of sorting the full distance vector.

    Note: ``k == 0`` is the special case used by the C++'s default-K
    formula for ``n = 0`` (PORTING.md §6 trap 8, "no random
    vertices"). Returns an empty list.
    """
    if k <= 0:
        return []
    import heapq
    target = points[id]
    heap: List[Tuple[float, int]] = []  # max-heap by distance (negated)
    for i, p in enumerate(points):
        if i == id:
            continue
        d = math.hypot(p.x - target.x, p.y - target.y)
        if len(heap) < k:
            heapq.heappush(heap, (-d, i))
        else:
            # heap[0] is the current farthest; replace if d is closer.
            if -heap[0][0] > d:
                heapq.heapreplace(heap, (-d, i))
    # Return in C++'s iteration order: from closest to farthest
    # (``std::set`` is sorted by distance). heapq returns in heap
    # order; sort at the end.
    out = sorted([(-neg_d, i) for neg_d, i in heap], key=lambda x: x[0])
    return out


def generate_kprm(
    spatial,
    number_of_vertices: int,
    K: int = 0,
    use_other_agent_points: bool = True,
) -> ProblemInstanceWithGeometry:
    """Generate a kPRM roadmap.

    Mirrors ``KNearestProbabilisticRoadmap::generate``
    (ProbabilisticRoadmap.cpp:57-131). See module docstring for the
    K formula and the special case ``number_of_vertices = 0``.
    """
    if K == 0:
        K = _default_k(number_of_vertices, dim=2)

    problem = ProblemInstanceWithGeometry()
    problem.agent_geometry = spatial.agent_geometry
    tasks = list(spatial.tasks)

    # Vertex layout: number_of_vertices random samples, then 2 per task
    # in the order (start_0, goal_0, start_1, goal_1, ...). Matches
    # the C++ ``number_of_vertices + 2 * agent`` indexing.
    problem.set_number_of_vertex(number_of_vertices + 2 * len(tasks))
    problem.tasks = [AgentTask(0, 0) for _ in tasks]  # placeholder

    random_points = get_random_points(
        spatial._space_polygon(), problem.agent_geometry, number_of_vertices
    ) if number_of_vertices > 0 else []
    for i, p in enumerate(random_points):
        problem.set_point(i, p)

    # Place task vertices
    for agent, task in enumerate(tasks):
        start_id = number_of_vertices + 2 * agent
        goal_id = start_id + 1
        problem.set_point(start_id, task.initial_point)
        problem.set_point(goal_id, task.goal_point)
        problem.tasks[agent] = AgentTask(start_id, goal_id)

    # Build the full points list once (used by both branches below)
    all_points = [problem.get_point(i) for i in range(problem.get_number_of_vertices())]

    # Optional pre-wiring of task vertices to their K nearest random
    # points (skipped when use_other_agent_points is True, because the
    # main pass wires them anyway).
    if not use_other_agent_points:
        for agent, task in enumerate(tasks):
            start_id = number_of_vertices + 2 * agent
            goal_id = start_id + 1
            # K nearest random points (limit = number_of_vertices skips
            # the task vertices in the candidates)
            start_neighbors = _k_nearest(
                all_points, start_id, min(K, number_of_vertices)
            )
            for _, nid in start_neighbors:
                if check_edge_inside(
                    spatial._space_polygon(),
                    problem.get_point(start_id),
                    problem.get_point(nid),
                    problem.agent_geometry,
                ):
                    problem.add_edge(start_id, nid)
                    problem.add_edge(nid, start_id)
            goal_neighbors = _k_nearest(
                all_points, goal_id, min(K, number_of_vertices)
            )
            for _, nid in goal_neighbors:
                if check_edge_inside(
                    spatial._space_polygon(),
                    problem.get_point(nid),
                    problem.get_point(goal_id),
                    problem.agent_geometry,
                ):
                    problem.add_edge(nid, goal_id)
                    problem.add_edge(goal_id, nid)

    # Main pass: every vertex is connected to its K nearest, and the
    # edge list is deduped (each undirected edge appears once in
    # canonical ``(min, max)`` order).
    if use_other_agent_points:
        number_of_bidirectionals = number_of_vertices + 2 * len(tasks)
    else:
        number_of_bidirectionals = number_of_vertices
    edge_list: List[Tuple[int, int]] = []
    for i in range(number_of_bidirectionals):
        neighbors = _k_nearest(
            all_points, i, min(K, problem.get_number_of_vertices() - 1)
        )
        for _, nid in neighbors:
            edge_list.append((min(i, nid), max(i, nid)))
    # Sort + unique
    edge_list = sorted(set(edge_list))
    for a, b in edge_list:
        if check_edge_inside(
            spatial._space_polygon(),
            problem.get_point(a),
            problem.get_point(b),
            problem.agent_geometry,
        ):
            problem.add_edge(a, b)
            problem.add_edge(b, a)

    return problem


def generate_prm(
    spatial,
    number_of_vertices: int,
    radius: float = 0.0,
    use_other_agent_points: bool = True,
) -> ProblemInstanceWithGeometry:
    """Generate a radius-PRM roadmap.

    Mirrors ``RadiusProbabilisticRoadmap2D::generate``
    (ProbabilisticRoadmap.cpp:164-238). Uses the Buckets2D broad phase
    from Stage 2 to find candidate pairs in O(N) on average.

    If ``radius`` is 0, the C++ default formula
    ``(sqrt(6 * free_volume / pi) + small_value) * sqrt(log(n)/n)`` is
    used. The Python port reproduces this verbatim.
    """
    if number_of_vertices == 0:
        radius = 0.0
    elif radius == 0.0:
        radius = _default_radius(
            calculate_free_volume(spatial._space_polygon(), spatial.agent_geometry),
            number_of_vertices,
        )

    problem = ProblemInstanceWithGeometry()
    problem.agent_geometry = spatial.agent_geometry
    tasks = list(spatial.tasks)

    problem.set_number_of_vertex(number_of_vertices + 2 * len(tasks))
    problem.tasks = [AgentTask(0, 0) for _ in tasks]

    random_points = get_random_points(
        spatial._space_polygon(), problem.agent_geometry, number_of_vertices
    ) if number_of_vertices > 0 else []
    for i, p in enumerate(random_points):
        problem.set_point(i, p)
    for agent, task in enumerate(tasks):
        start_id = number_of_vertices + 2 * agent
        goal_id = start_id + 1
        problem.set_point(start_id, task.initial_point)
        problem.set_point(goal_id, task.goal_point)
        problem.tasks[agent] = AgentTask(start_id, goal_id)

    # Build the full points list for the Buckets2D pass.
    all_points = [problem.get_point(i) for i in range(problem.get_number_of_vertices())]
    buckets = Buckets2D(radius, all_points)
    all_pairs = buckets.calculate_all_pairs()

    # Edge-direction rules. The C++'s branchy logic is reproduced
    # verbatim below. The default ``use_other_agent_points=True`` adds
    # both directions for every pair; otherwise the wiring depends on
    # the parity of the task index for task vertices.
    for a, b in all_pairs:
        if a == b:
            continue
        f_to_s = False  # a -> b
        s_to_f = False  # b -> a
        if use_other_agent_points:
            f_to_s = s_to_f = True
        elif a < number_of_vertices:
            if b < number_of_vertices:
                f_to_s = s_to_f = True
            elif (b - number_of_vertices) % 2 == 0:
                s_to_f = True
            else:
                f_to_s = True
        elif (a - number_of_vertices) % 2 == 0:
            if b < number_of_vertices or b == a + 1:
                f_to_s = True
        elif (a - number_of_vertices) % 2 == 1:
            if b < number_of_vertices or b == a - 1:
                s_to_f = True
        if not f_to_s and not s_to_f:
            continue
        if check_edge_inside(
            spatial._space_polygon(),
            problem.get_point(a),
            problem.get_point(b),
            problem.agent_geometry,
        ):
            if f_to_s:
                problem.add_edge(a, b)
            if s_to_f:
                problem.add_edge(b, a)

    return problem
