"""Stage 0 -- the data model.

Port of
  PSIPP-CTC_src/src/base/problem.hpp            (core: lines 17-70, 76-88)
  PSIPP-CTC_src/src/base/problem_with_geometry.hpp

A *roadmap* is a directed adjacency list: ``edges[source] = [target, ...]``.
The identity of an edge is its index in that out-list -- there is no separate
``Edge`` object. Edge length equals travel time at unit speed; a
``Move.start_time`` is a real number in continuous time; a wait is encoded as
``edge_id < 0``.

The geometry subclass adds 2D Euclidean coordinates and an agent radius.
The reference templates ``ProblemInstanceWithGeometry<Geometry>``; in Python
the only geometry used is 2D Euclidean, so the 2D case is hardcoded here. If
another geometry backend appears later, parameterise via a ``Protocol``.

Collision / interval / validity methods are abstract at this stage. They are
filled in by Stage 2 (geometry kernels), Stage 4 (explicit-collisions
materialisation), and Stage 1 (I/O print formats).

Preserve exactly
----------------
* Edge id IS the out-list index. ``add_edge`` appends.
* ``agent_geometry`` is the disc *radius*; the conflict threshold is
  ``2 * radius`` (strict ``<`` in the geometry kernels).
* ``get_edge_length(source, edge_id)`` is the Euclidean distance to the target
  vertex. This is why length equals time: the reference moves at unit speed.
* ``Move.start_time`` is a ``float``, not a step index. Waiting is
  ``edge_id < 0``.
* ``ProblemInstance.EPS = 1e-8``. The geometry module's own ``EPS = 0`` is a
  *different* constant in a *different* class; do not unify them.
* ``set_number_of_vertex`` on the geometry subclass resizes **both**
  ``coordinates`` and ``edges``; the base version resizes only ``edges``. The
  two behaviours must be on the right class, or the two arrays silently
  desync (PORTING.md §3 trap 1).
"""

from __future__ import annotations

import math
import time
from typing import List, NamedTuple, Tuple


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


class Point(NamedTuple):
    """2D point. Mirrors the reference's ``Geometry::Point`` -- a struct with
    ``x`` and ``y`` members. The geometry kernels index it as ``p.x`` and
    ``p.y``; using ``NamedTuple`` keeps that attribute access in Python."""
    x: float
    y: float


class AgentTask(NamedTuple):
    """One agent's start and goal vertex. Mirrors ``ProblemInstance::AgentTask``."""
    initial_vertex: int
    goal_vertex: int


class Move(NamedTuple):
    """One step of a plan.

    ``edge_id`` is the index into the source vertex's out-list, **or negative
    for a wait** at the current vertex. ``start_time`` is a real number in
    continuous time, not a step index. Together they form a time-parameterised
    leg of the agent's route.
    """
    edge_id: int
    start_time: float


#: A plan is one list of moves per agent, in agent order.
Plan = List[List[Move]]


# ---------------------------------------------------------------------------
# ProblemInstance
# ---------------------------------------------------------------------------


class ProblemInstance:
    """Roadmap + agent tasks, with no geometry.

    The adjacency list ``edges[source] = [target, ...]`` IS the graph; an edge
    is identified by ``(source, index)``. ``add_edge`` appends, so the order
    of insertion **is** the edge id. This is the contract every downstream
    stage and the text I/O format depend on. Do not introduce an ``Edge``
    class (PORTING.md §3 trap 2).

    Methods that depend on geometry (``get_edge_length``, ``get_coordinate_*``,
    ``get_agent_radius``, all ``collision_check`` / ``get_collision_interval``
    overloads, ``check_plan_validity``, ``print_vertex``) are abstract here
    and are filled in by subclasses -- first by the geometry subclass, later
    by the explicit-collisions subclass.
    """

    #: Numerical infinity used in collision / interval arithmetic.
    INF: float = math.inf

    #: Epsilon used by the data model. The reference defines this as
    #: ``const double EPS = 1e-8`` on ``ProblemInstance``. Note: the geometry
    #: module's own ``EPS = 0`` is a *different* constant in a *different*
    #: class; do not unify them (PORTING.md §5 trap 5).
    EPS: float = 1e-8

    def __init__(self) -> None:
        # Adjacency list. ``edges[v][k]`` is the k-th out-neighbour of ``v``;
        # ``k`` is the edge id. ``add_edge`` appends, so insertion order IS
        # the edge id.
        self.edges: List[List[int]] = []

        # One ``AgentTask`` per agent. Order is the agent's index.
        self.tasks: List[AgentTask] = []

        # Counters and reentrant-timer state, mirroring problem.hpp:71-74.
        self.number_of_collision_checks: int = 0
        self.number_of_interval_calculations: int = 0
        # ``used_time`` is a float (seconds) here, in place of the reference's
        # ``std::chrono::duration<double>``. The semantics are the same: a
        # cumulative accumulator that only the outermost ``timer_on`` /
        # ``timer_off`` pair contributes to.
        self.used_time: float = 0.0
        self.timer_time: float = 0.0
        self.timer_switch: int = 0

    # -- Topology (concrete, no geometry needed) -------------------------

    def set_number_of_vertex(self, n: int) -> None:
        """Resize the adjacency list to ``n`` vertices.

        This is the **base** behaviour (``problem.hpp:23-27``): only
        ``edges`` is resized. The geometry subclass **overrides** this to
        also resize ``coordinates``; that override is what keeps the two
        arrays in lock-step. A base call on a geometry instance would leave
        ``coordinates`` at its old size and silently desync (PORTING.md §3
        trap 1).
        """
        self.edges = [[] for _ in range(n)]

    def add_edge(self, source: int, target: int) -> None:
        """Append a directed edge from ``source`` to ``target``.

        The new edge's id becomes the current out-list size at ``source``
        (i.e. ``len(self.edges[source]) - 1`` after the call).
        """
        self.edges[source].append(target)

    def get_number_of_vertices(self) -> int:
        return len(self.edges)

    def get_out_degree(self, vertex: int) -> int:
        return len(self.edges[vertex])

    def get_target_vertex(self, source: int, edge_id: int) -> int:
        return self.edges[source][edge_id]

    # -- Abstract geometry-dependent accessors ---------------------------

    def get_edge_length(self, source: int, edge_id: int) -> float:
        """Edge length = travel time at unit speed. Filled in by the geometry
        subclass as the Euclidean distance to the target vertex. This is the
        property that makes the A*'s Euclidean heuristic *admissible* -- see
        HANDOFF.md §5 invariant 4."""
        raise NotImplementedError

    def get_coordinate_x(self, vertex: int) -> float:
        raise NotImplementedError

    def get_coordinate_y(self, vertex: int) -> float:
        raise NotImplementedError

    def get_agent_radius(self) -> float:
        raise NotImplementedError

    def print_vertex(self, file, vertex: int) -> None:
        raise NotImplementedError

    # -- Abstract collision / interval / validity interface ---------------
    #
    # The C++ base carries the multi-arg dispatchers concretely; Python has no
    # static dispatch, so the dispatchers and the kernels are both deferred
    # to Stage 2 (geometry). Stage 0 only exposes the surface; downstream
    # stages decide how to implement the kernels and the dispatchers.

    def collision_check(self, *args, **kwargs) -> bool:  # type: ignore[no-untyped-def]
        """Stage 2 (geometry) implements the vertex/edge kernels and the
        multi-arg dispatchers (see ``problem.hpp:90-138``)."""
        raise NotImplementedError

    def collision_check2(self, *args, **kwargs) -> bool:  # type: ignore[no-untyped-def]
        raise NotImplementedError

    def get_collision_interval(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        raise NotImplementedError

    def get_collision_interval2(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        raise NotImplementedError

    def check_plan_validity(self, plan: Plan):  # type: ignore[no-untyped-def]
        """Stage 2 implements; the geometry subclass then overrides with the
        event-sweep variant from ``problem_with_geometry.hpp:207-292``."""
        raise NotImplementedError

    def get_all_vv_collisions(self, vertex: int) -> List[int]:
        raise NotImplementedError

    def get_all_ve_collisions(
        self, vertex: int
    ) -> List[Tuple[int, int, float, float]]:
        raise NotImplementedError

    def get_all_ev_collisions(
        self, vertex: int, edge_id: int
    ) -> List[Tuple[int, float, float]]:
        raise NotImplementedError

    def get_all_ee_collisions(
        self, vertex: int, edge_id: int
    ) -> List[Tuple[int, int, float, float]]:
        raise NotImplementedError

    # -- Reentrant timer (problem.hpp:76-88) ------------------------------

    def timer_on(self) -> None:
        """Start (or nest) a timed region.

        Reentrant: the reference's ``if (timer_switch++ == 0)`` guard means
        the start time is recorded **only on the outermost call**. Nested
        ``timer_on`` increments the switch but does not overwrite the
        recorded time, so the matching ``timer_off`` can still measure the
        full outer interval.

        Pairing: every ``timer_on`` must be matched by a ``timer_off`` that
        receives the value to return. The Python port follows the same
        pattern as the C++ template, including the rule that ``timer_off``
        is called exactly as many times as ``timer_on``.
        """
        if self.timer_switch == 0:
            self.timer_time = time.perf_counter()
        self.timer_switch += 1

    def timer_off(self, ret):  # type: ignore[no-untyped-def]
        """Close a timed region and return ``ret`` unchanged.

        Only the outermost pair -- the one that brings ``timer_switch`` back
        to zero -- adds to ``used_time``. The returned ``ret`` is forwarded
        as-is, so callers can wrap a return value inline:
        ``return self.timer_off(self.do_work())``.
        """
        self.timer_switch -= 1
        if self.timer_switch == 0:
            self.used_time += time.perf_counter() - self.timer_time
        return ret


# ---------------------------------------------------------------------------
# ProblemInstanceWithGeometry
# ---------------------------------------------------------------------------


class ProblemInstanceWithGeometry(ProblemInstance):
    """Roadmap + tasks + 2D coordinates + agent radius.

    The reference templates this class over a ``Geometry`` concept
    (``Point``, ``AgentGeometry``, ``distance``, ``printPoint``,
    ``CollisionCheck``, ``getCollisionInterval``). In Python the only
    geometry used is 2D Euclidean, so the type parameters are hardcoded:

    * ``Point`` -- 2D ``(x, y)``.
    * ``AgentGeometry`` -- a single ``float`` (the disc radius).
    * ``distance`` -- Euclidean (``math.hypot``).

    Subclasses (e.g. the explicit-collisions subclass in Stage 4) override
    the collision / interval methods to return table-lookup answers.

    The override of ``set_number_of_vertex`` is load-bearing: it must resize
    **both** ``coordinates`` and ``edges`` in lock-step. The base version
    resizes only ``edges``, so the two arrays would silently desync if a
    caller ever fell back to the base implementation. Do not remove this
    override.
    """

    def __init__(self) -> None:
        super().__init__()
        # 2D point per vertex. Resized by ``set_number_of_vertex``.
        self.coordinates: List[Point] = []
        # Disc radius. The conflict threshold is ``2 * radius`` (strict).
        self.agent_geometry: float = 0.0

    # -- Override: resize BOTH arrays in lock-step -----------------------

    def set_number_of_vertex(self, n: int) -> None:
        """Resize ``coordinates`` and ``edges`` together.

        Trap (PORTING.md §3, trap 1): the base version
        (``problem.hpp:23-27``) resizes only ``edges``; the geometry version
        (``problem_with_geometry.hpp:26-30``) resizes both. If a caller used
        the base method on a geometry instance, ``coordinates`` would be
        shorter than ``edges`` and every later ``coordinates[v]`` access
        would either fail or read garbage.
        """
        self.coordinates = [Point(0.0, 0.0) for _ in range(n)]
        self.edges = [[] for _ in range(n)]

    def set_point(self, vertex: int, point: Point) -> None:
        self.coordinates[vertex] = point

    def get_point(self, vertex: int) -> Point:
        return self.coordinates[vertex]

    def get_coordinate_x(self, vertex: int) -> float:
        return self.coordinates[vertex].x

    def get_coordinate_y(self, vertex: int) -> float:
        return self.coordinates[vertex].y

    def get_agent_radius(self) -> float:
        return self.agent_geometry

    def get_edge_length(self, source: int, edge_id: int) -> float:
        """Unit-speed assumption: edge length = travel time.

        The C++ uses ``Geometry::distance(coordinates[source], target_point)``;
        in the 2D Euclidean case this is just the Euclidean distance. The
        only reason the A*'s Euclidean heuristic is *admissible* is this
        equality -- see HANDOFF.md §5 invariant 4. Do not introduce a
        separate speed model here; that is what the kinematics extension in
        Phase 3 is for.
        """
        target = self.edges[source][edge_id]
        sp = self.coordinates[source]
        tp = self.coordinates[target]
        return math.hypot(tp.x - sp.x, tp.y - sp.y)
