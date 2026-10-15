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

    # -- Dispatchers: route by edge_id<0 (wait) ---------------------------
    # Mirrors ``ProblemInstance::CollisionCheck`` (problem.hpp:116-138) and
    # the getCollisionInterval overloads at problem.hpp:185-207, 233-254.
    # These are the time-frame-mirroring dispatchers: a wait on agent 1 is
    # equivalent to a stationary check, and the time interval must be
    # mirrored back into the caller's frame. The math is in
    # :mod:`psipp_ctc.geometry.geometry_2d`; this class only does the
    # routing.

    def _collision_check_dispatch(
        self, sv0, st0, eid0, tt0, sv1, st1, eid1, tt1
    ) -> bool:
        """Route by ``edge_id < 0`` (waiting) to the right kernel.

        Mirrors ``ProblemInstance::CollisionCheck(sv0, st0, eid0, tt0, sv1, st1, eid1, tt1)``
        (``problem.hpp:116-138``). The four cases:

        * both waiting:    VV kernel.
        * 0 waits, 1 moves: VE kernel (VEC).
        * 0 moves, 1 waits: VE kernel, args swapped.
        * both moving:     EE kernel (EEC).
        """
        if eid0 < 0:
            if eid1 < 0:
                return self.collision_check_vv(sv0, sv1)
            return self.collision_check_ve(
                sv0, st0, tt0, sv1, st1, eid1
            )
        if eid1 < 0:
            return self.collision_check_ve(
                sv1, st1, tt1, sv0, st0, eid0
            )
        return self.collision_check_ee(
            sv0, st0, eid0, sv1, st1, eid1
        )

    def get_collision_interval(  # type: ignore[no-untyped-def,override]
        self, *args, **kwargs
    ):
        """Two overloads (time-frame mirroring on the second).

        ``get_collision_interval(v0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            Mirrors ``problem.hpp:185-207``. If ``eid1 < 0`` (agent 1 is
            waiting), the interval is ``[st1, tt1]`` if the agents
            conflict; otherwise the kernel is a no-op. If ``eid1 >= 0``,
            the VEC kernel is run and its ``[lower, upper]`` (relative
            to agent 1's clock) is shifted into the caller's frame by
            ``lower += st1, upper += st1``.

        ``get_collision_interval(sv0, eid0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            Mirrors ``problem.hpp:233-254``. If ``eid1 < 0``, the
            EEC kernel is run with agent 1's clock as the inner
            variable and the result is mirrored: ``lower = st1 - lower,
            upper = tt1 - upper`` (reverse-time convention; do NOT
            "fix" the signs -- see ``cpp-to-python`` skill rule 9). If
            ``eid1 >= 0``, the EEC kernel is run in the normal way
            and shifted by ``st1``.
        """
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

    # -- Stage 2 collision kernels (override the abstract base) ----------
    # Mirrors the template specialisations on
    # ``ProblemInstanceWithGeometry<Geometry2D>`` in problem.hpp / geometry_2D.hpp.
    # Three concrete kernels (VV, VE, EE) plus the time-frame-mirroring
    # ``get_collision_interval`` overloads from problem.hpp:185-254.
    # The actual VEC and EEC math is in :mod:`psipp_ctc.geometry.geometry_2d`;
    # this subclass only does the geometry lookups and the time frame
    # bookkeeping.
    #
    # Naming: Python has no C++ overloading, so the four argument shapes
    # are exposed as four methods. The base class' ``_collision_check_dispatch``
    # already does the time-mirror dispatching (waiting vs moving); the
    # kernels below are the concrete implementations it routes to.

    def collision_check_vv(self, v0: int, v1: int) -> bool:
        """Two static disc agents: ``distance < 2r - EPS``.

        Mirrors ``Geometry2D::CollisionCheck(p0, p1, r, EPS)`` and the
        abstract ``ProblemInstance::CollisionCheck(v0, v1)``. Uses
        ``self.EPS`` (1e-8) so a VV conflict is *slightly* more
        permissive than the geometric strict-2r check; this matches the
        data-model EPS convention used elsewhere in the planners.
        """
        from psipp_ctc.geometry.geometry_2d import collision_check_static
        p0 = self.coordinates[v0]
        p1 = self.coordinates[v1]
        return collision_check_static(
            p0, p1, self.agent_geometry, self.EPS
        )

    def collision_check_ve(
        self, v0: int, st0: float, tt0: float,
        sv1: int, st1: float, eid1: int,
    ) -> bool:
        """Stationary ``v0`` over ``[st0, tt0]`` vs moving edge ``eid1``
        from ``sv1``. Mirrors ``geometry_2D.hpp:150-162`` and the
        abstract ``ProblemInstance::CollisionCheck(v0, st0, tt0, sv1, st1, eid1)``.
        """
        from psipp_ctc.geometry.geometry_2d import collision_check_vertex_edge
        p0 = self.coordinates[v0]
        sp1 = self.coordinates[sv1]
        tv1 = self.edges[sv1][eid1]
        tp1 = self.coordinates[tv1]
        dt1 = self.get_edge_length(sv1, eid1)
        return collision_check_vertex_edge(
            p0, st0, tt0, sp1, st1, tp1, st1 + dt1,
            self.agent_geometry, self.EPS,
        )

    def collision_check_ee(
        self, sv0: int, st0: float, eid0: int,
        sv1: int, st1: float, eid1: int,
    ) -> bool:
        """Edge ``eid0`` from ``sv0`` vs edge ``eid1`` from ``sv1``,
        both at unit speed. Mirrors the abstract
        ``ProblemInstance::CollisionCheck(sv0, st0, eid0, sv1, st1, eid1)``
        and the geometry-2D ``CollisionCheck`` with two timed segments.
        """
        from psipp_ctc.geometry.geometry_2d import collision_check_moving
        sp0 = self.coordinates[sv0]
        tv0 = self.edges[sv0][eid0]
        tp0 = self.coordinates[tv0]
        sp1 = self.coordinates[sv1]
        tv1 = self.edges[sv1][eid1]
        tp1 = self.coordinates[tv1]
        dt0 = self.get_edge_length(sv0, eid0)
        dt1 = self.get_edge_length(sv1, eid1)
        return collision_check_moving(
            sp0, st0, tp0, st0 + dt0,
            sp1, st1, tp1, st1 + dt1,
            self.agent_geometry, self.EPS,
        )

    def get_collision_interval(  # type: ignore[no-untyped-def,override]
        self, *args, **kwargs
    ):
        """Four overloads: two kernel forms, two time-mirror forms.

        The arity picks the form:

        * 3 args ``(v0, sv1, eid1) -> (collide, lo, up)``
            VEC kernel form. Mirrors
            ``ProblemInstanceWithGeometry::getCollisionInterval(v0, sv1,
            eid1, &lo, &up)`` (``problem_with_geometry.hpp:106-119``).
            Returns the VEC interval ``[lo, up]`` *relative to agent
            1's start* (``st1 = 0``). If ``length == 0`` the kernel
            skips and returns ``(False, 0, 0)``; otherwise it runs
            ``get_collision_interval_vec`` exactly.

        * 4 args ``(sv0, eid0, sv1, eid1) -> (collide, lo, up)``
            EEC kernel form. Mirrors
            ``ProblemInstanceWithGeometry::getCollisionInterval(sv0,
            eid0, sv1, eid1, &lo, &up)`` (``problem_with_geometry.hpp:171-187``).
            Returns the EEC interval ``[lo, up]`` *relative to agent
            1's start*. Skips if either edge has length 0; otherwise
            runs ``get_collision_interval_eec`` exactly.

        * 5 args ``(v0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            VEC time-mirror form. Mirrors ``ProblemInstance::
            getCollisionInterval(v0, sv1, st1, eid1, tt1, &lo, &up)``
            (``problem.hpp:185-207``). If ``eid1 < 0`` (agent 1 is
            waiting), the interval is ``[st1, tt1]`` if the agents
            conflict; otherwise the kernel is a no-op. If ``eid1 >=
            0``, the VEC kernel is run and its ``[lower, upper]``
            (relative to agent 1's clock) is shifted into the
            caller's frame by ``lower += st1, upper += st1``.

        * 6 args ``(sv0, eid0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            EEC time-mirror form. Mirrors ``ProblemInstance::
            getCollisionInterval(sv0, eid0, sv1, st1, eid1, tt1, &lo,
            &up)`` (``problem.hpp:233-254``). If ``eid1 < 0``, the
            EEC kernel is run with agent 1's clock as the inner
            variable and the result is mirrored: ``lower = st1 -
            lower, upper = tt1 - upper`` (reverse-time convention;
            do NOT "fix" the signs -- see ``cpp-to-python`` skill
            rule 9). If ``eid1 >= 0``, the EEC kernel is run in the
            normal way and shifted by ``st1``.
        """
        from psipp_ctc.geometry.geometry_2d import (
            get_collision_interval_eec,
            get_collision_interval_vec,
        )
        n = len(args)
        if n == 3:
            # VEC kernel form: 3-arg call, no time frame, lo/up
            # relative to agent 1's clock (st1 = 0).
            v0, sv1, eid1 = args
            p0 = self.coordinates[v0]
            sp1 = self.coordinates[sv1]
            tv1 = self.edges[sv1][eid1]
            tp1 = self.coordinates[tv1]
            dt1 = self.get_edge_length(sv1, eid1)
            if dt1 < self.EPS:
                return (False, 0.0, 0.0)
            return get_collision_interval_vec(
                p0, sp1, tp1, dt1, self.agent_geometry
            )
        if n == 4:
            # EEC kernel form: 4-arg call, no time frame, lo/up
            # relative to agent 1's clock.
            sv0, eid0, sv1, eid1 = args
            sp0 = self.coordinates[sv0]
            tv0 = self.edges[sv0][eid0]
            tp0 = self.coordinates[tv0]
            sp1 = self.coordinates[sv1]
            tv1 = self.edges[sv1][eid1]
            tp1 = self.coordinates[tv1]
            dt0 = self.get_edge_length(sv0, eid0)
            dt1 = self.get_edge_length(sv1, eid1)
            if dt0 < self.EPS or dt1 < self.EPS:
                return (False, 0.0, 0.0)
            return get_collision_interval_eec(
                sp0, tp0, dt0, sp1, tp1, dt1, self.agent_geometry
            )
        if n == 5:
            v0, sv1, st1, eid1, tt1 = args
            if eid1 < 0:
                # Agent 1 is waiting: just check VV.
                collide = self.collision_check_vv(v0, sv1)
                return (collide, st1, tt1) if collide else (False, 0.0, 0.0)
            p0 = self.coordinates[v0]
            sp1 = self.coordinates[sv1]
            tv1 = self.edges[sv1][eid1]
            tp1 = self.coordinates[tv1]
            dt1 = self.get_edge_length(sv1, eid1)
            if dt1 < self.EPS:
                return (False, 0.0, 0.0)
            collide, lo, up = get_collision_interval_vec(
                p0, sp1, tp1, dt1, self.agent_geometry
            )
            if collide:
                lo += st1
                up += st1
            return (collide, lo, up)
        if n == 6:
            sv0, eid0, sv1, st1, eid1, tt1 = args
            if eid1 < 0:
                # Agent 1 is waiting: route through VEC with the
                # arguments reversed (agent 1's start = agent 0's
                # static point), then mirror the time frame.
                # Mirrors the C++ at problem.hpp:239-243 which does
                # ``getCollisionInterval(sv1, sv0, eid0, upper, lower)``
                # followed by ``lower = st1 - lower; upper = tt1 - upper``.
                p0 = self.coordinates[sv1]
                sp0 = self.coordinates[sv0]
                tv0 = self.edges[sv0][eid0]
                tp0 = self.coordinates[tv0]
                dt0 = self.get_edge_length(sv0, eid0)
                if dt0 < self.EPS:
                    return (False, 0.0, 0.0)
                collide, lo, up = get_collision_interval_vec(
                    p0, sp0, tp0, dt0, self.agent_geometry
                )
                if collide:
                    new_lo = st1 - up
                    new_up = tt1 - lo
                    return (True, new_lo, new_up)
                return (False, 0.0, 0.0)
            # Both moving: EEC kernel, shift by st1.
            sp0 = self.coordinates[sv0]
            tv0 = self.edges[sv0][eid0]
            tp0 = self.coordinates[tv0]
            sp1 = self.coordinates[sv1]
            tv1 = self.edges[sv1][eid1]
            tp1 = self.coordinates[tv1]
            dt0 = self.get_edge_length(sv0, eid0)
            dt1 = self.get_edge_length(sv1, eid1)
            if dt0 < self.EPS or dt1 < self.EPS:
                return (False, 0.0, 0.0)
            collide, lo, up = get_collision_interval_eec(
                sp0, tp0, dt0, sp1, tp1, dt1, self.agent_geometry
            )
            if collide:
                lo += st1
                up += st1
            return (collide, lo, up)
        raise TypeError(
            f"get_collision_interval: unsupported argument count {n}"
        )


# ---------------------------------------------------------------------------
# SpatialProblemInstance  (Stage 1b)
# ---------------------------------------------------------------------------
#
# Port of PSIPP-CTC_src/src/base/spatial.hpp, lines 11-26 (the data members)
# and Task struct (16-23). The class is templated on ``SpaceGeometry`` in the
# C++; in Python the only ``SpaceGeometry`` used is ``Space2D``, so the type
# parameters are hardcoded. The geometry-backed methods (``isInside``,
# ``getBound``, ``checkRoadspaceValidity``, ``checkPlanValidity``) are
# deferred to Stage 3 because they need the Space2D backend, the roadspace
# validity check, and the planning-event sweep.
#
# Trap: THREE different EPS values in three classes (PORTING.md §4):
#   Geometry2D::EPS              = 0       (geometry kernels)
#   ProblemInstance::EPS         = 1e-8    (this file, data model)
#   SpatialProblemInstance::EPS  = 1e-6    (clearance margin for isInside)
# ``SpatialProblemInstance::EPS`` is a clearance margin, not a comparison
# tolerance -- ``isInside(point)`` shrinks the free space by
# ``agent_geometry - EPS`` (``spatial.hpp:29-30``). Do not unify it with the
# other two.


class SpatialTask:
    """One agent's start and goal in continuous space.

    Mirrors ``SpatialProblemInstance::Task`` (``spatial.hpp:16-23``): a pair
    of ``Point`` values in 2D continuous coordinates, no vertex id. The
    roadspace-validity check (``spatial.hpp:70-85``) compares each task's
    points to the corresponding vertices in the generated roadmap, and
    Stage 5's ``checkPlanValidity`` (``spatial.hpp:96-155``) sweeps events
    in the same coordinate frame.
    """

    __slots__ = ("initial_point", "goal_point")

    def __init__(self, initial_point: Point, goal_point: Point) -> None:
        self.initial_point = initial_point
        self.goal_point = goal_point

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SpatialTask):
            return NotImplemented
        return (
            self.initial_point == other.initial_point
            and self.goal_point == other.goal_point
        )

    def __repr__(self) -> str:
        return (
            f"SpatialTask(initial={self.initial_point!r}, "
            f"goal={self.goal_point!r})"
        )


class SpatialProblemInstance:
    """Continuous-space problem: a free polygon + a list of (start, goal)
    tasks + a disc radius.

    Mirrors ``SpatialProblemInstance<Space2D>`` (``spatial.hpp:11-26``). The
    data members are:

    * ``outer`` -- the outer polygon ring as a list of ``Point``, with the
      first point repeated at the end (closed ring). Produced by
      ``loadTextFile`` (``input_spatial.hpp:31``) or by the right-hand
      boundary tracer in ``loadGripMap`` (``input_spatial.hpp:147``).
    * ``inners`` -- list of closed rings, one per hole. Same convention.
      Order is the trace order, not geometric containment
      (``input_spatial.hpp:143-158``). Empty if the file/map has no holes.
    * ``agent_geometry`` -- disc radius. The conflict threshold is
      ``2 * radius`` (strict ``<``).
    * ``tasks`` -- list of ``SpatialTask``. For a polygon-format file this
      comes from the file; for a .map + .scen pair it is built by
      ``loadScenario`` (``input_spatial.hpp:184``), which is **deferred to
      Stage 3** because it calls ``isInside()`` and therefore needs the
      Space2D backend.

    Methods that depend on the Space2D backend -- ``isInside`` (point and
    point-pair), ``getBound``, ``checkRoadspaceValidity``,
    ``checkPlanValidity`` -- raise ``NotImplementedError`` here and are
    filled in by Stage 3.
    """

    #: Clearance margin used by ``isInside`` (``spatial.hpp:27``). NOT a
    #: comparison tolerance; the geometry kernels' ``EPS = 0`` and the data
    #: model's ``EPS = 1e-8`` are different constants in different classes.
    EPS: float = 1e-6

    def __init__(
        self,
        outer: List[Point],
        inners: List[List[Point]],
        agent_geometry: float,
        tasks: List[SpatialTask],
    ) -> None:
        self.outer: List[Point] = outer
        self.inners: List[List[Point]] = inners
        self.agent_geometry: float = agent_geometry
        self.tasks: List[SpatialTask] = tasks
        # Lazy-built shapely Polygon (the Space2D backend). Built on
        # first access via ``_space_polygon``; cached for the lifetime
        # of the instance. Avoids paying the cost when a
        # SpatialProblemInstance is created and immediately consumed in
        # a way that does not need the backend.
        self._space_polygon_cache = None  # type: ignore[var-annotated]

    # -- Geometry backend (Stage 3) --------------------------------------

    def _space_polygon(self):
        """Lazily build the shapely Polygon backing the Space2D methods.

        Mirrors the C++ ``SpaceGeometry::Space`` type. Both ``outer`` and
        each ``inners[i]`` are closed rings (first == last) per the
        loaders in ``io/spatial.py``. shapely closes the ring
        automatically so the loader's closing duplicate is dropped before
        construction.
        """
        if self._space_polygon_cache is None:
            from psipp_ctc.roadmap.space_2d import _make_polygon
            self._space_polygon_cache = _make_polygon(self.outer, self.inners)
        return self._space_polygon_cache

    def is_inside(self, point: Point) -> bool:
        """True iff ``point`` is in the free polygon with ``agent_geometry``
        clearance.

        Mirrors ``SpatialProblemInstance<Space2D>::isInside(Point)``
        (``spatial.hpp:29-31``) which delegates to
        ``Space2D::checkVertexInside(space, point, agent_geometry - EPS)``.
        The EPS is a *clearance margin*: a disc of radius
        ``agent_geometry`` centred at ``point`` must not cross any
        boundary.
        """
        from psipp_ctc.roadmap.space_2d import check_vertex_inside
        return check_vertex_inside(
            self._space_polygon(), point, self.agent_geometry - self.EPS
        )

    def is_edge_inside(self, p0: Point, p1: Point) -> bool:
        """True iff a disc of radius ``agent_geometry`` can travel along the
        segment from ``p0`` to ``p1`` without crossing the boundary.

        Mirrors ``spatial.hpp:33-37``. Used by the roadmap generators to
        decide whether a candidate edge may be added: every edge of the
        final roadmap must pass this test, otherwise the planner could
        route an agent through a wall.
        """
        from psipp_ctc.roadmap.space_2d import check_edge_inside
        return check_edge_inside(
            self._space_polygon(), p0, p1, self.agent_geometry - self.EPS
        )

    def get_bound(self, axis: int) -> Tuple[float, float]:
        """Bounding-box of the outer ring along ``axis`` (0 = x, 1 = y).

        Mirrors ``spatial.hpp:39-41`` -> ``Space2D::getBound``. Used by
        the Grid generator to size the cell lattice across the BBox;
        cells that fall in holes are then dropped via ``is_inside``.
        """
        from psipp_ctc.roadmap.space_2d import get_bound
        return get_bound(self._space_polygon(), axis)

    def check_roadspace_validity(
        self, problem_instance: "ProblemInstanceWithGeometry"
    ) -> bool:
        """Validate a generated roadmap against the spatial problem.

        Mirrors ``spatial.hpp:43-91``. Three checks:

        1. Every vertex of the roadmap lies in the free polygon (with
           ``agent_geometry - EPS`` clearance). A single failure prints
           to stderr and returns False.
        2. Every edge of the roadmap lies in the free polygon (the
           disc can travel from source to target without crossing the
           boundary).
        3. Each task's start/goal coordinate matches the corresponding
           vertex in the roadmap (within EPS), and the agent radii
           agree.

        Returns True iff all three pass. The C++ also prints to stderr
        on failure; the port raises ``ValueError`` instead so the test
        harness can see which check failed.
        """
        from psipp_ctc.roadmap.space_2d import check_edge_inside, check_vertex_inside
        r = self.agent_geometry - self.EPS
        n = problem_instance.get_number_of_vertices()
        for vertex in range(n):
            if not check_vertex_inside(
                self._space_polygon(), problem_instance.get_point(vertex), r
            ):
                raise ValueError(
                    f"roadspace validity: vertex {vertex} at "
                    f"{problem_instance.get_point(vertex)} is outside the map"
                )
        for vertex in range(n):
            for target in problem_instance.edges[vertex]:
                if not check_edge_inside(
                    self._space_polygon(),
                    problem_instance.get_point(vertex),
                    problem_instance.get_point(target),
                    r,
                ):
                    raise ValueError(
                        f"roadspace validity: edge {vertex} -> {target} "
                        f"is outside the map"
                    )
        for i, task in enumerate(self.tasks):
            init_v = problem_instance.tasks[i].initial_vertex
            goal_v = problem_instance.tasks[i].goal_vertex
            if math.hypot(
                task.initial_point.x - problem_instance.get_point(init_v).x,
                task.initial_point.y - problem_instance.get_point(init_v).y,
            ) >= self.EPS:
                raise ValueError(
                    f"roadspace validity: task {i} initial point does not "
                    f"match vertex {init_v}"
                )
            if math.hypot(
                task.goal_point.x - problem_instance.get_point(goal_v).x,
                task.goal_point.y - problem_instance.get_point(goal_v).y,
            ) >= self.EPS:
                raise ValueError(
                    f"roadspace validity: task {i} goal point does not "
                    f"match vertex {goal_v}"
                )
        if abs(problem_instance.agent_geometry - self.agent_geometry) >= self.EPS:
            raise ValueError(
                f"roadspace validity: agent_geometry mismatch "
                f"({problem_instance.agent_geometry} vs {self.agent_geometry})"
            )
        return True
