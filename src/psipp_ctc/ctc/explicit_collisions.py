"""Offline CTC annotation.

The planner's online query budget is a few microseconds per call. Running
the VEC and EEC kernels (linear / quadratic inequality chains) inside the
inner loop is the wrong place. CTC materialises the conflict set once at
construction time so the planner's online queries are pure map lookups.

What the constructor does
-------------------------
1. **Flatten the adjacency list into GLOBAL edge indices.** The planner
   names an edge by ``(source, out-list-index)``; the annotation names it
   by a single global integer. ``edge_ids[v][k]`` is the global id for v's
   k-th out-edge; numbering is sequential per vertex in declaration
   order. ``reverse_edge_ids[target]`` lists every global id whose target
   is ``target``. ``edge_data[global_id] = (source, target, edge_id)``.

2. **Broad phase for VV and VEC candidates.** A single ``Buckets2D`` is
   built with cell size ``2r - EPS`` (the same slack the geometry
   kernels use via ``self.EPS = 1e-8``). Its ``calculate_all_pairs`` is
   the VV candidate set; ``calculate_collision_vertices`` per edge
   returns the VEC candidates (vertices within the conflict radius
   perpendicular of the open segment).

3. **Broad phase for EE candidates.** ``BentleyOttoman`` sweeps the full
   set of segments in random-rotated space, returning every pair of
   segments that geometrically cross. The sweep is unseeded; the
   Python ``BentleyOttoman`` uses the same unseeded ``random.SystemRandom``.

4. **Exact kernels per surviving candidate.** The VEC kernel
   (``get_collision_interval_vec``) and the EEC kernel
   (``get_collision_interval_eec``) are run once per surviving pair and
   the interval is stored.

5. **Build the four ``collisions_lists_*`` tables** for the
   ``get_all_*`` accessors the planner reads while constructing the
   blocked-window structure for the next agent.

What the query overrides do
---------------------------
Five methods are replaced with table lookups. The geometry kernels are
NO LONGER CALLED in the online path:

* ``collision_check_vv(v0, v1)`` -- set membership of ``(min, max)`` in
  ``collision_pairs_v2v``.
* ``get_collision_interval(v0, sv1, eid1)`` (5-arg VE) -- direct
  ``(v0, edge_ids[sv1][eid1])`` lookup in ``collision_pairs_v2e``.
* ``collision_check_ve(v0, st0, tt0, sv1, st1, eid1)`` -- VEC time
  condition: ``max(st0, st1+lo) + EPS < min(tt0, st1+up)``.
* ``get_collision_interval(sv0, eid0, sv1, eid1)`` (6-arg EE) -- lookup
  in ``collision_pairs_e2e`` keyed by ``(min_global, max_global)``, with
  the sign-swap convention when the caller passes the reversed pair.
* ``collision_check_ee(sv0, st0, eid0, sv1, st1, eid1)`` -- EEC time
  condition: ``lo + EPS < st0 - st1 < up - EPS``.

Sign convention trap
--------------------
The 6-arg ``get_collision_interval`` is the EEC kernel with the
**reverse-time** sign convention. The storage key is always
``(min_global_id, max_global_id)`` with ``(lo, up)`` in the natural
time direction (relative to agent 1). When the caller passes the
reversed pair, the time direction is mirrored, so the returned interval
is ``(-up, -lo)``. Do NOT "fix" the signs (cpp-to-python skill rule 9).

Only the ``ve`` accessor list carries the reverse-time convention:
``get_all_ve_collisions(v)`` returns ``(src_of_e, edge_id_of_e, -upper,
-lower)``. The ``get_all_ee_collisions(v, eid)`` accessor returns the
NATURAL-time interval ``(src_of_e, edge_id_of_e, lower, upper)`` -- the
C++ ``ProblemInstance::getAllEECollisions`` (``problem.hpp:325-339``)
calls the 5-arg ``getCollisionInterval`` which returns natural time and
appends it directly without negation. The planner adds ``t_step`` to
both endpoints as-is. ``get_all_ee_collisions`` lists BOTH directions
for a non-self pair (self entries appear once).
"""

from __future__ import annotations

import math
from typing import Dict, List, Set, Tuple

from psipp_ctc.geometry.collision_report import BentleyOttoman, Buckets2D
from psipp_ctc.geometry.geometry_2d import (
    get_collision_interval_eec,
    get_collision_interval_vec,
)
from psipp_ctc.problem import Plan, ProblemInstanceWithGeometry


# Type aliases for the three table value shapes.
_Interval = Tuple[float, float]
_VEListEntry = Tuple[int, int, float, float]  # (source_of_e, edge_id_of_e, -upper, -lower)
_EVListEntry = Tuple[int, float, float]        # (vertex, lower, upper)


class ProblemInstanceWithExplicitCollisions(ProblemInstanceWithGeometry):
    """Roadmap + tasks + 2D coordinates + agent radius + materialised
    VV / VEC / EEC tables.

    Constructed from any ``ProblemInstanceWithGeometry``; the constructor
    copies the base fields, builds the global edge indices, runs the
    broad phases, and stores one exact interval per surviving VV / VEC
    / EEC pair.

    Online queries are table lookups, not geometry calls. The 8-arg
    dispatcher ``_collision_check_dispatch`` (inherited from
    ``ProblemInstance``) routes to the three kernels; those kernels
    (``collision_check_vv``, ``collision_check_ve``,
    ``collision_check_ee``) are overridden here to use the tables.
    """

    def __init__(self, source: ProblemInstanceWithGeometry) -> None:
        super().__init__()
        # Copy base fields. The C++ does an element-by-element copy
        # of ``edges``, ``coordinates``, ``tasks``, ``agent_geometry``
        # (lines 122-125). We do the same with shallow copies; the
        # ``Point`` and ``AgentTask`` values are immutable
        # ``NamedTuple``s, so aliasing is safe.
        self.edges = [list(e) for e in source.edges]
        self.coordinates = list(source.coordinates)
        self.tasks = list(source.tasks)
        self.agent_geometry = source.agent_geometry

        n_vertices = self.get_number_of_vertices()

        # -- Global edge indices (lines 128-141 of the C++) -----------
        # ``edge_ids[v][k]`` is the global id for v's k-th out-edge;
        # numbering is sequential per vertex in declaration order.
        # ``reverse_edge_ids[target]`` is the inverse mapping. The
        # global id is the *insertion* order across all out-edges.
        self.edge_ids: List[List[int]] = [[] for _ in range(n_vertices)]
        self.reverse_edge_ids: List[List[int]] = [[] for _ in range(n_vertices)]
        # ``edge_data[global_id] = (source, target, edge_id)`` -- the
        # inverse of ``edge_ids`` plus the local out-list index, for
        # one-step global-id -> (source, target) translation.
        self.edge_data: List[Tuple[int, int, int]] = []
        n_edges = 0
        for v in range(n_vertices):
            outdeg = self.get_out_degree(v)
            self.edge_ids[v] = list(range(n_edges, n_edges + outdeg))
            n_edges += outdeg
            for edge_id in range(outdeg):
                target = self.get_target_vertex(v, edge_id)
                self.reverse_edge_ids[target].append(len(self.edge_data))
                self.edge_data.append((v, target, edge_id))

        # -- Broad phase (Buckets2D) for VV and VEC (lines 143-191) ----
        # Cell size = 2r - EPS. The set ``collision_pairs_v2v`` then
        # contains every pair with ``distance < 2r - EPS``. The same
        # predicate is the geometry kernel's
        # ``collision_check_static(p0, p1, r, self.EPS)`` threshold
        # (problem_with_geometry.hpp:61), so the geometric and
        # explicit instances agree on every VV pair -- this is what
        # the differential test verifies.
        buckets = Buckets2D(
            2.0 * self.agent_geometry - self.EPS, self.coordinates
        )
        self.collision_pairs_v2v: Set[Tuple[int, int]] = buckets.calculate_all_pairs()

        # -- Annotate VV pairs with VE and EE candidates --------------
        # Storage maps: ``collision_pairs_v2e`` is keyed by
        # ``(v, edge_global_id)``; ``collision_pairs_e2e`` is keyed by
        # ``(min_global, max_global)`` (canonicalised in
        # ``_add_collision_ee``). Both use ``dict`` because the C++
        # ``std::map`` ordering is not used by any of the lookups --
        # only insertion and equality matter.
        self.collision_pairs_v2e: Dict[Tuple[int, int], _Interval] = {}
        self.collision_pairs_e2e: Dict[Tuple[int, int], _Interval] = {}

        # For every VV pair (v0, v1), add VEC for v0 with every edge
        # of v1 and v1 with every edge of v0 (both out-going and
        # in-coming), then EEC for the four cross-products.
        for v0, v1 in self.collision_pairs_v2v:
            for eid in self.edge_ids[v1]:
                self._add_collision_ve(v0, eid)
            for eid in self.reverse_edge_ids[v1]:
                self._add_collision_ve(v0, eid)
            for eid in self.edge_ids[v0]:
                self._add_collision_ve(v1, eid)
            for eid in self.reverse_edge_ids[v0]:
                self._add_collision_ve(v1, eid)
            for eid0 in self.edge_ids[v0]:
                for eid1 in self.edge_ids[v1]:
                    self._add_collision_ee(eid0, eid1)
                for eid1 in self.reverse_edge_ids[v1]:
                    self._add_collision_ee(eid0, eid1)
            for eid0 in self.reverse_edge_ids[v0]:
                for eid1 in self.edge_ids[v1]:
                    self._add_collision_ee(eid0, eid1)
                for eid1 in self.reverse_edge_ids[v1]:
                    self._add_collision_ee(eid0, eid1)

        # Per-edge candidate vertices (lines 177-191). For every edge,
        # find any vertex within the perpendicular-extended rectangle
        # (radius ``2r - EPS``); for each, add a VEC pair and the two
        # EE cross products with that vertex's edges.
        for eid in range(len(self.edge_data)):
            source_v, target_v, _ = self.edge_data[eid]
            p_src = self.coordinates[source_v]
            p_tgt = self.coordinates[target_v]
            for v in buckets.calculate_collision_vertices(p_src, p_tgt):
                self._add_collision_ve(v, eid)
                for other_eid in self.edge_ids[v]:
                    self._add_collision_ee(eid, other_eid)
                for other_eid in self.reverse_edge_ids[v]:
                    self._add_collision_ee(eid, other_eid)

        # -- Segment crossings (lines 192-203) -------------------------
        # ``BentleyOttoman`` is unseeded in the C++. The pair SET
        # produced is deterministic; only the iteration order in which
        # they are reported is not. The Python
        # ``BentleyOttoman.calculate_all_crosses`` uses the same
        # unseeded ``random.SystemRandom``.
        segments = [
            (self.coordinates[s], self.coordinates[t])
            for (s, t, _) in self.edge_data
        ]
        for eid0, eid1 in BentleyOttoman(segments).calculate_all_crosses():
            self._add_collision_ee(eid0, eid1)

        # -- Build the four accessor lists (line 205) -----------------
        self._make_lists()

    # ------------------------------------------------------------------
    # Internal: VEC and EEC materialisation
    # ------------------------------------------------------------------

    def _add_collision_ve(self, vertex: int, eid: int) -> None:
        """Add a VEC conflict ``(v, edge)`` if the exact test agrees.

        Idempotent on ``collision_pairs_v2e``: a second call with
        the same ``(v, eid)`` is a no-op. Skips degenerate
        zero-length edges (self-loops), matching the C++ ``if (length
        == 0.0) return;``. Asserts the VEC kernel reports a
        collision, because the broad phase is supposed to have
        filtered to candidates only.
        """
        key = (vertex, eid)
        if key in self.collision_pairs_v2e:
            return
        source_v, target_v, _ = self.edge_data[eid]
        p_v = self.coordinates[vertex]
        p_src = self.coordinates[source_v]
        p_tgt = self.coordinates[target_v]
        length = math.hypot(p_tgt.x - p_src.x, p_tgt.y - p_src.y)
        if length == 0.0:
            return
        collide, lo, up = get_collision_interval_vec(
            p_v, p_src, p_tgt, length, self.agent_geometry
        )
        # ``assert(collision)`` at C++ line 80. We mirror it as a
        # runtime assertion: a future port regression that lets a
        # non-conflict slip past the broad phase fails loudly.
        assert collide, (
            f"VEC broad-phase candidate {key} produced no interval; "
            "broad phase or VEC kernel drift"
        )
        self.collision_pairs_v2e[key] = (lo, up)

    def _add_collision_ee(self, a: int, b: int) -> None:
        """Add an EEC conflict ``(edge_a, edge_b)`` if the exact test agrees.

        The storage key is canonicalised: ``a <= b`` always. The
        lookup side (``get_collision_interval`` 6-arg EE) handles the
        reverse-time sign swap when the caller asks for the
        reversed pair. Skips if either edge has zero length.
        """
        if a > b:
            a, b = b, a
        key = (a, b)
        if key in self.collision_pairs_e2e:
            return
        s0, t0, _ = self.edge_data[a]
        s1, t1, _ = self.edge_data[b]
        p_s0 = self.coordinates[s0]
        p_t0 = self.coordinates[t0]
        p_s1 = self.coordinates[s1]
        p_t1 = self.coordinates[t1]
        len0 = math.hypot(p_t0.x - p_s0.x, p_t0.y - p_s0.y)
        len1 = math.hypot(p_t1.x - p_s1.x, p_t1.y - p_s1.y)
        if len0 == 0.0 or len1 == 0.0:
            return
        collide, lo, up = get_collision_interval_eec(
            p_s0, p_t0, len0, p_s1, p_t1, len1, self.agent_geometry
        )
        assert collide, (
            f"EEC broad-phase candidate {key} produced no interval; "
            "broad phase or EEC kernel drift"
        )
        self.collision_pairs_e2e[key] = (lo, up)

    def _make_lists(self) -> None:
        """Build the four ``collisions_lists_*`` tables.

        These are the read-side lookup tables for ``get_all_*``; the
        planner reads them per-vertex / per-edge when constructing
        the blocked-window structure for the next agent.

        * ``v2v[v]`` -- list of vertices v collides with. Each
          ``(v0, v1)`` pair is appended in BOTH directions
          (asymmetric: ``v0 < v1`` to ``v2v[v0]``, ``v1 < v0`` to
          ``v2v[v1]``). The C++ ``if (v0 != v1)`` branch avoids
          double-appending self pairs, which can only arise if
          ``(v, v)`` is in the set -- the broad phase cannot
          produce that, so this is defensive.
        * ``v2e[v]`` -- list of ``(src_of_e, edge_id, -upper, -lower)``
          for every ``(v, e)`` entry. The negation is the
          reverse-time convention: relative entry times face agent
          1's *arrival*, not its start.
        * ``e2v[e]`` -- list of ``(v, lower, upper)`` for every
          ``(v, e)`` entry. Time direction is the natural one.
        * ``e2e[e0]`` -- list of ``(src_of_other, edge_id_of_other,
          -upper, -lower)`` AND (if non-self) the reverse
          ``(src_of_self, edge_id_of_self, lower, upper)``.

        The Python ``set`` iteration is unordered (C++ ``std::set``
        is sorted); the LIST contents are the same, only the
        order differs. Tests that compare lists should sort first.
        """
        n_v = self.get_number_of_vertices()
        n_e = len(self.edge_data)
        self.collisions_lists_v2v: List[List[int]] = [[] for _ in range(n_v)]
        self.collisions_lists_v2e: List[List[_VEListEntry]] = (
            [[] for _ in range(n_v)]
        )
        self.collisions_lists_e2v: List[List[_EVListEntry]] = (
            [[] for _ in range(n_e)]
        )
        self.collisions_lists_e2e: List[List[_VEListEntry]] = (
            [[] for _ in range(n_e)]
        )
        # v2v -- list per vertex of the OTHER vertices it collides with.
        for v0, v1 in self.collision_pairs_v2v:
            self.collisions_lists_v2v[v0].append(v1)
            if v0 != v1:
                self.collisions_lists_v2v[v1].append(v0)
        # v2e + e2v -- the (v, e) pairs.
        for (v, eid), (lo, up) in self.collision_pairs_v2e.items():
            src_of_e, _, edge_id_of_e = self.edge_data[eid]
            self.collisions_lists_v2e[v].append(
                (src_of_e, edge_id_of_e, -up, -lo)
            )
            self.collisions_lists_e2v[eid].append((v, lo, up))
        # e2e -- the (e0, e1) pairs, both directions for non-self.
        # The C++ ``getAllEECollisions`` returns ``(vertex_0, edge_id_0,
        # lower, upper)`` in NATURAL time (no negation); the planner
        # adds ``t_step`` to lower/upper directly. We mirror that: the
        # canonical entry at eid0 carries ``(lo, up)`` and the
        # reverse entry at eid1 also carries ``(lo, up)`` (no swap
        # is needed -- eid1 is the other direction's vertex, and
        # looking it up via ``getCollisionInterval(eid0, eid1)``
        # already returned the natural-time interval).
        for (eid0, eid1), (lo, up) in self.collision_pairs_e2e.items():
            src1, _, edge_id1 = self.edge_data[eid1]
            self.collisions_lists_e2e[eid0].append((src1, edge_id1, lo, up))
            if eid0 != eid1:
                src0, _, edge_id0 = self.edge_data[eid0]
                self.collisions_lists_e2e[eid1].append((src0, edge_id0, lo, up))

    # ------------------------------------------------------------------
    # Query overrides -- table lookups, no geometry
    # ------------------------------------------------------------------

    def collision_check_vv(self, v0: int, v1: int) -> bool:
        """VV: set membership of the canonicalised pair.

        The set is stored with ``(min, max)`` ordering, so the caller can pass
        either order. Replaces the geometry subclass's
        ``distance(p0, p1) < 2r - EPS`` test with a constant-time
        dictionary lookup. Both implementations use the same
        predicate, so the differential test must agree.
        """
        return (min(v0, v1), max(v0, v1)) in self.collision_pairs_v2v

    def get_collision_interval(  # type: ignore[override]
        self, *args
    ):
        """Four overloads: two kernel forms (table lookup), two time-mirror
        forms (kernel + frame shift).

        3-arg ``(v0, sv1, eid1) -> (collide, lo, up)``
            VEC kernel form. Pure map lookup of
            ``(v0, edge_ids[sv1][eid1])`` in ``collision_pairs_v2e``.
            ``(lo, up)`` is *relative to agent 1's start* (st1 = 0).

        4-arg ``(sv0, eid0, sv1, eid1) -> (collide, lo, up)``
            EEC kernel form. Storage is keyed by
            ``(min_global, max_global)`` with ``(lo, up)`` in
            agent-1-natural direction. When the caller passes the
            reversed pair, the time axis is mirrored, returning
            ``(-up, -lo)`` -- the **reverse-time** sign convention.
            Do NOT "fix" the signs (cpp-to-python skill rule 9).

        5-arg ``(v0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            VEC time-mirror form. If ``eid1 < 0``, falls back to
            the VV check. Otherwise looks up the 3-arg form and
            shifts ``(lo, up)`` by ``+st1``.

        6-arg ``(sv0, eid0, sv1, st1, eid1, tt1) -> (collide, lo, up)``
            EEC time-mirror form. If ``eid1 < 0``, looks up the
            4-arg form with the args reversed and mirrors the
            time axis. Otherwise looks up the 4-arg form and
            shifts by ``+st1``.
        """
        n = len(args)
        if n == 3:
            v0, sv1, eid1 = args
            entry = self.collision_pairs_v2e.get(
                (v0, self.edge_ids[sv1][eid1])
            )
            if entry is None:
                return (False, 0.0, 0.0)
            lo, up = entry
            return (True, lo, up)
        if n == 4:
            sv0, eid0, sv1, eid1 = args
            edge_0 = self.edge_ids[sv0][eid0]
            edge_1 = self.edge_ids[sv1][eid1]
            if edge_0 <= edge_1:
                entry = self.collision_pairs_e2e.get((edge_0, edge_1))
                if entry is None:
                    return (False, 0.0, 0.0)
                lo, up = entry
                return (True, lo, up)
            # Sign-swap convention: storage is in agent-1-natural
            # direction; the caller asked for agent-0-natural
            # direction, so mirror the time axis. The C++ writes
            # ``lower = -it->second.second; upper = -it->first``;
            # with Python's (lo, up) tuple that is ``(lo, up) =
            # (-up, -lo)``.
            entry = self.collision_pairs_e2e.get((edge_1, edge_0))
            if entry is None:
                return (False, 0.0, 0.0)
            lo, up = entry
            return (True, -up, -lo)
        if n == 5:
            # VEC time-mirror: 5-arg form. Look up the 3-arg form
            # and shift by st1.
            v0, sv1, st1, eid1, tt1 = args
            if eid1 < 0:
                collide = self.collision_check_vv(v0, sv1)
                return (collide, st1, tt1) if collide else (False, 0.0, 0.0)
            collide, lo, up = self.get_collision_interval(v0, sv1, eid1)
            if collide:
                return (True, lo + st1, up + st1)
            return (False, 0.0, 0.0)
        if n == 6:
            # EEC time-mirror: 6-arg form. If eid1 < 0, route through
            # the 4-arg form with args swapped, then mirror.
            sv0, eid0, sv1, st1, eid1, tt1 = args
            if eid1 < 0:
                collide, lo, up = self.get_collision_interval(
                    sv1, sv0, eid0
                )
                if collide:
                    new_lo = st1 - up
                    new_up = tt1 - lo
                    return (True, new_lo, new_up)
                return (False, 0.0, 0.0)
            # Both moving: 4-arg form + shift by st1.
            collide, lo, up = self.get_collision_interval(
                sv0, eid0, sv1, eid1
            )
            if collide:
                return (True, lo + st1, up + st1)
            return (False, 0.0, 0.0)
        raise TypeError(
            f"get_collision_interval: unsupported argument count {n}"
        )

    def collision_check_ve(
        self, v0: int, st0: float, tt0: float,
        sv1: int, st1: float, eid1: int,
    ) -> bool:
        """VE time check: the VEC interval overlaps agent 0's window.

        The condition is the VEC time overlap:

            max(st0, st1 + lo) + EPS < min(tt0, st1 + up)

        The VEC kernel returns ``(lo, up)`` *relative to agent 1's
        start* (``st1 = 0``). The dispatcher here adds ``st1`` to
        lift it into the caller's frame, so the condition becomes a
        window-overlap test on the absolute time axis.
        """
        collide, lo, up = self.get_collision_interval(v0, sv1, eid1)
        if not collide:
            return False
        return max(st0, st1 + lo) + self.EPS < min(tt0, st1 + up)

    def collision_check_ee(
        self, sv0: int, st0: float, eid0: int,
        sv1: int, st1: float, eid1: int,
    ) -> bool:
        """EE time check: ``st0 - st1`` is in the EEC open interval.

        The condition is the EEC time overlap:

            lo + EPS < st0 - st1 < up - EPS

        where ``(lo, up)`` is the EEC interval from
        ``get_collision_interval`` (with the sign-swap convention
        already applied). The interval is in the *relative* time
        frame ``s = st0 - st1`` -- the start time offset of agent
        0 against agent 1's clock.
        """
        collide, lo, up = self.get_collision_interval(sv0, eid0, sv1, eid1)
        if not collide:
            return False
        return lo + self.EPS < st0 - st1 < up - self.EPS

    # ------------------------------------------------------------------
    # get_all_* accessors -- read the precomputed lists
    # ------------------------------------------------------------------

    def get_all_vv_collisions(self, v: int) -> List[int]:
        """List of vertices v collides with. Empty if v has none."""
        return self.collisions_lists_v2v[v]

    def get_all_ve_collisions(self, v: int) -> List[_VEListEntry]:
        """List of ``(source_of_e, edge_id, -upper, -lower)`` for every
        ``(v, e)`` VEC entry.

        The negation is the reverse-time convention (entry times
        relative to agent 1's arrival, not its start).
        """
        return self.collisions_lists_v2e[v]

    def get_all_ev_collisions(
        self, v: int, edge_id: int
    ) -> List[_EVListEntry]:
        """List of ``(vertex, lower, upper)`` for every ``(v, e)`` VEC
        entry, addressed by the global id of the (v, edge_id) edge.
        """
        return self.collisions_lists_e2v[self.edge_ids[v][edge_id]]

    def get_all_ee_collisions(
        self, v: int, edge_id: int
    ) -> List[_VEListEntry]:
        """List of ``(other_src, other_edge_id, lower, upper)`` for
        every EEC pair involving this edge, in NATURAL time (no
        negation). Both directions are listed for non-self pairs;
        self entries appear once. The planner adds ``t_step`` to
        both endpoints to get the absolute blocked window.
        """
        return self.collisions_lists_e2e[self.edge_ids[v][edge_id]]

    # ------------------------------------------------------------------
    # check_plan_validity
    # ------------------------------------------------------------------

    def check_plan_validity(
        self, plan: Plan
    ) -> Tuple[float, float]:
        """Verify a plan is collision-free and compute
        ``(makespan, sum_of_costs)``.

        A Cartesian-product gate over every (agent, step) pair,
        dispatching each pair to the 8-arg ``_collision_check_dispatch``
        (VV / VEC / EEC) and to ``get_collision_interval`` for
        explicit-collisions instances. The C++ has an event-sweep
        variant on the geometry subclass; the Python port uses the
        base form for now -- the O(N^2 x steps^2) loop is correct and
        the table-lookup kernels keep the inner loop fast enough for
        parity.

        On a valid plan: returns ``(makespan, sum_of_costs)``
        where ``makespan`` is the max per-agent cost and
        ``sum_of_costs`` is the sum.

        On an invalid plan: raises ``RuntimeError`` with one of:

        * ``"agent starts before reach start vertex"`` -- a
          move's ``start_time`` is before the previous move's
          end time minus EPS.
        * ``"edge id is invalid"`` -- the move's ``edge_id``
          is out of range for the agent's current vertex.
        * ``"agent does not reach goal"`` -- the agent's
          final position is not its goal.
        * ``"collision occurs"`` -- the 8-arg
          ``_collision_check_dispatch`` returns True for some
          pair of (step0, step1).
        """
        n = len(plan)
        if n != len(self.tasks):
            raise ValueError(
                f"plan has {n} agents but problem has {len(self.tasks)}"
            )

        # Reconstruct the C++'s ``routes`` and ``time_stamps``
        # data structures from the plan. The C++ inserts a
        # wait (vertex, -1) with the current time whenever
        # ``start_time > current_time + EPS``; the resulting
        # ``routes`` and ``time_stamps`` have
        # ``time_stamps[i].size() == routes[i].size() + 1``
        # (the extra trailing entry is the goal arrival).
        routes: List[List[Tuple[int, int]]] = [[] for _ in range(n)]
        time_stamps: List[List[float]] = [[] for _ in range(n)]
        # ``agents`` is the task list. The C++ uses
        # ``tasks[i % tasks.size()]`` (line 352) which means
        # the task list is treated as a circular buffer if
        # the plan has more agents than tasks. We replicate
        # that behaviour exactly.
        for i in range(n):
            task = self.tasks[i % len(self.tasks)]
            current_time = 0.0
            current_vertex = task.initial_vertex
            for move in plan[i]:
                if move.start_time < current_time - self.EPS:
                    raise RuntimeError(
                        "agent starts before reach start vertex"
                    )
                if move.edge_id < 0:
                    # Wait: stays at current_vertex; end time
                    # is ``start_time`` (zero-length leg).
                    if move.start_time > current_time + self.EPS:
                        time_stamps[i].append(current_time)
                        routes[i].append((current_vertex, -1))
                    time_stamps[i].append(move.start_time)
                    routes[i].append((current_vertex, move.edge_id))
                    current_time = move.start_time
                else:
                    out_deg = self.get_out_degree(current_vertex)
                    if move.edge_id >= out_deg:
                        raise RuntimeError("edge id is invalid")
                    target = self.get_target_vertex(current_vertex, move.edge_id)
                    length = self.get_edge_length(current_vertex, move.edge_id)
                    end_time = move.start_time + length
                    if current_time + self.EPS < move.start_time:
                        # Implicit wait between this edge and
                        # the previous.
                        time_stamps[i].append(current_time)
                        routes[i].append((current_vertex, -1))
                    time_stamps[i].append(move.start_time)
                    routes[i].append((current_vertex, move.edge_id))
                    current_vertex = target
                    current_time = end_time
            if current_vertex != task.goal_vertex:
                raise RuntimeError("agent does not reach goal")
            time_stamps[i].append(current_time)

        # Collision check: O(n^2 * steps^2) Cartesian product
        # over every (agent_i, step_0) x (agent_j, step_1)
        # pair. The 8-arg ``_collision_check_dispatch`` is
        # inherited from ``ProblemInstance`` and routes to
        # the three concrete kernels (``collision_check_vv``,
        # ``collision_check_ve``, ``collision_check_ee``) --
        # all of which the explicit-collisions class has
        # overridden with table lookups.
        for i in range(n):
            for step0 in range(len(routes[i])):
                for j in range(i + 1, n):
                    for step1 in range(len(routes[j])):
                        collides = self._collision_check_dispatch(
                            routes[i][step0][0],
                            time_stamps[i][step0],
                            routes[i][step0][1],
                            time_stamps[i][step0 + 1],
                            routes[j][step1][0],
                            time_stamps[j][step1],
                            routes[j][step1][1],
                            time_stamps[j][step1 + 1],
                        )
                        if collides:
                            raise RuntimeError(
                                f"collision occurs: agent {i} step {step0} "
                                f"(v={routes[i][step0][0]}, e={routes[i][step0][1]}, "
                                f"t=[{time_stamps[i][step0]}, {time_stamps[i][step0+1]}]) "
                                f"vs agent {j} step {step1} "
                                f"(v={routes[j][step1][0]}, e={routes[j][step1][1]}, "
                                f"t=[{time_stamps[j][step1]}, {time_stamps[j][step1+1]}])"
                            )

        # Compute makespan and sum_of_costs. The C++ uses the
        # LAST entry of ``time_stamps[i]`` (the goal arrival).
        makespan = 0.0
        sum_of_costs = 0.0
        for i in range(n):
            cost = time_stamps[i][-1]
            makespan = max(makespan, cost)
            sum_of_costs += cost
        return (makespan, sum_of_costs)
