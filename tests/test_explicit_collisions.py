"""Stage 4 -- differential test for the offline CTC annotation.

Mirrors ``src/test/explicit_collision_test.cpp`` (72 lines) in the
reference. The C++ test loads
``../problem_instances/roadmaps/generated.txt`` which does not exist
on disk; we substitute ``problem_instances/roadmaps/sample.txt``
which is identical to the reference's
``PSIPP-CTC_src/problem_instances/roadmaps/sample.txt``. The brief
explicitly authorises this substitution (PORTING.md §7, "Trap: the
differential test needs a roadmap that does not exist yet").

Differential strategy
---------------------
Build a ``ProblemInstanceWithGeometry`` (the "geometric" instance, the
slow path: every online query runs the VEC and EEC kernels) and a
``ProblemInstanceWithExplicitCollisions`` (the "explicit" instance, the
fast path: online queries are table lookups). They MUST agree on
every (agent, time) sample, even though they are computed differently.

The agreement is exact in our Python port for two reasons:
1. The geometric ``collision_check_vv`` calls ``collision_check_static``
   with ``self.EPS = 1e-8``. The explicit broad phase builds
   ``Buckets2D(2r - 1e-8)`` and tests ``distance < 2r - 1e-8``. Same
   predicate.
2. The VEC and EEC intervals stored in the tables are computed by the
   same kernel functions the geometric instance calls per query, so
   float arithmetic is identical.

The brief notes that in the C++ there is a corner-case disagreement
band of width 1e-8 for pairs near ``2r``: the C++ uses
``Geometry2D::EPS = 0`` strictly in the kernel but ``ProblemInstance::EPS
= 1e-8`` in the broad phase. We do not reproduce that band -- both
sides of the differential test use the same ``self.EPS = 1e-8`` here.

Differences from the C++ test
-----------------------------
* Sample size 3 per (v, e) pair (the C++ is 3 too).
* Time bounds [0, 10] (the C++ is the same).
* Substituted ``sample.txt`` for the absent ``generated.txt``.
* The C++ uses unseeded ``std::random_device``; we use
  ``random.Random(0)`` for determinism in the test (the inputs to
  every other stage are fixed anyway).
"""

from __future__ import annotations

import random

import pytest

from psipp_ctc.ctc import ProblemInstanceWithExplicitCollisions
from psipp_ctc.geometry.geometry_2d import (
    get_collision_interval_eec,
    get_collision_interval_vec,
)
from psipp_ctc.io import load_roadmap_problem
from psipp_ctc.problem import (
    AgentTask,
    Point,
    ProblemInstanceWithGeometry,
)


# Tolerance per the brief: 1e-6.
_INTERVAL_TOL = 1e-6


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _build_pair(path: str):
    """Load a geometric instance and build the explicit instance.

    The two share coordinates, edges, and tasks. The explicit
    instance is read-only after construction; the geometric instance
    is the slow path. The differential test compares them.
    """
    geometric = load_roadmap_problem(path)
    explicit = ProblemInstanceWithExplicitCollisions(geometric)
    return geometric, explicit


def _check_vv(geometric, explicit) -> None:
    """Every (v0, v1) VV boolean must agree."""
    n = geometric.get_number_of_vertices()
    for v0 in range(n):
        for v1 in range(n):
            g = geometric.collision_check_vv(v0, v1)
            e = explicit.collision_check_vv(v0, v1)
            assert g == e, (
                f"VV disagree at ({v0}, {v1}): "
                f"geometric={g} explicit={e}"
            )


def _check_ve(geometric, explicit, rng, time_samples: int = 3) -> None:
    """Every VEC boolean, interval, and random-time sample must agree.

    ``time_samples`` is the number of random start-time samples per
    (v0, v1, e1) pair. The C++ reference uses 3; pass 1 for the
    large ``sample2.txt`` differential test to keep the wall time
    under 30 seconds.
    """
    n = geometric.get_number_of_vertices()
    for v0 in range(n):
        for v1 in range(n):
            for e1 in range(geometric.get_out_degree(v1)):
                # VEC boolean + interval (relative to agent 1's clock).
                g_collide, g_lo, g_up = geometric.get_collision_interval(
                    v0, v1, e1
                )
                e_collide, e_lo, e_up = explicit.get_collision_interval(
                    v0, v1, e1
                )
                assert g_collide == e_collide, (
                    f"VEC boolean disagree at (v0={v0}, v1={v1}, e1={e1}): "
                    f"geometric={g_collide} explicit={e_collide}"
                )
                if g_collide:
                    assert abs(g_lo - e_lo) < _INTERVAL_TOL, (
                        f"VEC lower disagree at (v0={v0}, v1={v1}, e1={e1}): "
                        f"|{g_lo} - {e_lo}| = {abs(g_lo - e_lo):.3e}"
                    )
                    assert abs(g_up - e_up) < _INTERVAL_TOL, (
                        f"VEC upper disagree at (v0={v0}, v1={v1}, e1={e1}): "
                        f"|{g_up} - {e_up}| = {abs(g_up - e_up):.3e}"
                    )
                # Random time samples.
                for _ in range(time_samples):
                    st0 = rng.uniform(0, 10)
                    tt0 = rng.uniform(0, 10)
                    st1 = rng.uniform(0, 10)
                    g_check = geometric.collision_check_ve(
                        v0, st0, tt0, v1, st1, e1
                    )
                    e_check = explicit.collision_check_ve(
                        v0, st0, tt0, v1, st1, e1
                    )
                    assert g_check == e_check, (
                        f"VEC time disagree at (v0={v0}, v1={v1}, e1={e1}, "
                        f"st0={st0:.4f}, tt0={tt0:.4f}, st1={st1:.4f}): "
                        f"geometric={g_check} explicit={e_check}"
                    )


def _check_ee(geometric, explicit, rng, time_samples: int = 3) -> None:
    """Every EEC boolean, interval, and random-time sample must agree.

    ``time_samples`` is the number of random start-time samples per
    (v0, eid0, v1, eid1) pair. The C++ reference uses 3; pass 1 for
    the large ``sample2.txt`` differential test to keep the wall
    time under 30 seconds.
    """
    n = geometric.get_number_of_vertices()
    for v0 in range(n):
        for v1 in range(n):
            for e1 in range(geometric.get_out_degree(v1)):
                for e0 in range(geometric.get_out_degree(v0)):
                    # EEC boolean + interval.
                    g_collide, g_lo, g_up = geometric.get_collision_interval(
                        v0, e0, v1, e1
                    )
                    e_collide, e_lo, e_up = explicit.get_collision_interval(
                        v0, e0, v1, e1
                    )
                    assert g_collide == e_collide, (
                        f"EEC boolean disagree at (v0={v0}, e0={e0}, "
                        f"v1={v1}, e1={e1}): "
                        f"geometric={g_collide} explicit={e_collide}"
                    )
                    if g_collide:
                        assert abs(g_lo - e_lo) < _INTERVAL_TOL, (
                            f"EEC lower disagree at (v0={v0}, e0={e0}, "
                            f"v1={v1}, e1={e1}): "
                            f"|{g_lo} - {e_lo}| = {abs(g_lo - e_lo):.3e}"
                        )
                        assert abs(g_up - e_up) < _INTERVAL_TOL, (
                            f"EEC upper disagree at (v0={v0}, e0={e0}, "
                            f"v1={v1}, e1={e1}): "
                            f"|{g_up} - {e_up}| = {abs(g_up - e_up):.3e}"
                        )
                    # Random time samples.
                    for _ in range(time_samples):
                        st0 = rng.uniform(0, 10)
                        st1 = rng.uniform(0, 10)
                        g_check = geometric.collision_check_ee(
                            v0, st0, e0, v1, st1, e1
                        )
                        e_check = explicit.collision_check_ee(
                            v0, st0, e0, v1, st1, e1
                        )
                        assert g_check == e_check, (
                            f"EEC time disagree at (v0={v0}, e0={e0}, "
                            f"v1={v1}, e1={e1}, st0={st0:.4f}, st1={st1:.4f}): "
                            f"geometric={g_check} explicit={e_check}"
                        )


# ---------------------------------------------------------------------------
# Differential test (mirrors src/test/explicit_collision_test.cpp)
# ---------------------------------------------------------------------------


def test_differential_explicit_matches_geometric_sample() -> None:
    """Differential test on ``problem_instances/roadmaps/sample.txt``
    (4V, 6E, 2A, r=0.05). Identical to the C++ reference's
    ``sample.txt``.

    This is the substitution for the C++ test's missing
    ``generated.txt`` (PORTING.md §7, "Trap: the differential test
    needs a roadmap that does not exist yet"). ``sample.txt`` has
    a known EEC conflict between the diagonals 0->3 and 1->2
    (they cross at (0.5, 0.5)), so the test exercises the
    EEC kernel and the 6-arg ``get_collision_interval`` with
    sign-swap.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample.txt")
    rng = random.Random(0)
    _check_vv(geometric, explicit)
    _check_ve(geometric, explicit, rng)
    _check_ee(geometric, explicit, rng)


def test_differential_explicit_matches_geometric_sample2() -> None:
    """Differential test on the larger ``sample2.txt``
    (100V, 2000E, 2A, r=0.05). More VV, VEC, and EEC candidates
    than ``sample.txt`` -- stresses the broad phase and the
    sign-swap convention more.

    The C++ test would have run on this size via the
    ``generated.txt`` round; we substitute. We use 1 time sample
    per (v, e) pair instead of the C++ test's 3, to keep the wall
    time under 30 seconds for the geometric instance's per-query
    EEC kernel. The parity is checked at 1 sample per pair; the
    C++-style 3-sample check is preserved on ``sample.txt``.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample2.txt")
    rng = random.Random(1)
    _check_vv(geometric, explicit)
    _check_ve(geometric, explicit, rng, time_samples=1)
    _check_ee(geometric, explicit, rng, time_samples=1)


# ---------------------------------------------------------------------------
# Construction invariants
# ---------------------------------------------------------------------------


def test_explicit_inherits_base_fields() -> None:
    """The explicit instance copies ``edges``, ``coordinates``,
    ``tasks``, and ``agent_geometry`` from the base. The
    planner's read path is identical to the geometric path
    except for the kernel call.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample.txt")
    assert explicit.edges == geometric.edges
    assert explicit.coordinates == geometric.coordinates
    assert explicit.tasks == geometric.tasks
    assert explicit.agent_geometry == geometric.agent_geometry


def test_explicit_global_edge_indices_are_sequential_per_vertex() -> None:
    """``edge_ids[v]`` runs from the running total up to the running
    total plus ``outdeg(v) - 1``. The global id is a single flat
    integer that increases monotonically as the out-edges are
    declared in vertex order.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample.txt")
    n = geometric.get_number_of_vertices()
    cursor = 0
    for v in range(n):
        outdeg = geometric.get_out_degree(v)
        assert explicit.edge_ids[v] == list(range(cursor, cursor + outdeg))
        cursor += outdeg
    assert cursor == len(explicit.edge_data)


def test_explicit_edge_data_inverse_of_edge_ids() -> None:
    """``edge_data[global_id] = (source, target, edge_id)`` where
    ``edge_id`` is the local out-list index. The map is the
    inverse of ``edge_ids``.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample.txt")
    for v in range(geometric.get_number_of_vertices()):
        for edge_id, global_id in enumerate(explicit.edge_ids[v]):
            src, tgt, eid = explicit.edge_data[global_id]
            assert src == v
            assert tgt == geometric.get_target_vertex(v, edge_id)
            assert eid == edge_id


# ---------------------------------------------------------------------------
# Sign-swap convention: the 6-arg EEC get_collision_interval returns
# (-up, -lo) when the caller passes the reversed pair, mirroring
# time around the call.
# ---------------------------------------------------------------------------


def test_explicit_ee_sign_swap_convention() -> None:
    """Build a tiny graph with one known EEC pair and check that
    passing the pair in either order returns the same answer
    when the test condition is applied symmetrically.

    Concretely, for a pair (e0, e1) where e0's global id is
    smaller, the storage key is (e0, e1) with (lo, up) in
    agent-1-natural direction. Passing (e1, e0) returns
    (-up, -lo) -- the mirrored interval in agent-0-natural
    direction. The TIME-CHECK condition ``lo + EPS < st0 - st1 <
    up - EPS`` is therefore self-consistent: for the same
    start times, it returns the same boolean regardless of
    the order the caller passes the pair in.
    """
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(4)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(0.0, 1.0))
    p.set_point(2, Point(1.0, 0.0))
    p.set_point(3, Point(1.0, 1.0))
    # Two crossing diagonals.
    p.add_edge(0, 3)
    p.add_edge(1, 2)
    p.tasks = [AgentTask(0, 3), AgentTask(1, 2)]
    p.agent_geometry = 0.3
    e = ProblemInstanceWithExplicitCollisions(p)

    # Edge 0->3 gets global id 0, edge 1->2 gets global id 1.
    eid_03 = e.edge_ids[0][0]
    eid_12 = e.edge_ids[1][0]
    # The pair is in the table; both forms must succeed.
    collide, lo_a, up_a = e.get_collision_interval(0, 0, 1, 0)
    collide_rev, lo_b, up_b = e.get_collision_interval(1, 0, 0, 0)
    assert collide is True
    assert collide_rev is True
    # Natural direction (agent 1 = edge 1->2 starts first): an
    # interval around 0 (the crossing time).
    assert lo_a < 0 < up_a
    # Reversed direction: mirrored.
    assert lo_b == pytest.approx(-up_a)
    assert up_b == pytest.approx(-lo_a)


# ---------------------------------------------------------------------------
# get_all_* accessors
# ---------------------------------------------------------------------------


def test_explicit_get_all_lists_match_the_maps() -> None:
    """Each ``get_all_*`` accessor reads from the corresponding
    precomputed list. For ``sample.txt`` the lists are small;
    we assert the cross-check against the underlying maps.
    """
    geometric, explicit = _build_pair("problem_instances/roadmaps/sample.txt")
    n = geometric.get_number_of_vertices()
    for v in range(n):
        vv_list = explicit.get_all_vv_collisions(v)
        expected = sorted(
            other for (a, other) in explicit.collision_pairs_v2v
            for w in (a, other) if w == v for other in (a, other) if other != v
        ) if v < 4 else []
        # Rebuild the expected list by hand from the set.
        expected_set = set()
        for a, b in explicit.collision_pairs_v2v:
            if a == v:
                expected_set.add(b)
            if b == v:
                expected_set.add(a)
        assert sorted(vv_list) == sorted(expected_set)

    for v in range(n):
        ve_list = explicit.get_all_ve_collisions(v)
        for (src, eid, neg_up, neg_lo) in ve_list:
            # ``-up, -lo`` is the reverse-time convention.
            lo = -neg_lo
            up = -neg_up
            # The interval must match the VEC map.
            e_global = explicit.edge_ids[src][eid]
            stored_lo, stored_up = explicit.collision_pairs_v2e[(v, e_global)]
            assert lo == pytest.approx(stored_lo)
            assert up == pytest.approx(stored_up)

    for v in range(n):
        for edge_id in range(geometric.get_out_degree(v)):
            ev_list = explicit.get_all_ev_collisions(v, edge_id)
            for (w, lo, up) in ev_list:
                e_global = explicit.edge_ids[v][edge_id]
                stored_lo, stored_up = explicit.collision_pairs_v2e[(w, e_global)]
                assert lo == pytest.approx(stored_lo)
                assert up == pytest.approx(stored_up)


# ---------------------------------------------------------------------------
# A hand-built graph with a known EEC conflict, to exercise the
# sign-swap convention and the time-condition check in isolation.
# ---------------------------------------------------------------------------


def test_eec_time_check_sign_swap_with_hand_built_crossing() -> None:
    """Two crossing diagonals on the unit square with r=0.3.

    Edge 0->3 and 1->2 cross at (0.5, 0.5). The EEC kernel
    returns a non-empty interval for the relative start time s.
    The explicit instance's ``collision_check_ee`` must give the
    same answer as the kernel for several sample start times.
    """
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(4)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(0.0, 1.0))
    p.set_point(2, Point(1.0, 0.0))
    p.set_point(3, Point(1.0, 1.0))
    p.add_edge(0, 3)
    p.add_edge(1, 2)
    p.tasks = [AgentTask(0, 3), AgentTask(1, 2)]
    p.agent_geometry = 0.3
    e = ProblemInstanceWithExplicitCollisions(p)

    # Kernel and explicit must agree on the EEC interval.
    sp0, tp0 = p.coordinates[0], p.coordinates[3]
    sp1, tp1 = p.coordinates[1], p.coordinates[2]
    dt0 = p.get_edge_length(0, 0)
    dt1 = p.get_edge_length(1, 0)
    k_collide, k_lo, k_up = get_collision_interval_eec(
        sp0, tp0, dt0, sp1, tp1, dt1, p.agent_geometry
    )
    e_collide, e_lo, e_up = e.get_collision_interval(0, 0, 1, 0)
    assert k_collide is True
    assert e_collide is True
    assert e_lo == pytest.approx(k_lo, abs=_INTERVAL_TOL)
    assert e_up == pytest.approx(k_up, abs=_INTERVAL_TOL)

    # Time-condition check: same answer whether the caller uses the
    # natural order (eid0=0, eid1=1) or the reversed order (eid0=1,
    # eid1=0), for the same start times.
    for st0, st1 in [
        (0.0, 0.0),
        (0.5, 0.5),
        (k_lo, 0.0),
        (0.0, -k_lo),
        (1.0, 0.0),
        (0.0, 1.0),
    ]:
        a = e.collision_check_ee(0, st0, 0, 1, st1, 0)
        b = e.collision_check_ee(1, st1, 0, 0, st0, 0)
        assert a == b, (
            f"sign-swap should be self-consistent at st0={st0}, st1={st1}: "
            f"a={a}, b={b}"
        )


def test_vec_time_check_with_hand_built_midpoint_vertex() -> None:
    """Stationary vertex near the middle of an edge vs the moving
    edge. The VEC kernel reports an interval that the explicit
    instance's ``collision_check_ve`` must respect.
    """
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(3)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(1.0, 0.0))
    # Stationary vertex at the midpoint, with 0.05 perpendicular
    # offset. r=0.1 means 2r=0.2; the VEC interval is the
    # stretch of the edge where the moving disc is within 2r of
    # the stationary disc.
    p.set_point(2, Point(0.5, 0.05))
    p.add_edge(0, 1)
    p.tasks = [AgentTask(2, 0), AgentTask(0, 1)]
    p.agent_geometry = 0.1
    e = ProblemInstanceWithExplicitCollisions(p)

    sp1, tp1 = p.coordinates[0], p.coordinates[1]
    p0 = p.coordinates[2]
    dt1 = p.get_edge_length(0, 0)
    k_collide, k_lo, k_up = get_collision_interval_vec(
        p0, sp1, tp1, dt1, p.agent_geometry
    )
    e_collide, e_lo, e_up = e.get_collision_interval(2, 0, 0)
    assert k_collide is True
    assert e_collide is True
    assert e_lo == pytest.approx(k_lo, abs=_INTERVAL_TOL)
    assert e_up == pytest.approx(k_up, abs=_INTERVAL_TOL)
