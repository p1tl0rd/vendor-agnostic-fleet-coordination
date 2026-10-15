"""Stage 0 -- data model tests.

Covers the three items the brief calls out:
  1. Edge id IS the out-list index (``add_edge`` appends).
  2. ``get_edge_length`` equals a hand-computed distance.
  3. The reentrant timer accumulates nested intervals correctly.

Also covers the trap that ``set_number_of_vertex`` is overridden in the
geometry subclass to resize BOTH ``coordinates`` and ``edges`` (PORTING.md §3
trap 1).
"""

from __future__ import annotations

import math
import time

import pytest

from psipp_ctc.problem import (
    AgentTask,
    Move,
    Point,
    ProblemInstance,
    ProblemInstanceWithGeometry,
)


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


def test_point_carries_x_and_y() -> None:
    p = Point(1.0, 2.0)
    assert p.x == 1.0
    assert p.y == 2.0


def test_agent_task_carries_initial_and_goal() -> None:
    t = AgentTask(initial_vertex=3, goal_vertex=7)
    assert t.initial_vertex == 3
    assert t.goal_vertex == 7


def test_move_carries_edge_id_and_start_time() -> None:
    move = Move(edge_id=2, start_time=1.5)
    assert move.edge_id == 2
    assert move.start_time == 1.5


def test_move_wait_is_negative_edge_id() -> None:
    """Waiting is encoded as ``edge_id < 0``; ``source == target`` is the
    equivalent check on the planner side (HANDOFF.md §5 invariant 5)."""
    wait = Move(edge_id=-1, start_time=0.0)
    assert wait.edge_id < 0


def test_start_time_is_a_float_not_an_int_tick() -> None:
    """``start_time`` is a real number in continuous time, not a step index."""
    m = Move(edge_id=0, start_time=0.123456789)
    assert isinstance(m.start_time, float)


# ---------------------------------------------------------------------------
# Edge id IS the out-list index
# ---------------------------------------------------------------------------


def test_add_edge_appends_in_order() -> None:
    """Insertion order IS the edge id; ``add_edge`` does not reuse slots."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(3)
    p.add_edge(0, 1)
    p.add_edge(0, 2)
    p.add_edge(1, 2)

    assert p.get_out_degree(0) == 2
    assert p.get_out_degree(1) == 1
    assert p.get_out_degree(2) == 0

    # The first edge from 0 leads to 1, the second to 2.
    assert p.get_target_vertex(0, 0) == 1
    assert p.get_target_vertex(0, 1) == 2
    assert p.get_target_vertex(1, 0) == 2


def test_edge_ids_run_0_to_out_degree_minus_one() -> None:
    """Edge ids for vertex ``v`` are exactly the integers
    ``0, 1, ..., out_degree(v) - 1``."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(1)
    targets = [10, 20, 30, 40, 50]
    for target in targets:
        p.add_edge(0, target)
    for k, expected in enumerate(targets):
        assert p.get_target_vertex(0, k) == expected


# ---------------------------------------------------------------------------
# set_number_of_vertex trap: the geometry subclass resizes BOTH arrays
# ---------------------------------------------------------------------------


def test_geometry_set_number_of_vertex_resizes_both_arrays() -> None:
    """Trap from PORTING.md §3 trap 1: ``coordinates`` and ``edges`` must
    stay in lock-step. If a future refactor accidentally drops the override
    and falls back to the base method, this test fails."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(5)
    assert len(p.coordinates) == 5
    assert len(p.edges) == 5

    # After populating, resizing must wipe both arrays consistently.
    p.set_point(0, Point(1.0, 2.0))
    p.add_edge(0, 1)
    p.set_number_of_vertex(3)
    assert len(p.coordinates) == 3
    assert len(p.edges) == 3
    # Default-constructed Point.
    assert p.coordinates[0] == Point(0.0, 0.0)
    assert p.edges[0] == []


def test_base_set_number_of_vertex_resizes_only_edges() -> None:
    """The base class has no ``coordinates`` field. ``set_number_of_vertex``
    only touches ``edges``. (This is a sanity check, not the trap itself.)"""
    p = ProblemInstance()
    p.set_number_of_vertex(4)
    assert p.get_number_of_vertices() == 4
    assert all(ls == [] for ls in p.edges)


def test_geometry_subclass_inherits_topology_methods() -> None:
    """``add_edge``, ``get_target_vertex`` etc. work on the geometry subclass
    via inheritance -- no duplicate code."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.add_edge(0, 1)
    assert p.get_out_degree(0) == 1
    assert p.get_target_vertex(0, 0) == 1


# ---------------------------------------------------------------------------
# get_edge_length == Euclidean distance
# ---------------------------------------------------------------------------


def test_get_edge_length_3_4_5_triangle() -> None:
    """Hand-computed distance: a 3-4-5 triangle gives 5.0."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(3.0, 4.0))
    p.add_edge(0, 1)
    assert p.get_edge_length(0, 0) == pytest.approx(5.0)


def test_get_edge_length_is_symmetric() -> None:
    """The edge length is just a distance, so direction does not matter."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(3.0, 4.0))
    p.add_edge(0, 1)
    p.add_edge(1, 0)
    forward = p.get_edge_length(0, 0)
    backward = p.get_edge_length(1, 0)
    assert forward == pytest.approx(backward)
    assert forward == pytest.approx(5.0)


def test_get_edge_length_zero_for_self_loop() -> None:
    """A self-loop is length zero. The geometry kernel uses this to skip
    short edges (see ``problem_with_geometry.hpp:112-114``'s
    ``if (length < EPS) return timer_off(false)``)."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(1)
    p.set_point(0, Point(1.0, 1.0))
    p.add_edge(0, 0)
    assert p.get_edge_length(0, 0) == pytest.approx(0.0)


def test_get_edge_length_equals_travel_time_at_unit_speed() -> None:
    """The unit-speed assumption: a 1.0-unit edge takes 1.0 time unit to
    traverse. This is the property that makes the A*'s Euclidean heuristic
    admissible (HANDOFF.md §5 invariant 4)."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(1.0, 0.0))
    p.add_edge(0, 1)
    assert p.get_edge_length(0, 0) == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Reentrant timer
# ---------------------------------------------------------------------------


def test_timer_records_a_single_interval() -> None:
    """Baseline: one ``timer_on`` / ``timer_off`` pair records the time it
    covers."""
    p = ProblemInstance()
    p.timer_on()
    time.sleep(0.05)
    p.timer_off(None)
    assert p.used_time >= 0.04  # allow for clock granularity
    assert p.used_time < 0.30


def test_timer_off_returns_its_argument() -> None:
    """The C++ template returns ``ret`` so callers can write
    ``return timer_off(do_work())``. Python port keeps that."""
    p = ProblemInstance()
    p.timer_on()
    result = p.timer_off("answer")
    assert result == "answer"


def test_timer_accumulates_two_disjoint_intervals() -> None:
    """Sequential, non-nested pairs both contribute to ``used_time``."""
    p = ProblemInstance()
    p.timer_on()
    time.sleep(0.03)
    p.timer_off(None)
    after_first = p.used_time
    assert after_first >= 0.02

    p.timer_on()
    time.sleep(0.03)
    p.timer_off(None)
    after_second = p.used_time
    # Both intervals contributed.
    assert after_second >= after_first + 0.02


def test_timer_zero_duration_pair_records_near_zero() -> None:
    """A balanced pair with no work in between contributes essentially zero."""
    p = ProblemInstance()
    p.timer_on()
    p.timer_off(None)
    assert p.used_time < 0.01


def test_timer_nested_intervals_do_not_inflate() -> None:
    """Nested pairs must NOT double-count.

    The reference uses ``if (timer_switch++ == 0)`` so the start time is
    recorded only on the outermost call. With two sleeps of 0.04s, the outer
    interval covers ~0.08s. If the outer contribution is dropped, used_time
    is ~0.04s. If the inner is double-counted on top of the outer, used_time
    exceeds ~0.12s. The test pins used_time inside ``[0.06, 0.15]``, which
    accepts only the correct attribution.
    """
    p = ProblemInstance()
    p.timer_on()
    time.sleep(0.04)  # outer interval so far
    p.timer_on()
    time.sleep(0.04)  # inner interval: must NOT add a second contribution
    p.timer_off(None)
    p.timer_off(None)
    assert 0.06 <= p.used_time <= 0.15, (
        f"used_time {p.used_time:.4f} outside [0.06, 0.15]; "
        "the reentrant timer likely double-counted or dropped the outer "
        "interval"
    )


def test_timer_switch_is_zero_after_balanced_pair() -> None:
    """A balanced pair leaves the switch at 0, so a fresh outermost pair can
    start a new measurement."""
    p = ProblemInstance()
    p.timer_on()
    p.timer_off(None)
    assert p.timer_switch == 0
    # Second pair must therefore start a new measurement.
    wall_before = time.perf_counter()
    p.timer_on()
    time.sleep(0.02)
    p.timer_off(None)
    wall_elapsed = time.perf_counter() - wall_before
    assert p.used_time >= 0.5 * wall_elapsed


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------


def test_eps_constant_value_and_visibility() -> None:
    """``ProblemInstance.EPS`` is 1e-8, accessible via the class and via
    instances (PORTING.md §5 trap 5: do not unify with ``Geometry2D::EPS = 0``)."""
    assert ProblemInstance.EPS == 1e-8
    p = ProblemInstance()
    assert p.EPS == 1e-8
    g = ProblemInstanceWithGeometry()
    assert g.EPS == 1e-8


def test_inf_constant_is_infinity() -> None:
    """``INF`` is +infinity, used as a sentinel for empty intervals in the
    reference's inequality solver."""
    p = ProblemInstance()
    assert p.INF == math.inf
    assert ProblemInstance.INF == math.inf


# ---------------------------------------------------------------------------
# Geometry subclass specifics
# ---------------------------------------------------------------------------


def test_agent_geometry_is_a_radius() -> None:
    """The reference's ``agent_geometry`` is a radius; the conflict
    threshold is ``2 * radius`` (strict) in the geometry kernels."""
    p = ProblemInstanceWithGeometry()
    p.agent_geometry = 0.5
    assert p.get_agent_radius() == 0.5


def test_get_coordinate_x_and_y_return_components() -> None:
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(1.5, 2.5))
    p.set_point(1, Point(-3.0, 4.0))
    assert p.get_coordinate_x(0) == 1.5
    assert p.get_coordinate_y(0) == 2.5
    assert p.get_coordinate_x(1) == -3.0
    assert p.get_coordinate_y(1) == 4.0
