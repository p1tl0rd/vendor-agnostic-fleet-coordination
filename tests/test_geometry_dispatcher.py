"""Tests for the geometry-kernel dispatchers on ProblemInstance.

Exercises the VEC, EE, and VV kernels via ``ProblemInstanceWithGeometry``
(the class that ships with a 2D coordinate array and a disc radius) and
verifies that the dispatchers produce the same answers as the standalone
kernels in :mod:`psipp_ctc.geometry.geometry_2d`.

The C++ version of these tests is implicit: ``PrioritizedSIPP`` exercises
the dispatchers end-to-end, and the C++ property test on the kernels
exercises the kernels standalone. The Python port keeps them split
because the kernels are Stage 2 algebra and the dispatchers are Stage 2
plumbing -- it is easier to verify them independently.

Trap reminder (Stage 0 trap 1): ``set_number_of_vertex`` on the
geometry subclass resizes BOTH ``coordinates`` AND ``edges``. Calling
it after building the edges clears them. The fixtures below build the
edges only after the final resize.
"""

from __future__ import annotations

import pytest

from psipp_ctc.geometry.geometry_2d import (
    get_collision_interval_eec,
    get_collision_interval_vec,
)
from psipp_ctc.problem import (
    AgentTask,
    Point,
    ProblemInstanceWithGeometry,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _two_vertex_line(radius: float = 0.1) -> ProblemInstanceWithGeometry:
    """0 at (0,0), 1 at (1,0), edge 0->1, radius ``radius``."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(1.0, 0.0))
    p.add_edge(0, 1)
    p.tasks = [AgentTask(0, 1)]
    p.agent_geometry = radius
    return p


def _three_vertex_T(radius: float = 0.1) -> ProblemInstanceWithGeometry:
    """0 at (0,0), 1 at (1,0), 2 at (0,1); edges 0->1 and 0->2."""
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(3)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(1.0, 0.0))
    p.set_point(2, Point(0.0, 1.0))
    p.add_edge(0, 1)
    p.add_edge(0, 2)
    p.tasks = [AgentTask(0, 1)]
    p.agent_geometry = radius
    return p


def _three_vertex_with_midpoint(radius: float = 0.1) -> ProblemInstanceWithGeometry:
    """0 at (0,0), 1 at (1,0), 2 at (0.5, 0.05) (near the middle of edge 0->1).

    Vertex 2 is well within 2r of the edge's midpoint, so a stationary
    vertex-2 vs moving edge 0->1 should report a VEC collision.
    """
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(3)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(1.0, 0.0))
    p.set_point(2, Point(0.5, 0.05))
    p.add_edge(0, 1)
    p.tasks = [AgentTask(0, 1)]
    p.agent_geometry = radius
    return p


# ---------------------------------------------------------------------------
# VV kernel: distance(v0, v1) < 2r - EPS
# ---------------------------------------------------------------------------


def test_vv_collision_far_apart():
    p = _two_vertex_line(radius=0.1)
    # distance(0, 1) = 1, 2r = 0.2 -> no collision
    assert p.collision_check_vv(0, 1) is False


def test_vv_collision_close_together():
    p = ProblemInstanceWithGeometry()
    p.set_number_of_vertex(2)
    p.set_point(0, Point(0.0, 0.0))
    p.set_point(1, Point(0.05, 0.0))
    p.add_edge(0, 1)
    p.tasks = [AgentTask(0, 1)]
    p.agent_geometry = 0.1
    # distance(0, 1) = 0.05 < 0.2 - 1e-8 -> collision
    assert p.collision_check_vv(0, 1) is True


# ---------------------------------------------------------------------------
# VE kernel: VEC via dispatcher
# ---------------------------------------------------------------------------


def test_ve_collision_via_dispatcher_matches_kernel():
    p = _three_vertex_with_midpoint(radius=0.1)
    # Direct kernel call (relative-time view: agent 1's clock is [0, 1]).
    sp1 = p.get_point(0)
    tp1 = p.get_point(1)
    p0 = p.get_point(2)
    dt1 = p.get_edge_length(0, 0)
    collide_k, lo_k, up_k = get_collision_interval_vec(
        p0, sp1, tp1, dt1, p.agent_geometry
    )
    # Dispatcher: VE on (vertex 2, agent 1 starting at t=0 over [0, 10]).
    collide_d = p.collision_check_ve(2, 0.0, 10.0, 0, 0.0, 0)
    # The two should agree on the collision boolean.
    assert collide_k == collide_d
    if collide_k:
        assert lo_k < up_k


# ---------------------------------------------------------------------------
# EE kernel: EEC via dispatcher
# ---------------------------------------------------------------------------


def test_ee_collision_via_dispatcher_matches_kernel():
    p = _three_vertex_T(radius=0.1)
    # Direct EEC kernel: agent on edge 0->1 vs agent on edge 0->2.
    sp0 = p.get_point(0)
    tp0 = p.get_point(1)
    dt0 = p.get_edge_length(0, 0)
    sp1 = p.get_point(0)
    tp1 = p.get_point(2)
    dt1 = p.get_edge_length(0, 1)
    collide_k, _, _ = get_collision_interval_eec(
        sp0, tp0, dt0, sp1, tp1, dt1, p.agent_geometry
    )
    # Dispatcher: both start at t=0. Use the EE-with-end-times form.
    collide_d = p._collision_check_dispatch(
        0, 0.0, 0, dt0, 0, 0.0, 1, dt1
    )
    assert collide_k == collide_d


# ---------------------------------------------------------------------------
# get_collision_interval dispatcher
# ---------------------------------------------------------------------------


def test_get_collision_interval_ve_mirrors_time_frame():
    p = _three_vertex_with_midpoint(radius=0.1)
    sp1 = p.get_point(0)
    tp1 = p.get_point(1)
    p0 = p.get_point(2)
    dt1 = p.get_edge_length(0, 0)
    collide_k, lo_k, up_k = get_collision_interval_vec(
        p0, sp1, tp1, dt1, p.agent_geometry
    )
    # Dispatcher: agent 1 starts at t=5.
    st1 = 5.0
    collide_d, lo_d, up_d = p.get_collision_interval(
        2, 0, st1, 0, st1 + dt1
    )
    assert collide_k == collide_d
    if collide_k:
        # The dispatcher should shift by +5.
        assert lo_d == pytest.approx(lo_k + st1)
        assert up_d == pytest.approx(up_k + st1)


# ---------------------------------------------------------------------------
# Waiting agents (edge_id < 0) route to the right kernel
# ---------------------------------------------------------------------------


def test_waiting_routes_to_vv():
    p = _two_vertex_line(radius=0.1)
    # Both agents waiting at vertices 0 and 1: distance = 1 > 0.2 = 2r.
    result = p._collision_check_dispatch(
        0, 0.0, -1, 100.0,  # wait at 0
        1, 0.0, -1, 100.0,  # wait at 1
    )
    assert result is False


def test_one_waiting_routes_to_ve_with_collision():
    # Agent 0 waits at vertex 0, agent 1 moves along edge 0->1 starting
    # at t=0. At t=0 the moving agent IS at (0, 0) where the waiter is,
    # so the VEC kernel should report collision. This is the
    # "swap-when-edge_id_0<0" branch of the dispatcher.
    p = _two_vertex_line(radius=0.1)
    result = p._collision_check_dispatch(
        0, 0.0, -1, 100.0,  # wait at 0
        0, 0.0, 0, 1.0,     # move along edge 0->1
    )
    assert result is True
