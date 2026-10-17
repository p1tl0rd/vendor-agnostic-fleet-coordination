"""The geometry module.

The continuous-time collision algebra (:mod:`.geometry_2d`) and the
broad-phase candidate generators (:mod:`.collision_report`). Both
are pure functions / classes -- no ROS, no CGAL, no boost.
"""

from psipp_ctc.geometry.collision_report import (
    BentleyOttoman,
    Buckets2D,
)
from psipp_ctc.geometry.geometry_2d import (
    GEOMETRY_EPS,
    INF,
    collision_check_moving,
    collision_check_static,
    collision_check_vertex_edge,
    distance,
    get_collision_interval_eec,
    get_collision_interval_vec,
    solve_linear_inequation,
    solve_quadratic_inequation,
    solve_vector_inequation,
)

__all__ = [
    "GEOMETRY_EPS",
    "INF",
    "BentleyOttoman",
    "Buckets2D",
    "collision_check_moving",
    "collision_check_static",
    "collision_check_vertex_edge",
    "distance",
    "get_collision_interval_eec",
    "get_collision_interval_vec",
    "solve_linear_inequation",
    "solve_quadratic_inequation",
    "solve_vector_inequation",
]
