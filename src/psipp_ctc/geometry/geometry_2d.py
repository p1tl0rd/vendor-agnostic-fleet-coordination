"""The continuous-time collision algebra.

This is the algebraic heart of the system. Every continuous-time collision
query in the planner funnels through one of two kernels:

  * VEC (Vertex-Edge Conflict) -- a stationary disc at a vertex vs an
    agent moving along an edge. Reduces to ``|a*t + b| <= 2r`` on the
    moving agent's own time window, clipped to ``[0, dt]``.

  * EEC (Edge-Edge Conflict) -- two agents, each on their own edge. The
    return is the **union of start-time offsets ``s``** (relative to
    agent 1) for which the two come within ``2r`` at some shared time
    ``t in [0, dt0]`` with ``s <= t <= s+dt1``. The minimum of the
    distance is achieved at one of five candidate configurations:
    ``t = 0``, ``t = s``, ``t = dt1``, ``t = s+dt0``, or the interior
    perpendicular case. Each candidate contributes a *range* of ``s``
    that the kernel intersects with the linear motion constraints and
    union-merges with the survivors.

Both kernels are built on three primitive inequality solvers:

  * ``solve_linear_inequation``   -- ``A*x + B <= 0``
  * ``solve_quadratic_inequation`` -- ``A*x^2 + 2*B*x + C <= 0``, ``A >= 0``
  * ``solve_vector_inequation``    -- ``|a*t + b| <= r`` (reduces to quadratic)

The inequality solvers mutate the (lower, upper) pair in the C++ via
reference. In Python we return new tuples. Note the load-bearing
asymmetry in the C++: the linear solver INTERSECTS with the input range,
the quadratic solver REPLACES it. The EEC interior case depends on
this: the quadratic resets the bounds to the unconstrained solution,
then four linear calls intersect with linear motion constraints.

Preserve exactly
----------------
* ``Geometry2D.EPS = 0.0``. Comparisons are STRICT. Do not introduce a
  tolerance here; the planner depends on the exact value at 1e-8 scale.
* Float expression grouping. The C++ writes
  ``sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2))``; the Python port
  uses ``math.hypot`` which is bit-equivalent on IEEE-754 doubles.
* Sign conventions, root order, and the case-order in EEC (t=0, t=s,
  t=dt1, t=s+dt0, interior). The merge is min-of-lowers and max-of-uppers
  into the running ``[collision_start, collision_end]`` pair.
* The empty-interval sentinel ``(INF, -INF)`` (lower > upper).
* ``INF = math.inf`` represents positive infinity.
"""

from __future__ import annotations

import math
from typing import Tuple

from psipp_ctc.problem import Point

#: The geometry kernel's tolerance. Distinct from
#: ``ProblemInstance.EPS = 1e-8`` and ``SpatialProblemInstance.EPS = 1e-6``.
#: Do not unify.
GEOMETRY_EPS: float = 0.0

#: Sentinel for empty intervals and unbounded ranges. Used by the
#: EEC kernel's running merge.
INF: float = math.inf


# ---------------------------------------------------------------------------
# Primitive inequality solvers
# ---------------------------------------------------------------------------


def solve_linear_inequation(
    A: float, B: float, lower: float, upper: float
) -> Tuple[float, float]:
    """Solve ``A*x + B <= 0`` intersected with ``[lower, upper]``.

    * If ``A < -EPS`` (negative), the inequality pulls the **lower** bound
      up to ``-B/A``.
    * If ``A > EPS`` (positive), the inequality pulls the **upper** bound
      down to ``-B/A``.
    * If ``A ~ 0`` and ``B > 0``, the inequality is unsatisfiable -- return
      the empty-interval sentinel ``(INF, -INF)``.
    * If ``A ~ 0`` and ``B <= 0``, the inequality is satisfied everywhere
      -- return the input range unchanged.

    Note: the C++ **intersects** with the input range (max/min), unlike
    ``solve_quadratic_inequation`` which replaces. The asymmetry is
    load-bearing for the EEC interior case.
    """
    if A < -GEOMETRY_EPS:
        lower = max(lower, -B / A)
    elif A > GEOMETRY_EPS:
        upper = min(upper, -B / A)
    elif B > 0:
        return (INF, -INF)
    return (lower, upper)


def solve_quadratic_inequation(
    A: float, B: float, C: float, lower: float, upper: float
) -> Tuple[float, float]:
    """Solve ``A*x^2 + 2*B*x + C <= 0`` (``A >= 0``), REPLACING bounds.

    The C++ assumes ``A >= 0`` (precondition; the caller is responsible
    for the check). It REPLACES the input ``[lower, upper]`` with the
    unconstrained solution of the inequality -- it does NOT intersect.

    * ``A <= EPS`` is the degenerate-linear branch. (The C++ checks
      ``A < EPS`` strictly and divides by zero when ``A == 0``; the
      Python port handles ``A == 0`` in the same branch, which is what
      the C++ WOULD return if the NaN were filtered out by the
      ``lower < upper`` check that the EEC kernel applies at the call
      site. The EEC interior case can produce ``A == 0`` when the two
      agents move antiparallel at equal speed; the C++ produces NaN
      there and the EEC kernel silently drops the case.)
        - ``B < -EPS``: ``x >= C/(2B)`` so ``lower = C/(2B)``, upper = INF.
        - ``B > EPS``:  ``x <= C/(2B)`` so ``lower = -INF``, upper = C/(2B).
        - ``B ~ 0`` and ``C <= 0``: holds everywhere, ``(-INF, INF)``.
        - ``B ~ 0`` and ``C > 0``: never holds, ``(INF, -INF)``.
    * ``A > EPS``: discriminant ``D = B^2 - A*C``.
        - ``D < 0``: ``A > 0`` and no real roots means the quadratic is
          strictly positive everywhere, never <= 0. Empty ``(INF, -INF)``.
        - ``D >= 0``: roots ``(-B ± sqrt(D)) / A``; the inequality holds
          between them.

    The two ``lower``/``upper`` parameters are accepted for API symmetry
    with the linear solver and to allow chaining in the EEC interior
    case, but the quadratic branch overwrites them.

    Deviation from the C++: the strict ``A < EPS`` check is widened to
    ``A <= EPS`` so the ``A == 0`` case (antiparallel agents) is
    handled. The C++ crashes on this case via division by zero, but
    the resulting NaN is then filtered out by the ``lower < upper``
    check at the EEC call site, so the planner behaviour is the same
    as this Python port: the interior case contributes nothing.
    """
    if A <= GEOMETRY_EPS:
        if B < -GEOMETRY_EPS:
            return (C / B / 2.0, INF)
        elif B > GEOMETRY_EPS:
            return (-INF, C / B / 2.0)
        elif C <= 0:
            return (-INF, INF)
        else:
            return (INF, -INF)
    D = B * B - A * C
    if D < 0.0:
        return (INF, -INF)
    return ((-B - math.sqrt(D)) / A, (-B + math.sqrt(D)) / A)


def solve_vector_inequation(
    ax: float, ay: float, bx: float, by: float, r: float,
    lower: float, upper: float,
) -> Tuple[float, float]:
    """Solve ``|a*t + b| <= r``, REPLACING bounds.

    Reduces to the quadratic with
    ``A = |a|^2``, ``B = a.b``, ``C = |b|^2 - r^2``. The squared
    inequality ``|a*t + b|^2 <= r^2`` is equivalent to the original
    (``r >= 0``) and avoids the sign work in the absolute value.

    The EEC kernel relies on this REPLACING the bounds; the linear
    motion constraints are then intersected via subsequent
    ``solve_linear_inequation`` calls.
    """
    return solve_quadratic_inequation(
        ax * ax + ay * ay, ax * bx + ay * by, bx * bx + by * by - r * r,
        lower, upper,
    )


# ---------------------------------------------------------------------------
# Distance + simple collision checks
# ---------------------------------------------------------------------------


def distance(p0: Point, p1: Point) -> float:
    """Euclidean distance. C++ uses ``sqrt(pow(dx,2) + pow(dy,2))``; the
    Python ``math.hypot`` is bit-equivalent and more numerically robust."""
    return math.hypot(p0.x - p1.x, p0.y - p1.y)


def inner(p0: Point, p1: Point) -> float:
    """Dot product.

    Used by the broad phase to compare a vertex's projection onto a
    segment direction and perpendicular.
    """
    return p0.x * p1.x + p0.y * p1.y


def collision_check_static(
    p0: Point, p1: Point, r: float, eps: float = GEOMETRY_EPS
) -> bool:
    """Two stationary disc agents: conflict iff ``distance < 2r - eps``.

    The default ``eps = 0`` matches the geometry module's strict
    convention; ``ProblemInstance`` callers pass their own ``1e-8``
    here.
    """
    return distance(p0, p1) < 2.0 * r - eps


# ---------------------------------------------------------------------------
# VEC kernel: stationary vertex vs moving edge
# ---------------------------------------------------------------------------


def get_collision_interval_vec(
    p0: Point, sp1: Point, tp1: Point, dt1: float, r: float
) -> Tuple[bool, float, float]:
    """VEC kernel: stationary ``p0`` vs agent moving ``sp1 -> tp1`` over ``dt1``.

    Returns ``(collide, collision_start, collision_end)`` where
    ``[collision_start, collision_end]`` is clipped to ``[0, dt1]`` and
    is the time range on the moving agent's clock during which the two
    come within ``2r``.

    The construction:

        a = -(tp1 - sp1) / dt1
        b = p0 - sp1
        solve |a*t + b| <= 2r  ->  [lower, upper]
        collision_start = max(0, lower)
        collision_end   = min(dt1, upper)

    Note ``lower < upper`` is STRICT: if the solving interval is a single
    point, the function reports no collision.
    """
    if dt1 == 0.0:
        # Degenerate zero-length edge (self-loop or coincident
        # endpoints). The C++ caller in
        # ``problem_with_geometry.hpp:111-114`` has this guard
        # before invoking the kernel; we add it at the kernel
        # itself for robustness. The planner's behaviour for
        # non-degenerate edges is unchanged.
        return (False, 0.0, 0.0)
    ax = -(tp1.x - sp1.x) / dt1
    ay = -(tp1.y - sp1.y) / dt1
    bx = p0.x - sp1.x
    by = p0.y - sp1.y
    lower, upper = solve_vector_inequation(ax, ay, bx, by, 2.0 * r, 0.0, 0.0)
    collision_start = max(0.0, lower)
    collision_end = min(dt1, upper)
    return (collision_start < collision_end, collision_start, collision_end)


def collision_check_vertex_edge(
    p0: Point, st0: float, tt0: float,
    sp1: Point, st1: float, tp1: Point, tt1: float,
    r: float, eps: float = GEOMETRY_EPS,
) -> bool:
    """Stationary ``p0`` over ``[st0, tt0]`` vs moving agent in ``[st1, tt1]``.

    Returns True iff the VEC window overlaps the stationary agent's
    time window.
    """
    lower, upper = (0.0, 0.0)
    collide, lower, upper = get_collision_interval_vec(
        p0, sp1, tp1, tt1 - st1, r
    )
    if not collide:
        return False
    return max(st0, st1 + lower) + eps < min(tt0, st1 + upper)


# ---------------------------------------------------------------------------
# EEC kernel: two moving edges (the most important routine in the system)
# ---------------------------------------------------------------------------


def get_collision_interval_eec(
    sp0: Point, tp0: Point, dt0: float,
    sp1: Point, tp1: Point, dt1: float,
    r: float,
) -> Tuple[bool, float, float]:
    """EEC kernel: agent on edge ``sp0 -> tp0`` over ``dt0`` vs edge ``sp1 -> tp1`` over ``dt1``.

    Returns ``(collide, lower, upper)`` where ``[lower, upper]`` is the
    set of start-time offsets ``s`` (on agent 1's clock) such that the
    two agents come within ``2r`` at some shared time ``t in [0, dt0]``
    with ``s <= t <= s + dt1``.

    The five candidate configurations of the inter-agent distance
    minimum, in order:

        1. ``t = 0``     -- agent 0 at its start.
        2. ``t = s``     -- simultaneous start.
        3. ``t = dt1``   -- agent 1 at its end.
        4. ``t = s+dt0`` -- simultaneous finish.
        5. interior perpendicular case (the relative position is
           orthogonal to the relative velocity, ``t = a.(b + c*s)/|a|^2``).

    The first four reduce to a vector inequality ``|a*t + b| <= 2r``
    followed by a linear clamping to the motion constraints. The
    interior case requires ``|a|^2 > EPS``; if so it is a quadratic in
    ``s`` intersected with four linear motion constraints via
    ``solve_linear_inequation``.

    The five candidate ranges are min-merged into the running
    ``[collision_start, collision_end]`` pair (the merge is set UNION,
    not intersection, because the min over ``t`` can be achieved at
    different ``s`` values for different cases).
    """
    cx = -(tp0.x - sp0.x) / dt0
    cy = -(tp0.y - sp0.y) / dt0
    ax = -cx - (tp1.x - sp1.x) / dt1
    ay = -cy - (tp1.y - sp1.y) / dt1
    bx = sp0.x - sp1.x
    by = sp0.y - sp1.y

    collision_start = INF
    collision_end = -INF

    # case 1: t = 0 (agent 0 at its start, s in [-dt0, 0])
    lower, upper = solve_vector_inequation(cx, cy, bx, by, 2.0 * r, 0.0, 0.0)
    lower = max(lower, -dt0)
    upper = min(upper, 0.0)
    if lower < upper:
        collision_start = min(collision_start, lower)
        collision_end = max(collision_end, upper)

    # case 2: t = s (simultaneous start, s in [0, dt1])
    lower, upper = solve_vector_inequation(
        ax + cx, ay + cy, bx, by, 2.0 * r, 0.0, 0.0
    )
    lower = max(lower, 0.0)
    upper = min(upper, dt1)
    if lower < upper:
        collision_start = min(collision_start, lower)
        collision_end = max(collision_end, upper)

    # case 3: t = dt1 (agent 1 at its end, s in [dt1 - dt0, dt1])
    lower, upper = solve_vector_inequation(
        cx, cy, ax * dt1 + bx, ay * dt1 + by, 2.0 * r, 0.0, 0.0
    )
    lower = max(lower, dt1 - dt0)
    upper = min(upper, dt1)
    if lower < upper:
        collision_start = min(collision_start, lower)
        collision_end = max(collision_end, upper)

    # case 4: t = s + dt0 (simultaneous finish, s in [-dt0, dt1 - dt0])
    lower, upper = solve_vector_inequation(
        ax + cx, ay + cy, ax * dt0 + bx, ay * dt0 + by, 2.0 * r, 0.0, 0.0
    )
    lower = max(lower, -dt0)
    upper = min(upper, dt1 - dt0)
    if lower < upper:
        collision_start = min(collision_start, lower)
        collision_end = max(collision_end, upper)

    # case 5: interior perpendicular, requires |a|^2 > EPS
    if ax * ax + ay * ay > GEOMETRY_EPS:
        aa = ax * ax + ay * ay
        ab = ax * bx + ay * by
        ac = ax * cx + ay * cy
        bb = bx * bx + by * by
        cc = cx * cx + cy * cy
        cb = cx * bx + cy * by
        # Quadratic: resets bounds to the unconstrained solution.
        lower, upper = solve_quadratic_inequation(
            aa * cc - ac * ac,
            aa * cb - ac * ab,
            aa * (bb - 4.0 * r * r) - ab * ab,
            0.0, 0.0,
        )
        # Four linear motion constraints (intersect, not replace):
        # 0 <= -(ac*s + ab)         <= aa*dt1
        # aa*s <= -(ac*s + ab)      <= aa*s + aa*dt0
        lower, upper = solve_linear_inequation(ac, ab, lower, upper)
        lower, upper = solve_linear_inequation(-ac, -ab - aa * dt1, lower, upper)
        lower, upper = solve_linear_inequation(aa + ac, ab, lower, upper)
        lower, upper = solve_linear_inequation(-ac - aa, -ab - aa * dt0, lower, upper)
        if lower < upper:
            collision_start = min(collision_start, lower)
            collision_end = max(collision_end, upper)

    return (collision_start < collision_end, collision_start, collision_end)


# ---------------------------------------------------------------------------
# Full two-moving-agent boolean check (used by the dispatcher)
# ---------------------------------------------------------------------------


def collision_check_moving(
    sp0: Point, st0: float, tp0: Point, tt0: float,
    sp1: Point, st1: float, tp1: Point, tt1: float,
    r: float, eps: float = GEOMETRY_EPS,
) -> bool:
    """Two moving agents: True iff their time windows overlap AND
    the separating distance falls below ``2r`` at some shared time.

    The check is the closed-form evaluation of the quadratic
    ``A*t^2 + 2*B*t + C`` at the shared window endpoints and the vertex
    ``t = -B/A``.

    Note: this is a strict-strategy check used by the dispatcher when
    BOTH agents are moving. The interval version is the EEC kernel
    above. The two are inverses; this one returns a bool, that one
    returns the ``s``-range.
    """
    if tt0 - eps < st1 or tt1 - eps < st0:
        return False
    dt0 = tt0 - st0
    dt1 = tt1 - st1
    # Degenerate zero-length edge: no collision (the C++ caller in
    # ``problem_with_geometry.hpp:178-180`` has this guard for
    # ``get_collision_interval`` 6-arg; the bare moving-vs-moving
    # check has no guard, so we add it here for robustness).
    if dt0 == 0.0 or dt1 == 0.0:
        return False
    ax = (tp0.x - sp0.x) / dt0 - (tp1.x - sp1.x) / dt1
    bx = (sp0.x * tt0 - tp0.x * st0) / dt0 - (sp1.x * tt1 - tp1.x * st1) / dt1
    ay = (tp0.y - sp0.y) / dt0 - (tp1.y - sp1.y) / dt1
    by = (sp0.y * tt0 - tp0.y * st0) / dt0 - (sp1.y * tt1 - tp1.y * st1) / dt1
    A = ax * ax + ay * ay
    B = ax * bx + ay * by
    C = bx * bx + by * by - 4.0 * r * r
    st = max(st0, st1)
    tt = min(tt0, tt1)
    # Endpoint tests.
    if A * st * st + 2.0 * B * st + C < -eps:
        return True
    if A * tt * tt + 2.0 * B * tt + C < -eps:
        return True
    # Interior vertex: A>0 and -B/A strictly inside (st, tt), AND the
    # quadratic actually crosses zero (B^2 - A*C > 0).
    if A > eps and A * st + eps < -B and -B < A * tt - eps and B * B - A * C > eps:
        return True
    return False
