"""Broad-phase collision candidate generation.

The candidate-generation step for the offline annotation. It does NOT
compute the exact interval -- it produces *pairs* that the CTC kernels
(VEC / EEC, in :mod:`psipp_ctc.geometry.geometry_2d`) then evaluate.
Two structures:

* :class:`Buckets2D` -- a uniform spatial hash with cell size equal to
  the conflict radius. Used for VV candidates and VE candidates (every
  vertex within the radius of a segment). Cell = ``radius``, so any two
  points that could be within ``2r`` of each other must share a bucket
  or sit in one of the 5 fixed neighbour offsets.

* :class:`BentleyOttoman` -- a Bentley-Ottmann sweep-line for crossing
  edges. The reference rotates the whole set of segments by a random
  angle so no segment is vertical, then sweeps in ``x`` with emerge /
  vanish / cross events. Every detected crossing is a guaranteed EEC
  candidate. The point of the sweep is to make edge-edge candidate
  generation ``O((n + k) log n)`` instead of ``O(n^2)``.

Preserve exactly
----------------
* Cell size = radius. A point at ``(x, y)`` is in cell
  ``(floor(x/radius), floor(y/radius))``.
* ``Buckets2D`` 5 fixed neighbour offsets: ``(0,0), (0,1), (1,-1),
  (1,0), (1,1)``. This enumerates every unordered pair exactly once.
* BentleyOttoman event ordering (earliest time, then emerge > vanish >
  cross on tie; for cross ties, smaller ``slope_0``, then smaller
  ``slope_1``, then smaller ``segment - segment_1``).
* The random rotation in ``BentleyOttoman.__init__``: segments are
  rotated by a uniform random angle in ``[-pi, pi]`` so no segment is
  vertical. The rotation uses an unseeded RNG, so cross-pair detection
  is not bit-reproducible run to run. Tests compare structural
  properties (number of crossings, who crosses), not coordinates.
* ``BentleyOttoman.EPS = 1e-11`` (its own internal tolerance, distinct
  from the geometry module's ``EPS = 0``).
"""

from __future__ import annotations

import math
import random
from typing import Dict, Iterable, List, Sequence, Set, Tuple

from psipp_ctc.geometry.geometry_2d import distance, inner
from psipp_ctc.problem import Point


# ---------------------------------------------------------------------------
# Buckets2D
# ---------------------------------------------------------------------------


# Fixed neighbour offsets for ``calculateAllPairs``. Picked so each
# unordered pair of adjacent cells is visited exactly once.
_BUCKET_NEIGHBOUR_OFFSETS: Tuple[Tuple[int, int], ...] = (
    (0, 0),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


class Buckets2D:
    """Uniform spatial hash for VV and VE candidates.

    The cell size is the conflict radius, so two points within ``2r``
    of each other must share a bucket or sit in one of the 5 fixed
    neighbour offsets.

    Two queries:

    * :meth:`calculate_all_pairs` -- VV candidates. Returns the set of
      unordered point pairs whose distance is strictly less than
      ``radius``. The 5-offset scan deduplicates by construction.
    * :meth:`calculate_collision_vertices` -- VE candidates. Returns the
      list of point ids that lie within ``radius`` perpendicular
      distance of the open segment ``s -> t``. The implementation walks
      the 4-corner rectangle of the inflated segment column by column.
    """

    def __init__(self, radius: float, points: Sequence[Point]) -> None:
        if radius <= 0.0:
            raise ValueError(f"radius must be positive, got {radius}")
        self.radius = float(radius)
        self.points: List[Point] = list(points)
        # Sorted-key map from cell coord to list of point ids in it.
        # We use a ``dict`` but iterate it sorted by key to match the
        # C++ ``std::map`` iteration order. The C++ relies on the
        # ordering only for the cross-pair traversal in
        # ``calculate_all_pairs``; the result set is order-independent.
        self._buckets: Dict[Tuple[int, int], List[int]] = {}
        for i, p in enumerate(self.points):
            cell = (int(math.floor(p.x / self.radius)),
                    int(math.floor(p.y / self.radius)))
            self._buckets.setdefault(cell, []).append(i)

    def calculate_all_pairs(self) -> Set[Tuple[int, int]]:
        """All unordered point pairs with distance < ``radius``.

        Each pair is emitted once (via the 5 fixed offsets and the
        ``min/max`` ordering of ids).
        """
        pairs: Set[Tuple[int, int]] = set()
        # Iterate buckets in deterministic (sorted) order to match the
        # C++ ``std::map`` iteration. The order does not affect the
        # returned set but keeps the traversal observable.
        for cell in sorted(self._buckets.keys()):
            ids = self._buckets[cell]
            for dx, dy in _BUCKET_NEIGHBOUR_OFFSETS:
                neighbour = (cell[0] + dx, cell[1] + dy)
                neighbour_ids = self._buckets.get(neighbour)
                if neighbour_ids is None:
                    continue
                for i in ids:
                    for j in neighbour_ids:
                        if distance(self.points[i], self.points[j]) < self.radius:
                            pairs.add((min(i, j), max(i, j)))
        return pairs

    def calculate_collision_vertices(
        self, s: Point, t: Point
    ) -> List[int]:
        """All point ids within ``radius`` of the open segment ``s -> t``.

        The 4-corner rectangle is the convex hull of the four points
        ``p0 ± u`` and ``p1 ± u`` where ``u`` is the unit perpendicular
        scaled by ``radius``. The function walks the bucket grid
        column-by-column, lower-bounded by the lower envelope and
        upper-bounded by the upper envelope.
        """
        r = self.radius
        # p0 has the smaller x; p1 has the larger x.
        if s.x <= t.x:
            p0, p1 = s, t
        else:
            p0, p1 = t, s
        # Perpendicular unit vector, scaled by ``radius``.
        vec = Point(p1.x - p0.x, p1.y - p0.y)
        vec_len_sq = inner(vec, vec)
        if vec_len_sq == 0.0:
            # Degenerate zero-length segment: fall back to a point
            # query at p0.
            return [
                pid for pid, p in enumerate(self.points)
                if distance(p, p0) < r
            ]
        scale = r / math.sqrt(vec_len_sq)
        u = Point(-vec.y * scale, vec.x * scale)
        if p0.y <= p1.y:
            l = Point(p0.x + u.x, p0.y + u.y)
            m0 = Point(p0.x - u.x, p0.y - u.y)
            m1 = Point(p1.x + u.x, p1.y + u.y)
            r_pt = Point(p1.x - u.x, p1.y - u.y)
        else:
            l = Point(p0.x - u.x, p0.y - u.y)
            m1 = Point(p0.x + u.x, p0.y + u.y)
            m0 = Point(p1.x - u.x, p1.y - u.y)
            r_pt = Point(p1.x + u.x, p1.y + u.y)

        L = int(math.floor(l.x / r))
        M0 = int(math.floor(m0.x / r))
        M1 = int(math.floor(m1.x / r))
        R = int(math.floor(r_pt.x / r))

        n_cols = R - L + 1
        lower = [0] * n_cols
        upper = [0] * n_cols
        # Lower envelope.
        for x in range(L, M0):
            lower[x - L] = int(math.floor(
                (l.y + ((x + 1) * r - l.x) / (m0.x - l.x) * (m0.y - l.y)) / r
            ))
        lower[M0 - L] = int(math.floor(m0.y / r))
        for x in range(M0 + 1, R + 1):
            lower[x - L] = int(math.floor(
                (m0.y + (x * r - m0.x) / (r_pt.x - m0.x) * (r_pt.y - m0.y)) / r
            ))
        # Upper envelope.
        for x in range(L, M1):
            upper[x - L] = int(math.floor(
                (l.y + ((x + 1) * r - l.x) / (m1.x - l.x) * (m1.y - l.y)) / r
            ))
        upper[M1 - L] = int(math.floor(m1.y / r))
        for x in range(M1 + 1, R + 1):
            upper[x - L] = int(math.floor(
                (m1.y + (x * r - m1.x) / (r_pt.x - m1.x) * (r_pt.y - m1.y)) / r
            ))

        result: List[int] = []
        for x in range(L, R + 1):
            for y in range(lower[x - L], upper[x - L] + 1):
                cell = self._buckets.get((x, y))
                if cell is None:
                    continue
                for pid in cell:
                    p = self.points[pid]
                    A = inner(Point(p.x - p0.x, p.y - p0.y), vec)
                    D = inner(Point(p.x - p0.x, p.y - p0.y), u)
                    if 0 < A < vec_len_sq and -r * r < D < r * r:
                        result.append(pid)
        return result


# ---------------------------------------------------------------------------
# BentleyOttoman sweep
# ---------------------------------------------------------------------------


_BENTLEY_EPS = 1e-11  # internal tolerance, separate from Geometry2D.EPS=0
_PI = 2.0 * math.asin(1.0)


class _Event:
    """One entry in the sweep-line priority queue.

    The strict ``<`` is preserved by storing the fields needed to
    reproduce the tiebreak order: time, then type (emerge > vanish > cross),
    then ``slope_0``, then ``slope_1``, then ``segment - segment_1``.
    """

    __slots__ = ("time", "type", "segment", "segment_1", "slope_0", "slope_1")

    EMERGE = 0
    VANISH = 1
    CROSS = 2

    def __init__(
        self, time: float, etype: int, segment: int,
        segment_1: int = -1, slope_0: float = 0.0, slope_1: float = 0.0,
    ) -> None:
        self.time = time
        self.type = etype
        self.segment = segment
        self.segment_1 = segment_1
        self.slope_0 = slope_0
        self.slope_1 = slope_1

    def heap_key(self) -> Tuple[float, int, float, float, int, int, int]:
        """A sort key that puts earlier-time events first, then
        emerge > vanish > cross, then slope tiebreak, then
        segment-segment_1.

        Python's min-heap pops the smallest key first, so ``time`` is
        in ascending order and the rest follow the C++ max-heap
        tiebreak inverted: emerge is type 0, vanish 1, cross 2, and
        within cross, smaller ``slope_0`` pops first (matching the
        C++ ``if slope_0 > event.slope_0: this < event``).
        """
        return (
            self.time,
            self.type,
            self.slope_0,
            self.slope_1,
            self.segment - self.segment_1,
            self.segment,
            self.segment_1,
        )

    def __lt__(self, other: "_Event") -> bool:
        """Required by ``heapq`` to compare events for the priority queue."""
        return self.heap_key() < other.heap_key()

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        names = ("emerge", "vanish", "cross")
        return (
            f"_Event(time={self.time}, type={names[self.type]}, "
            f"seg={self.segment}, seg1={self.segment_1})"
        )


class BentleyOttoman:
    """Bentley-Ottmann sweep for crossing segments.

    Detects every pair of segments that geometrically cross. The sweep
    is random-rotation-based: segments are rotated by a uniform random
    angle ``[-pi, pi]`` so no segment is vertical, then swept in ``x``
    with emerge / vanish / cross events.

    Note: the rotation is unseeded in the reference (rule 14). Two
    runs on the same segment set may detect different *orderings* of
    the same set of crossing pairs, but the pair SET itself is the
    same. Tests assert set membership, not order.
    """

    EPS: float = _BENTLEY_EPS

    def __init__(self, segments: Sequence[Tuple[Point, Point]]) -> None:
        # Random rotation. The C++ uses std::random_device +
        # default_random_engine; the Python equivalent is
        # ``random.SystemRandom`` (unseeded). Rule 14: never assert
        # bit-identical sample points against the oracle.
        rng = random.SystemRandom()
        angle = rng.uniform(-_PI, _PI)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        n = len(segments)
        self.segments: List[Tuple[Point, Point]] = [None] * n  # type: ignore[list-item]
        for i, (a, b) in enumerate(segments):
            ra = Point(cos_a * a.x - sin_a * a.y, sin_a * a.x + cos_a * a.y)
            rb = Point(cos_a * b.x - sin_a * b.y, sin_a * b.x + cos_a * b.y)
            if ra.x > rb.x:
                ra, rb = rb, ra
            self.segments[i] = (ra, rb)
        # Pre-compute line slopes (m, c) so y = m*x + c.
        self.lines: List[Tuple[float, float]] = [None] * n  # type: ignore[list-item]
        for i, (a, b) in enumerate(self.segments):
            if b.x - a.x == 0.0:
                # Vertical segment is excluded from the sweep (the
                # constructor pushes emerge/vanish for x == b.x, but
                # vertical means emerge.x == vanish.x). We store NaN
                # so the comparator can never match.
                self.lines[i] = (math.nan, math.nan)
            else:
                m = (b.y - a.y) / (b.x - a.x)
                c = a.y - m * a.x
                self.lines[i] = (m, c)
        self.cross_pairs: Set[Tuple[int, int]] = set()
        self._events: List[_Event] = []  # min-heap on heap_key
        import heapq as _heapq
        self._heapq = _heapq

    # -- Internal helpers ------------------------------------------------

    def _y_at(self, segment: int, time: float) -> float:
        m, c = self.lines[segment]
        if math.isnan(m):
            # Vertical segment: define y as ``time``-anchored (this
            # only matters if vertical segments leaked into the
            # sweep, which they don't because emerge/vanish are not
            # pushed for them).
            return math.nan
        return m * time + c

    def _maybe_push_cross(self, s0: int, s1: int) -> None:
        """checkPair: emit a cross event if the two segments cross
        within their x-extents. C++ lines 179-203.
        """
        m0, c0 = self.lines[s0]
        m1, c1 = self.lines[s1]
        if abs(m0 - m1) < self.EPS and abs(c0 - c1) < self.EPS:
            # Collinear: count as a cross.
            self.cross_pairs.add((min(s0, s1), max(s0, s1)))
            return
        if m0 <= m1 + self.EPS:
            return
        if (min(s0, s1), max(s0, s1)) in self.cross_pairs:
            return
        denom = m0 - m1
        if denom == 0.0:
            return
        cross_time = (c1 - c0) / denom
        # Reject if outside either segment's x-extent.
        if cross_time >= self.segments[s0][1].x - self.EPS:
            return
        if cross_time >= self.segments[s1][1].x - self.EPS:
            return
        self.cross_pairs.add((min(s0, s1), max(s0, s1)))
        ev = _Event(
            time=cross_time, etype=_Event.CROSS, segment=s0,
            segment_1=s1, slope_0=m0, slope_1=m1,
        )
        self._heapq.heappush(self._events, ev)

    # -- Public query ----------------------------------------------------

    def calculate_all_crosses(self) -> Set[Tuple[int, int]]:
        """All unordered segment pairs that geometrically cross.

        Emits emerge/vanish events for every non-vertical segment, then
        drains the priority queue.

        The on-line set is kept sorted by ``y = m*x + c`` at the
        current sweep ``time``. We re-sort at each event because
        Python's set / dict do not support a dynamic comparator (the
        C++ does it via a shared_ptr-bound lambda).
        """
        heapq = self._heapq
        # Build the event queue.
        for i, (a, b) in enumerate(self.segments):
            if a.x == b.x:
                # Vertical segment: skip (the C++ also skips by
                # checking ``segments[i].first.x == segments[i].second.x``
                # in the emerge/vanish push loop, lines 207-208).
                continue
            self._events.append(_Event(time=a.x, etype=_Event.EMERGE, segment=i))
            self._events.append(_Event(time=b.x, etype=_Event.VANISH, segment=i))
        heapq.heapify(self._events)

        # Sweep state.
        # ``on_line`` is a list of integer ids (assigned at emerge in
        # the C++) sorted by the comparator at the current time. The
        # mapping ``id_to_segments[id] = segment_id`` is maintained in
        # parallel.
        on_line: List[int] = []
        id_to_segments: List[int] = []
        current_time = 0.0
        next_id = 0
        # iterators[segment] = the position of the segment's id in
        # on_line, or -1 if not on line. C++ stores an iterator; we
        # store a list index.
        iterators: List[int] = [-1] * len(self.segments)

        def sort_on_line() -> None:
            on_line.sort(key=lambda i: self._y_at(id_to_segments[i], current_time))

        while self._events:
            ev = heapq.heappop(self._events)
            current_time = ev.time
            if ev.type == _Event.EMERGE:
                seg = ev.segment
                my_id = next_id
                next_id += 1
                id_to_segments.append(seg)
                # Insert into on_line in sorted order.
                target_y = self._y_at(seg, current_time)
                # Binary search for insertion position.
                lo, hi = 0, len(on_line)
                while lo < hi:
                    mid = (lo + hi) // 2
                    if self._y_at(id_to_segments[on_line[mid]], current_time) < target_y:
                        lo = mid + 1
                    else:
                        hi = mid
                on_line.insert(lo, my_id)
                # Update iterators of the shifted suffix.
                for k in range(lo, len(on_line)):
                    iterators[id_to_segments[on_line[k]]] = k
                iterators[seg] = lo
                if lo > 0:
                    self._maybe_push_cross(
                        id_to_segments[on_line[lo - 1]], seg
                    )
                if lo + 1 < len(on_line):
                    self._maybe_push_cross(
                        seg, id_to_segments[on_line[lo + 1]]
                    )
            elif ev.type == _Event.VANISH:
                seg = ev.segment
                it = iterators[seg]
                if it > 0 and it + 1 < len(on_line):
                    self._maybe_push_cross(
                        id_to_segments[on_line[it - 1]],
                        id_to_segments[on_line[it + 1]],
                    )
                # Remove the segment from on_line.
                for k in range(it + 1, len(on_line)):
                    iterators[id_to_segments[on_line[k]]] = k - 1
                del on_line[it]
                iterators[seg] = -1
            else:  # CROSS
                seg0, seg1 = ev.segment, ev.segment_1
                it0 = iterators[seg0]
                it1 = iterators[seg1]
                # The C++ asserts these are neighbours. If not, the
                # event is stale (the order changed since we pushed
                # it) and we nudge it forward by EPS and re-push.
                if it0 + 1 != it1:
                    ev.time += self.EPS
                    heapq.heappush(self._events, ev)
                    continue
                # Swap the two ids (and their segment mappings).
                on_line[it0], on_line[it1] = on_line[it1], on_line[it0]
                id_to_segments[on_line[it0]] = seg1
                id_to_segments[on_line[it1]] = seg0
                iterators[seg0] = it1
                iterators[seg1] = it0
                if it0 > 0:
                    self._maybe_push_cross(
                        id_to_segments[on_line[it0 - 1]], seg1
                    )
                if it1 + 1 < len(on_line):
                    self._maybe_push_cross(
                        seg0, id_to_segments[on_line[it1 + 1]]
                    )
        return self.cross_pairs
