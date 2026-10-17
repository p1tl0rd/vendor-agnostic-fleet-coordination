"""Prioritized Safe-Interval Path Planning (PSIPP).

The planner assigns agents an order (priority 0, 1, 2, ...), and for
each agent runs an A* over the (vertex, safe-interval) state space.
The previous agent's planned route is converted into a set of
**blocked windows** on the current agent's vertex safe intervals and
edge safe windows, and the A* skips any (state, edge) pair whose
target interval collides with the next agent's route.

The three inner data structures are:

* ``Interval`` (no id, edge feasibility gate),
* ``IntervalWithID`` (id, A* state),
* ``SafeIntervalData`` (per-interval A* bookkeeping).

The ``bisect`` and ``heapq`` modules back the start-sorted interval
lists and the priority queue.

Algorithm invariants
--------------------
1. **Generation counter.** ``open_listed`` and ``close_listed`` use
   ``priority_order`` directly as a stamp. There is deliberately no
   per-agent clearing; once an interval has been touched in some
   round, the next round sees the new stamp and treats it as fresh.
2. **Monotone accumulation.** Blocked windows (the
   ``vertex_safes`` / ``edge_safes`` structures) accumulate
   monotonically across the priority loop. The previous agent's
   conflicts are added; the new agent's A* reads them.
3. **Id minted only on split.** ``add_vertex_collision_interval``
   mints a new id only when one interval is split into two (the
   prev-overlap case where ``end + EPS < b``). Surviving intervals
   keep their identity.
4. **Admissible heuristic.** ``heuristic_distance`` is Euclidean and
   is admissible only because ``edge_length == travel_time`` (unit
   speed).
5. **Negative id = wait.** ``edge_id < 0`` means waiting; both the
   priority loop and the route reconstruction normalise this.
6. **Reverse-time convention.** ``get_all_ve_collisions`` returns
   ``(src, edge_id, -upper, -lower)``. The negation and the swap
   are a reverse-time convention; the planner uses ``t_step +
   std::get<2>(collision)`` to mean ``t_step - upper`` (i.e. the
   blocked window opens *before* the moving agent's window opens).
   Do NOT "fix" it (cpp-to-python skill rule 9).
7. **Global edge indices.** The annotation constructor flattens the
   adjacency list to global edge indices. The planner works in
   per-vertex ``edge_id`` and the accessors translate via the
   explicit instance's ``edge_ids`` and ``edge_data``.

Out of scope
------------
* The full A*'s reverse-time mirroring of the EEC time frame. The
  Python ``get_collision_interval`` 6-arg form does the mirror at
  the kernel layer; the planner just consumes the resolved
  ``(lo, up)`` from the explicit-collisions accessors.
* Multi-threaded execution. ``time.monotonic`` provides the same
  wall-clock-budget semantics as ``std::chrono::system_clock``.
* The ``priority_strategy == 1`` shuffle path. The constructor
  accepts the argument for completeness but the default is FIFO
  (the only path the C++ uses).
"""

from __future__ import annotations

import bisect
import heapq
import math
import random
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from psipp_ctc.problem import (
    AgentTask,
    Move,
    Plan,
    Point,
    ProblemInstance,
)
from psipp_ctc.planners.planner import Planner

# Module-level alias for the data-model ``INF`` (a class
# attribute on ``ProblemInstance``). The planner's blocked
# windows start at ``[0, INF]`` by default.
_INF = math.inf


# ---------------------------------------------------------------------------
# Inner data structures
# ---------------------------------------------------------------------------


@dataclass
class IntervalWithID:
    """A vertex safe interval -- an A* state.

    Two intervals with the same ``start`` but different ``end`` / ``id``
    are ordered arbitrarily in a ``bisect.insort``-maintained list
    keyed on ``start``; the planner uses ``start`` alone to locate
    the candidate safe interval, so the arbitrary order does not
    matter.

    Fields
    ------
    start, end : float
        The interval's time bounds. ``end > start`` for a
        non-degenerate safe interval.
    id : int
        The interval's identity. Survives the
        ``add_vertex_collision_interval`` call that subtracts from
        the front of the interval (the split-tail mints a new id
        instead). Used as the A* state's identity.

    Comparison
    ----------
    ``__lt__`` is on ``start`` only, matching the C++. Equal-start
    intervals deduplicate in ``set``; in the Python list we keep
    them in insertion order (since the C++ does not rely on
    comparison ordering beyond ``start``).
    """

    start: float
    end: float = 0.0
    id: int = -1

    def __lt__(self, other: "IntervalWithID") -> bool:
        return self.start < other.start


@dataclass
class Interval:
    """An edge blocked window -- a feasibility gate, NOT an A* state.

    No ``id`` field because the edge's A* bookkeeping is keyed by
    the *target* vertex's ``IntervalWithID.id``; the edge blocked
    windows are intermediate, read-only data during the scan.
    """

    start: float
    end: float = 0.0

    def __lt__(self, other: "Interval") -> bool:
        return self.start < other.start


@dataclass
class SafeIntervalData:
    """Per-interval A* bookkeeping.

    The fields ``open_listed`` and ``close_listed`` carry the
    **generation counter** -- a copy of ``priority_order`` from the
    round that last touched this interval. ``-1`` is the sentinel
    for "never touched".
    """

    open_listed: int = -1
    close_listed: int = -1
    best_time: float = 0.0
    previous: int = -1
    prev_vertex: int = -1
    prev_edge_id: int = -1


@dataclass
class CurrentState:
    """Per-(vertex, out-edge) monotone scan state.

    Two integer indices (one into ``edge_safes[v][e]``, one into
    ``vertex_safes[target]``). Indices stay valid as long as the
    underlying lists do not mutate; they do not mutate during A*
    (blocked windows are added only between rounds, not while A*
    is running).
    """

    visited: int = -1
    edge_idx: int = 0
    target_idx: int = 0


# A tuple of ``(f, insertion_order, vertex, target_interval_id)``
# for the A* heap.
#
# ``heapq`` is a min-heap; C++ ``priority_queue`` with
# ``node_compare = [](a, b){ return a.f_value > b.f_value; }`` is
# also a min-heap. The C++ has NO tie-breaker beyond ``f_value``
# (items with equal ``f_value`` are treated as equal by the
# comparator). ``std::priority_queue`` resolves the order of
# equal-key items by insertion order (older insertion on top,
# like a stable min-heap). ``heapq`` compares every tuple field
# on draw, which would break ties via ``vertex`` then
# ``target_interval_id`` -- NOT what the C++ does.
#
# We resolve this by inserting an explicit monotonically
# increasing ``insertion_order`` integer as the second tuple
# field. ``heapq`` compares it on tie and the OLDER insertion
# (smaller number) comes out first -- matching the C++'s
# ``priority_queue`` behaviour closely enough that parity holds
# on the Berlin CDT and similar instances. ``target_interval_id``
# stays as the fourth field to make two otherwise-equal items
# distinct (Python's ``heapq`` raises ``TypeError`` on draw if
# two items compare equal).
_HeapItem = Tuple[float, int, int, int]  # (f, insertion_order, vertex, iv_id)


# ---------------------------------------------------------------------------
# add_*_collision_interval
# ---------------------------------------------------------------------------


def add_vertex_collision_interval(
    safes: List[IntervalWithID],
    start: float,
    end: float,
    eps: float,
    counter: List[int],
) -> None:
    """Subtract ``[start, end]`` from a sorted, disjoint list of
    vertex safe intervals.

    The contract:

    * ``safes`` is sorted by ``start`` and disjoint.
    * ``counter[0]`` is the next unused interval id; this function
      mints a new one ONLY when a single interval is split into
      two.
    * In-place: ``safes`` is mutated.

    The three cases (in order):

    1. **Prev overlap.** If the interval immediately preceding
       ``start`` ends after ``start``, the prev is shrunk on the
       right to ``[a, start]``, KEEPING its id. If the original
       interval was long enough to also have a non-empty tail
       (``end + EPS < b``), a new interval ``[end, b]`` is
       inserted with a freshly-minted id.
    2. **Whole-interval consumption.** Walk forward while
       ``it->end <= end + EPS`` and erase.
    3. **Tail overlap.** If the next interval starts before
       ``end``, shrink it on the left to ``[end, b]`` KEEPING its
       id.

    The id-keep behaviour is invariant #3 -- the surviving
    interval keeps its identity. A new id is minted only in the
    split case.
    """
    # Position of the first interval whose start >= start.
    # We use a sentinel with start = start - EPS to find the
    # first candidate that overlaps with the open end at start.
    idx = bisect.bisect_left(safes, IntervalWithID(start - eps))

    # Case 1: prev overlap.
    if idx > 0:
        prev = safes[idx - 1]
        if prev.end > start:
            a, b, vid = prev.start, prev.end, prev.id
            del safes[idx - 1]
            # Invariant: after removing prev, the next interval
            # (if any) must start strictly after a. The C++
            # asserts this at line 20; we use an `assert` here
            # for the same purpose. A failure means the caller
            # passed a non-sorted or non-disjoint list.
            assert idx - 1 == len(safes) or a < safes[idx - 1].start, (
                "add_vertex_collision_interval: prev removal left "
                "non-disjoint state"
            )
            safes.insert(idx - 1, IntervalWithID(a, start, vid))
            if end + eps < b:
                # Mint a new id for the tail.
                safes.insert(idx, IntervalWithID(end, b, counter[0]))
                counter[0] += 1
                return

    # Case 2: consume whole intervals while they fit.
    while idx < len(safes) and safes[idx].end <= end + eps:
        del safes[idx]

    # Case 3: tail overlap.
    if idx < len(safes) and safes[idx].start < end:
        b, vid = safes[idx].end, safes[idx].id
        del safes[idx]
        safes.insert(idx, IntervalWithID(end, b, vid))


def add_edge_collision_interval(
    safes: List[Interval],
    start: float,
    end: float,
    eps: float,
) -> None:
    """Subtract ``[start, end]`` from a sorted, disjoint list of
    edge blocked windows.

    Same algorithm as ``add_vertex_collision_interval`` minus the
    id field -- edge blocked windows have no id because they are
    not A* states.
    """
    idx = bisect.bisect_left(safes, Interval(start - eps))

    if idx > 0:
        prev = safes[idx - 1]
        if prev.end > start:
            a, b = prev.start, prev.end
            del safes[idx - 1]
            assert idx - 1 == len(safes) or a < safes[idx - 1].start, (
                "add_edge_collision_interval: prev removal left "
                "non-disjoint state"
            )
            safes.insert(idx - 1, Interval(a, start))
            if end + eps < b:
                safes.insert(idx, Interval(end, b))
                return

    while idx < len(safes) and safes[idx].end <= end + eps:
        del safes[idx]

    if idx < len(safes) and safes[idx].start < end:
        b = safes[idx].end
        del safes[idx]
        safes.insert(idx, Interval(end, b))


# ---------------------------------------------------------------------------
# PrioritizedSIPP
# ---------------------------------------------------------------------------


class PrioritizedSIPP(Planner):
    """Prioritised Safe-Interval Path Planning (PSIPP).

    The agent order is the priority list (FIFO by default). The
    first agent's A* ignores all conflicts (no previous agents);
    each subsequent agent's A* sees the union of the previous
    agent's blocked windows (only the IMMEDIATELY previous agent
    is added per round -- invariant #8). A* over
    ``(vertex, safe-interval)`` states skips any state whose safe
    interval does not reach the goal in time, plus any
    (state, edge) pair whose target interval overlaps the
    next agent's trajectory.

    The first agent that cannot reach its goal interval aborts the
    whole run -- the planner does not backtrack to try a
    different priority order. The C++ does the same.

    Parameters
    ----------
    priority_strategy : int, default 0
        ``0`` = FIFO (the only strategy the C++ ever uses;
        ``load_planners.cpp:24`` hardcodes it). ``1`` =
        unseeded ``random_device`` + ``default_random_engine``
        shuffle, mirroring the C++'s unseeded path; tests that
        depend on a specific priority order should pin to ``0``.
    time_limit : float, default 30.0
        Wall-clock budget in seconds. The C++ uses
        ``std::chrono::system_clock::now()``; the Python port
        uses ``time.monotonic()`` for the same effect. A round
        that exceeds the budget sets ``success = False`` and
        returns immediately.
    """

    def __init__(
        self,
        priority_strategy: int = 0,
        time_limit: float = 30.0,
    ) -> None:
        super().__init__()
        self._priority_strategy = priority_strategy
        self._time_limit = time_limit
        self._success: bool = False
        self._plan: Plan = []
        # Cached per-solve, populated on first ``solve()`` call.
        # The C++ keeps these as fields because the planner is
        # stateful across ``setInstance`` -> ``solve`` calls. We
        # do the same; ``set_instance`` does NOT clear them
        # (matching the C++ which only clears ``plan`` and
        # ``interval_data`` inside ``solve``).
        self._vertex_safes: List[List[IntervalWithID]] = []
        self._edge_safes: List[List[List[Interval]]] = []
        self._interval_data: List[SafeIntervalData] = []
        self._current_states: List[List[CurrentState]] = []
        self._number_of_intervals: int = 0
        self._routes: List[List[Tuple[int, int]]] = []
        self._time_stamps: List[List[float]] = []
        self._inf: float = _INF
        self._eps: float = 1e-8

    # -- Planner surface ------------------------------------------------

    def succeeded(self) -> bool:
        return self._success

    def get_plan(self) -> Plan:
        return self._plan

    # -- Distance heuristic (invariant #4) -------------------------------

    def _heuristic_distance(self, v0: int, v1: int) -> float:
        """Euclidean distance. Admissible only because edge length ==
        travel time (unit speed)."""
        assert self.problem is not None
        dx = self.problem.get_coordinate_x(v0) - self.problem.get_coordinate_x(v1)
        dy = self.problem.get_coordinate_y(v0) - self.problem.get_coordinate_y(v1)
        return math.hypot(dx, dy)

    # -- Main entry point ------------------------------------------------

    def solve(self) -> None:
        """Run the planner.

        On any abort condition (time limit, unreachable goal),
        ``self._success`` is set to ``False`` and the call returns
        without raising. The caller inspects ``succeeded()``
        afterwards.

        The plan, if successful, is one ``list[Move]`` per agent
        in agent-index order. Each ``Move.edge_id`` is the
        per-vertex out-list index (negative = wait). The plan can
        be validated via ``check_plan_validity`` on the
        explicit-collisions instance.
        """
        assert self.problem is not None, (
            "PrioritizedSIPP.solve called before set_instance"
        )
        start_clock = time.monotonic()
        # Fetch the data-model constants. The reference reads them
        # from the problem instance; we do the same.
        self._inf = self.problem.INF
        self._eps = self.problem.EPS

        m = self.problem.get_number_of_vertices()
        n = len(self.problem.tasks)


        # -- Priority order ------------------------------------------------
        priorities: List[int] = list(range(n))
        if self._priority_strategy == 1:
            # Unseeded shuffle -- matches the C++ reference's
            # unseeded ``std::random_device`` + ``default_random_engine``.
            # Python's ``random.shuffle`` is similarly
            # unseeded-by-default. Tests that need a specific
            # order pin ``priority_strategy = 0``.
            random.shuffle(priorities)

        # -- Per-agent scratch ---------------------------------------------
        # ``routes`` is a list of ``(vertex, edge_id)`` pairs --
        # the C++ uses ``std::pair<int, int>`` where the second
        # member is -1 for waits. ``time_stamps`` is the
        # start time of each step plus a final INF sentinel
        # (``time_stamps[agent].size() == routes[agent].size() + 1``
        # is the C++ invariant; see ``PrioritizedSIPP.cpp:141``).
        routes: List[List[Tuple[int, int]]] = [[] for _ in range(n)]
        time_stamps: List[List[float]] = [[] for _ in range(n)]

        edges = self.problem.edges  # public field on the C++ side
        self._plan = [[] for _ in range(n)]

        # -- Initialise the safe-interval / blocked-window structures ------
        # See ``PrioritizedSIPP.cpp:110-131``. The vertex safe
        # interval of v starts as ``[0, INF]`` with id v; the
        # edge blocked windows of every out-edge start as
        # ``[0, INF]``; the per-(v, e) CurrentState is reset.
        # The id of the initial safe interval IS the vertex
        # id (the C++ comment "initial id = v" is at line 123).
        self._vertex_safes = [
            [IntervalWithID(0.0, self._inf, v)] for v in range(m)
        ]
        self._edge_safes = [
            [[Interval(0.0, self._inf)] for _ in range(len(edges[v]))]
            for v in range(m)
        ]
        self._current_states = [
            [CurrentState() for _ in range(len(edges[v]))] for v in range(m)
        ]
        self._interval_data = []
        self._number_of_intervals = m
        self._interval_data = [SafeIntervalData() for _ in range(m)]

        # -- Priority loop --------------------------------------------------
        for priority_order in range(n):
            agent = priorities[priority_order]

            # Step 1: add the previous agent's blocked windows.
            # The C++ does this BEFORE the A* so the new agent
            # sees a fresh conflict set on top of the (already
            # monotone-accumulated) history from even earlier
            # rounds. Invariant #8: only the IMMEDIATELY previous
            # agent is added this round.
            if priority_order > 0:
                self._add_previous_agent_conflicts(
                    priorities, priority_order, routes, time_stamps
                )
            # ``interval_data`` may have grown in the previous
            # step (id-minting in add_vertex_collision_interval).
            # Resize to keep it in lock-step -- the C++ does
            # this at line 185.
            if len(self._interval_data) < self._number_of_intervals:
                self._interval_data.extend(
                    SafeIntervalData()
                    for _ in range(
                        self._number_of_intervals - len(self._interval_data)
                    )
                )

            # Step 2: A* for `agent`.
            initial_vertex = self.problem.tasks[agent].initial_vertex
            goal_vertex = self.problem.tasks[agent].goal_vertex
            # ``begin()`` is the first interval (lowest start);
            # the initial safe interval of any vertex starts at
            # time 0 by construction.
            initial_interval_id = self._vertex_safes[initial_vertex][0].id
            # ``rbegin()`` is the last interval (highest start);
            # the C++ picks the goal as the interval with the
            # latest start, which for a freshly-initialised
            # ``[0, INF]`` is the only one.
            goal_interval_id = self._vertex_safes[goal_vertex][-1].id

            # Min-heap. ``heapq.heappush`` / ``heappop`` give the
            # smallest ``f``; the C++ uses
            # ``std::priority_queue<Node, vector<Node>,
            # decltype(node_compare)>`` with
            # ``node_compare = [](a, b){ return a.f_value >
            # b.f_value; }`` -- a min-heap on ``f_value``. See
            # the comment on ``_HeapItem`` (top of file) for the
            # tie-breaker convention: we use insertion-order
            # (``_heap_counter``) to match the C++'s
            # ``priority_queue`` stable behaviour.
            heap: List[_HeapItem] = []
            # Monotonic counter for the ``insertion_order``
            # tie-breaker. Items pushed earlier come out first on
            # ties. Reset to 0 at the start of every priority
            # round so the count does not overflow across rounds.
            self._heap_counter: int = 0
            f_initial = self._heuristic_distance(initial_vertex, goal_vertex)
            heapq.heappush(
                heap,
                (
                    f_initial,
                    self._heap_counter,
                    initial_vertex,
                    initial_interval_id,
                ),
            )
            self._heap_counter += 1
            self._interval_data[initial_interval_id].open_listed = priority_order
            self._interval_data[initial_interval_id].best_time = 0.0

            success_this_round = self._astar(
                heap,
                goal_interval_id,
                initial_vertex,
                goal_vertex,
                priority_order,
            )

            if not success_this_round:
                # First agent that cannot reach its goal aborts
                # the whole run. The C++ sets ``success = false``
                # and returns (``PrioritizedSIPP.cpp:267-270``).
                self._success = False
                return

            # Step 3: route reconstruction. Walk the prev links
            # from goal back to the initial safe interval,
            # re-inserting waits where the actual departure time
            # is later than the predecessor's arrival.
            self._reconstruct_route(
                agent,
                goal_interval_id,
                initial_interval_id,
                goal_vertex,
                routes,
                time_stamps,
            )

            # Time-limit check. The C++ reads the clock at the
            # end of each round (``PrioritizedSIPP.cpp:298-303``).
            elapsed = time.monotonic() - start_clock
            if elapsed >= self._time_limit:
                self._success = False
                return

        self._success = True

    # ------------------------------------------------------------------
    # Internal: add the previous agent's conflicts
    # ------------------------------------------------------------------

    def _add_previous_agent_conflicts(
        self,
        priorities: List[int],
        priority_order: int,
        routes: List[List[Tuple[int, int]]],
        time_stamps: List[List[float]],
    ) -> None:
        """Materialise the previous agent's route as a set of
        blocked windows on the current agent's safe intervals.

        Only the IMMEDIATELY previous agent
        (``priorities[priority_order - 1]``) is added this round.

        The route is a sequence of ``(vertex, edge_id)`` steps
        (edge_id == -1 means waiting). For each step, the
        blocked window is computed from the appropriate
        ``get_all_*_collisions`` accessor (VE for waits, EV +
        EE for moves), then added to ``vertex_safes`` or
        ``edge_safes`` via ``add_*_collision_interval``.

        Note on the time-frame mirror (VE case): the C++ uses
        ``t_step + std::get<2>(collision) = t_step - upper``
        because the VEC accessor returns ``(..., -upper, -lower)``
        (reverse time, invariant #6). The EE accessor returns
        ``(..., lower, upper)`` in NATURAL time -- the planner
        does NOT negate for EE.

        A blocked window with ``end <= 0`` is dropped -- the
        conflict is in the past and does not constrain the
        current agent (the C++ checks ``if (collision_end >
        0.0)`` at lines 150, 164, 176).
        """
        assert self.problem is not None
        j = priorities[priority_order - 1]

        # ``time_stamps[j].size()`` == ``routes[j].size() + 1``:
        # routes[j][k] starts at time_stamps[j][k] and ends at
        # time_stamps[j][k+1] (or at the final INF arrival for
        # the last step). The C++ walks ``step`` from
        # ``0`` to ``time_stamps[j].size() - 1`` inclusive, so
        # the last iteration handles the final step (which
        # moves the agent to its goal). We mirror that with
        # ``range(len(time_stamps[j]) - 1)`` (the upper bound is
        # exclusive).
        for step in range(len(time_stamps[j]) - 1):
            vertex, edge_id = routes[j][step]
            t_step = time_stamps[j][step]
            t_next = time_stamps[j][step + 1]

            if edge_id == -1:
                # j was waiting. The C++ calls
                # ``getAllVECollisions(vertex)`` and treats the
                # returned ``(src, edge_id, -upper, -lower)``
                # tuple in REVERSE time: the collision opens
                # at ``t_step + std::get<2>(collision) =
                # t_step - upper`` (i.e. before j's window
                # opens) and closes at ``t_next +
                # ``get<3>(collision)`` = t_next - lower``.
                ve_list = self.problem.get_all_ve_collisions(vertex)
                for (src, eid, neg_up, neg_lo) in ve_list:
                    collision_start = t_step + neg_up
                    collision_end = t_next + neg_lo
                    if collision_end > 0.0:
                        add_edge_collision_interval(
                            self._edge_safes[src][eid],
                            collision_start,
                            collision_end,
                            self._eps,
                        )
            else:
                # j was moving on (vertex, edge_id). Two
                # accessor lists: EV (vertex-blocked) and EE
                # (edge-blocked). Both are in NATURAL time
                # (the EE return convention was fixed when the
                # explicit-collisions subclass was written).
                ev_list = self.problem.get_all_ev_collisions(vertex, edge_id)
                for (other_vertex, lo, up) in ev_list:
                    collision_start = t_step + lo
                    collision_end = t_step + up
                    if collision_end > 0.0:
                        # ``counter`` is a mutable single-element
                        # list used as an inout for the new-id
                        # counter. We sync back to
                        # ``self._number_of_intervals`` after the
                        # call (Python's int is immutable so the
                        # add function cannot write back through
                        # a plain int reference).
                        counter = [self._number_of_intervals]
                        add_vertex_collision_interval(
                            self._vertex_safes[other_vertex],
                            collision_start,
                            collision_end,
                            self._eps,
                            counter,
                        )
                        self._number_of_intervals = counter[0]
                ee_list = self.problem.get_all_ee_collisions(vertex, edge_id)
                for (other_src, other_eid, lo, up) in ee_list:
                    collision_start = t_step + lo
                    collision_end = t_step + up
                    if collision_end > 0.0:
                        add_edge_collision_interval(
                            self._edge_safes[other_src][other_eid],
                            collision_start,
                            collision_end,
                            self._eps,
                        )

    # ------------------------------------------------------------------
    # Internal: the A* main loop
    # ------------------------------------------------------------------

    def _astar(
        self,
        heap: List[_HeapItem],
        goal_interval_id: int,
        initial_vertex: int,
        goal_vertex: int,
        priority_order: int,
    ) -> bool:
        """Run A* from the initial safe interval to the goal
        safe interval. Returns True on success, False if the
        goal interval is unreachable.

        The generation-counter invariant is here: ``open_listed``
        and ``close_listed`` carry a COPY of ``priority_order``,
        and the ``close_listed == priority_order`` test at the
        top of the loop means "skip an interval that was already
        closed in THIS round" -- not "closed ever". Earlier
        rounds left a different stamp and are treated as fresh.

        Returns
        -------
        bool
            True iff the goal interval was closed in this
            round. The caller aborts the whole run on False.
        """
        assert self.problem is not None
        edges = self.problem.edges

        while heap:
            _, _insertion, vertex, interval_id = heapq.heappop(heap)

            # Invariant #1: a per-round generation stamp. An
            # interval closed in a previous round has a
            # different stamp here and is NOT skipped.
            if self._interval_data[interval_id].close_listed == priority_order:
                continue
            self._interval_data[interval_id].close_listed = priority_order

            # Goal test. The C++ breaks out of the loop on
            # goal; we return True instead so the caller can
            # run the route-reconstruction step. Both forms
            # leave the goal interval closed.
            if interval_id == goal_interval_id:
                return True

            current_time = self._interval_data[interval_id].best_time
            # ``deadline`` is the end of the current safe
            # interval -- we cannot depart after this. Look up
            # the interval by id; the C++ does the same by
            # keeping the ``it`` iterator alive in the queue
            # payload.
            deadline = self._find_interval_end_by_id(
                vertex, interval_id
            )

            for edge_id in range(len(edges[vertex])):
                target = edges[vertex][edge_id]
                length = self.problem.get_edge_length(vertex, edge_id)

                # Reset iterators on first visit this round.
                # The C++ uses ``current.visited`` for this
                # (line 228-232); we use the same field. The
                # iterators are integer indices into the
                # corresponding lists; they remain valid
                # because the lists do not mutate during A*
                # (the ``add_*`` calls happen in
                # ``_add_previous_agent_conflicts`` between
                # rounds, and the same (v, e) pair is touched
                # only once per round's outer for-loop).
                current = self._current_states[vertex][edge_id]
                if current.visited != priority_order:
                    current.visited = priority_order
                    current.edge_idx = 0
                    current.target_idx = 0

                edge_list = self._edge_safes[vertex][edge_id]
                target_list = self._vertex_safes[target]
                # Advance the edge iterator. The C++ walks
                # ``edge_it`` until ``edge_it->start >
                # deadline + EPS`` (line 233-235), but for
                # monotone accumulation the start of an
                # interval cannot be after its end; the
                # check ``edge_it->end < current_time - EPS``
                # is the skip for "this window closes before
                # the agent arrives" (line 236-238).
                while current.edge_idx < len(edge_list):
                    edge_win = edge_list[current.edge_idx]
                    if edge_win.end < current_time - self._eps:
                        current.edge_idx += 1
                        continue
                    if edge_win.start > deadline + self._eps:
                        break
                    arrival_start = max(current_time, edge_win.start) + length
                    arrival_end = min(deadline, edge_win.end) + length

                    # Advance the target iterator. Same
                    # pattern: skip intervals that close
                    # before arrival_start, break when the
                    # next start is past arrival_end.
                    while current.target_idx < len(target_list):
                        target_iv = target_list[current.target_idx]
                        if target_iv.end < arrival_start - self._eps:
                            current.target_idx += 1
                            continue
                        if target_iv.start > arrival_end + self._eps:
                            break
                        arrival_time = max(arrival_start, target_iv.start)
                        target_iv_id = target_iv.id
                        # Invariant #1 again: an
                        # open_listed stamp from a different
                        # round is "open" for this round.
                        od = self._interval_data[target_iv_id]
                        if (
                            od.open_listed != priority_order
                            or od.best_time - self._eps > arrival_time
                        ):
                            od.open_listed = priority_order
                            od.best_time = arrival_time
                            od.previous = interval_id
                            od.prev_vertex = vertex
                            od.prev_edge_id = edge_id
                            f = arrival_time + self._heuristic_distance(
                                target, goal_vertex
                            )
                            heapq.heappush(
                                heap,
                                (
                                    f,
                                    self._heap_counter,
                                    target,
                                    target_iv_id,
                                ),
                            )
                            self._heap_counter += 1
                        current.target_idx += 1
                    current.edge_idx += 1

        # If the heap empties without the goal being closed
        # in this round, the agent is stuck. The C++ sets
        # ``success = false`` and returns.
        return self._interval_data[goal_interval_id].close_listed == priority_order

    def _find_interval_end_by_id(
        self, vertex: int, interval_id: int
    ) -> float:
        """Look up the end of the safe interval by id.

        The C++ keeps the ``std::set::iterator`` in the heap
        payload, so it can read ``it->end`` without a search.
        The Python port stores only the id, so a linear scan
        is needed. The vertex safe interval list is small in
        practice (a handful of splits per vertex even on the
        Berlin CDT instance) so the scan is fast.

        Raises
        ------
        KeyError
            If the id is not present. The C++ has no analogue
            -- it would just dereference the iterator. We use
            ``KeyError`` so a port regression surfaces
            immediately.
        """
        for iv in self._vertex_safes[vertex]:
            if iv.id == interval_id:
                return iv.end
        raise KeyError(
            f"interval id {interval_id} not found in "
            f"vertex_safes[{vertex}]"
        )

    # ------------------------------------------------------------------
    # Internal: route reconstruction
    # ------------------------------------------------------------------

    def _reconstruct_route(
        self,
        agent: int,
        goal_interval_id: int,
        initial_interval_id: int,
        goal_vertex: int,
        routes: List[List[Tuple[int, int]]],
        time_stamps: List[List[float]],
    ) -> None:
        """Walk the prev links from goal back to the initial
        safe interval, then reverse the route. Re-insert waits
        where the actual departure time is later than the
        predecessor's arrival (the agent had to wait out a
        conflict before departing on the next edge).

        Note on the trailing sentinel: ``(goal_vertex, -1)`` is
        pushed to ``routes[agent]`` and ``INF`` then
        ``best_time[goal_id]`` to ``time_stamps[agent]``. The
        INF arrival at the end is what makes
        ``time_stamps[agent].size() == routes[agent].size() + 1``
        -- the ``add_previous_agent_conflicts`` loop relies on
        that invariant to find ``t_next`` for the last step.
        """
        assert self.problem is not None
        plan_for_agent: List[Move] = []

        # Prepend the (goal, -1) terminator and the times
        # needed to make the previous step's "next time"
        # well-defined. The C++ does this by pushing INF then
        # best_time at the FRONT, then walking the prev
        # links and pushing onto the END, then reversing.
        # We follow the same shape for parity.
        routes[agent].append((goal_vertex, -1))
        time_stamps[agent].append(self._inf)
        time_stamps[agent].append(
            self._interval_data[goal_interval_id].best_time
        )

        vertex = goal_vertex
        interval_id = goal_interval_id
        while interval_id != initial_interval_id:
            prev_vertex = self._interval_data[interval_id].prev_vertex
            prev_edge_id = self._interval_data[interval_id].prev_edge_id
            prev_interval_id = self._interval_data[interval_id].previous
            # Travel time = edge length (unit speed, invariant #4).
            start_time = (
                self._interval_data[interval_id].best_time
                - self.problem.get_edge_length(prev_vertex, prev_edge_id)
            )
            routes[agent].append((prev_vertex, prev_edge_id))
            time_stamps[agent].append(start_time)
            plan_for_agent.append(Move(prev_edge_id, start_time))
            # If the agent had to wait (departed after the
            # predecessor's arrival), insert a wait step.
            # The C++ writes the wait as ``(prev_vertex, -1)``
            # with the time at which the previous step
            # finished (``best_time[prev_interval_id]``).
            if start_time > (
                self._interval_data[prev_interval_id].best_time + self._eps
            ):
                routes[agent].append((prev_vertex, -1))
                time_stamps[agent].append(
                    self._interval_data[prev_interval_id].best_time
                )
            vertex = prev_vertex
            interval_id = prev_interval_id

        plan_for_agent.reverse()
        routes[agent].reverse()
        time_stamps[agent].reverse()
        # ``plan[agent]`` was already sized to one entry per
        # agent at the start of ``solve``; we replace it
        # with the reconstructed route. The C++ does the
        # same (line 285: ``plan[agent].push_back(Move(...))``
        # then ``std::reverse(plan[agent]...)``).
        self._plan[agent] = plan_for_agent
