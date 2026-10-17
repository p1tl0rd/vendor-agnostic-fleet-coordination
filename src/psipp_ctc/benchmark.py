"""Stage 5-follow-up -- planner benchmark harness.

Mirrors
  PSIPP-CTC_src/src/base/benchmark.hpp  (433 lines)

The reference exposes two orchestrators: ``RunPlanner`` (a single planning
run, with metrics) and ``Benchmark`` (a registry of problem instances +
planners + a runner that iterates over their cross product and aggregates).
This port carries the non-spatial subset -- ``RoadmapBenchmarkAll`` and
``RunSpatialPlanner`` are out of scope until the spatial planners themselves
are ported (HANDOFF §11 item 4, ``Decide the CCBS baseline strategy``).

Metrics follow the C++ conventions:

* ``used_time`` is the **total** wall-clock time of the solve, in
  milliseconds. The Python ``time.monotonic`` analogue measures seconds,
  converted at the read-out boundary.
* ``used_time_for_collisions`` is the cumulative ``problem.used_time``
  (seconds) accumulated by the collision-timing re-entrant timer during
  the solve, converted to milliseconds.
* ``ptime`` = ``used_time - used_time_for_collisions`` (planning time).
* ``ctime`` = ``used_time_for_collisions`` (collision time).
* Aggregation over runs uses **mean over successful runs only**; the
  per-row ``success`` flag is a float in ``[0, 1]`` (the success
  fraction), matching ``displayAllResults``'s ``(double)success_count /
  iterations``.

Failure handling mirrors the C++: a solver that throws or returns
``succeeded() == False`` or fails ``check_plan_validity`` sets
``success = False`` and zeroes its metrics, so the per-iteration mean is
not poisoned.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

from psipp_ctc.planners.planner import Planner
from psipp_ctc.problem import Plan, ProblemInstance


def get_used_memory_kb() -> float:
    """RSS in KB from ``/proc/self/status``.

    Mirrors ``base/memory_usage.hpp::getUsedMemory``: reads the 24th
    field (``vsize``) and the 25th (``rss``) of ``/proc/self/stat`` on
    POSIX systems. Returns ``0.0`` on non-POSIX (the C++ emits a warning
    and returns ``0``).

    NOTE: the C++ actually returns ``rss * page_size / 1024`` (i.e. RSS in
    KB). The implementation here mirrors that. The C++ ignores the
    ``vsize`` field.
    """
    if not (hasattr(os, "POSIX_VERSION") or hasattr(os, "POSIX2_VERSION") or os.name == "posix"):
        return 0.0
    try:
        with open("/proc/self/status", "r") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    # ``VmRSS:   12345 kB`` -- report the kilobyte value
                    # directly. /proc/self/status always reports
                    # pages-rounded-up to KB regardless of the system
                    # page size, while the C++ reads raw pages from
                    # /proc/self/stat and multiplies by sysconf page
                    # size. The two agree on Linux for whole-page
                    # counts, which is all the benchmark uses.
                    parts = line.split()
                    return float(parts[1])
        return 0.0
    except OSError:
        return 0.0


def _ms(seconds: float) -> int:
    """Convert a seconds value to int milliseconds (rounded down).

    Matches the C++'s ``std::chrono::duration_cast<std::chrono::milliseconds>``
    which truncates toward zero for positive durations.
    """
    return int(seconds * 1000.0)


@dataclass
class RunPlanner:
    """Single planning-run metrics.

    Constructing the dataclass runs the planner immediately (mirroring the
    C++ constructor side-effecting on the planner). On failure the
    ``success`` flag is ``False`` and the metric fields are zero.

    Attributes
    ----------
    success : bool
        ``True`` iff the planner produced a plan that ``check_plan_validity``
        accepted.
    used_time : int
        Wall-clock milliseconds for the solve. Python ``time.monotonic``
        seconds scaled by 1000.
    used_time_for_collisions : int
        Cumulative ``problem_instance.used_time`` (in seconds) accrued during
        the solve, scaled to milliseconds.
    used_memory : float
        Peak RSS in KB, from ``get_used_memory_kb()``. Called at plan-finish
        time, matching the C++'s single-sample convention.
    makespan : float
        Per-plan makespan (``-1.0`` is the C++'s failure sentinel; we mirror it
        here so the aggregator's mean-over-successful treats runs with that
        value as failed).
    sum_of_costs : float
        Per-plan sum-of-costs (sum of per-agent final times).
    number_of_CC : int
        Collision-check count delta for the run.
    number_of_IC : int
        Interval-calculation count delta for the run.
    """

    success: bool = False
    used_time: int = 0
    used_time_for_collisions: int = 0
    used_memory: float = 0.0
    makespan: float = -1.0
    sum_of_costs: float = -1.0
    number_of_CC: int = 0
    number_of_IC: int = 0

    def __init__(
        self,
        planner: Planner,
        problem_instance: ProblemInstance,
    ) -> None:
        # Default values for the failure path. Constructor may overwrite.
        self.success = False
        self.used_time = 0
        self.used_time_for_collisions = 0
        self.used_memory = 0.0
        self.makespan = -1.0
        self.sum_of_costs = -1.0
        self.number_of_CC = 0
        self.number_of_IC = 0

        # Cache the starting counters (mirrors lines 33-36 of the C++).
        current_used_time_for_collisions = problem_instance.used_time
        current_number_of_CC = problem_instance.number_of_collision_checks
        current_number_of_IC = problem_instance.number_of_interval_calculations

        # ``try`` block mirrors ``lines 37-63``.
        try:
            start_time = time.monotonic()
            planner.solve()
        except RuntimeError as e:
            # The C++ writes to stderr via ``fprintf(stderr, ...)`` for
            # thrown errors; we mirror the *behaviour* (return failure)
            # but stay silent so users can decide verbosity.
            _ = e  # signal unused-binding-justification
            return

        elapsed_s = time.monotonic() - start_time
        self.used_time = _ms(elapsed_s)
        self.used_time_for_collisions = _ms(
            problem_instance.used_time - current_used_time_for_collisions
        )
        self.used_memory = get_used_memory_kb()
        self.number_of_CC = (
            problem_instance.number_of_collision_checks - current_number_of_CC
        )
        self.number_of_IC = (
            problem_instance.number_of_interval_calculations
            - current_number_of_IC
        )

        if not planner.succeeded():
            return

        plan: Plan = planner.get_plan()
        ms, soc = problem_instance.check_plan_validity(plan)
        self.makespan = ms
        self.sum_of_costs = soc
        self.success = True


@dataclass
class BenchmarkResult:
    """Aggregated result row for one ``(problem, planner)`` pair.

    Mirrors ``Benchmark::Result`` at ``benchmark.hpp:104-109``. The mean
    fields default to their sentinel (``0.0`` / ``0``) so that a fully-failed
    benchmark row is still well-formed and prints without exploding the
    C++-style table.
    """

    problem_instance_name: str = ""
    planner_name: str = ""
    iterations: int = 0
    success_count: int = 0
    used_time: float = 0.0
    used_memory: float = 0.0
    used_time_for_collisions: float = 0.0
    makespan: float = 0.0
    sum_of_costs: float = 0.0
    number_of_CC: int = 0
    number_of_IC: int = 0


class Benchmark:
    """Registry + runner.

    Mirrors ``class Benchmark`` (``benchmark.hpp:102-433``). The
    ``spatial_planners`` / ``generators`` registry and the spatial-only
    ``RoadmapBenchmarkAll`` member are **not** ported (the spatial planners
    are not in this tree).

    Construction parameters
    -----------------------
    iterations : int, default 0
        ``> 0`` means "run exactly ``iterations`` solves per pair". ``<= 0``
        means "loop until ``time_limit`` is exceeded" (matches the C++'s
        ``while (count < iterations || (iterations <= 0 && all_used_time <
        time_limit))``).
    time_limit : float, default 0.0
        Only consulted when ``iterations <= 0``. Units are milliseconds to
        match the C++'s ``all_used_time`` accumulator; the C++ compares the
        same accumulator to ``time_limit`` in seconds after a divisor; we
        keep the comparison in millisecond units for clarity.
    """

    def __init__(self, iterations: int = 0, time_limit: float = 0.0) -> None:
        self.iterations = iterations
        self.time_limit = time_limit
        # The C++'s ordered maps ``problem_instances`` and ``planners``. We
        # preserve insertion order via ``Dict`` (3.7+ preserves insertion
        # order).
        self.problem_instances: Dict[str, ProblemInstance] = {}
        self.planners: Dict[str, Planner] = {}
        self.pairs_to_run: Set[Tuple[str, str]] = set()
        self.results: List[BenchmarkResult] = []

    # -- Registry -------------------------------------------------------

    def add_problem_instance(
        self, problem_instance: ProblemInstance, name: str
    ) -> None:
        self.problem_instances[name] = problem_instance

    def add_planner(self, planner: Planner, name: str) -> None:
        self.planners[name] = planner

    def add_pair_to_run(
        self, problem_instance_name: str, planner_name: str
    ) -> None:
        # ``assert(planners.find(planner_name) != planners.end())`` -- the
        # C++ aborts if the planner is unknown. We do the same; in tests
        # the orchestrating layer (CLI / YAML) registers planners first.
        assert planner_name in self.planners, (
            f"Benchmark.add_pair_to_run: planner {planner_name!r} is not "
            f"registered; call add_planner() first."
        )
        self.pairs_to_run.add((problem_instance_name, planner_name))

    # -- Output helpers -------------------------------------------------

    def print_plan(
        self,
        problem_instance: ProblemInstance,
        plan: Plan,
        file_name: str,
    ) -> None:
        """Emit the C++'s ``printPlan`` output to ``file_name``.

        Mirrors ``Benchmark::printPlan`` at lines 127-158. The output format
        is the C++'s free-form waypoints: per agent, a header line with
        ``<radius>`` followed by ``<route_size>``, then alternating
        ``<time>`` and ``<vertex_x> <vertex_y>`` lines.

        The literal ``"stdout"`` sentinel (``benchmark.hpp:130``) is
        preserved.
        """
        close_after = False
        if file_name == "stdout":
            out = sys_stdout()
        else:
            out = open(file_name, "a")
            close_after = True
        out.write(f"{len(plan)} {problem_instance.get_agent_radius():f}\n")
        for agent in range(len(plan)):
            vertex = problem_instance.tasks[agent].initial_vertex
            t = 0.0
            route: List[Tuple[int, float]] = [(vertex, t)]
            for mv in plan[agent]:
                if t < mv.start_time:
                    t = mv.start_time
                    route.append((vertex, t))
                t += problem_instance.get_edge_length(vertex, mv.edge_id)
                vertex = problem_instance.get_target_vertex(vertex, mv.edge_id)
                route.append((vertex, t))
            out.write(f"{len(route)}\n")
            for (v, t) in route:
                out.write(f"{t:f}\n")
                problem_instance.print_vertex(out, v)
        if close_after:
            out.close()

    def display_all_results(self) -> None:
        """Mirror of ``displayAllResults``.

        Format matches ``benchmark.hpp:316-333`` exactly: a header line with
        10 columns, then one line per result with the same 10 columns.
        All floats use ``%10.xf`` formats (with the precision matching the
        C++ column widths).
        """
        # Use a list buffer so callers can capture or pipe the output to
        # wherever they need it. The CLI writes this to ``stderr`` to match
        # the C++ (``benchmark.hpp:316``: ``fprintf(stderr, ...)``).
        import io

        buf = io.StringIO()
        buf.write(
            "   planner    problem    success   makespan sumofcosts  "
            "ptime(ms)  ctime(ms) "
            "memory(kB)    countCC    countIC iterations\n"
        )
        for result in self.results:
            if result.iterations == 0:
                success_rate = 0.0
            else:
                success_rate = result.success_count / result.iterations
            buf.write(
                f"{result.planner_name:>10s} {result.problem_instance_name:>10s} "
                f"{success_rate:10.3f} {result.makespan:10.5f} "
                f"{result.sum_of_costs:10.5f} "
                f"{result.used_time - result.used_time_for_collisions:10.0f} "
                f"{result.used_time_for_collisions:10.0f} "
                f"{result.used_memory:10.0f} "
                f"{result.number_of_CC:10d} {result.number_of_IC:10d} "
                f"{result.iterations:10d}\n"
            )
        # Mirror the C++'s stream choice (stderr).
        import sys

        sys.stderr.write(buf.getvalue())

    # -- Runners --------------------------------------------------------

    def run(
        self,
        problem_instance_name: str,
        planner_name: str,
        output_file: str = "",
    ) -> BenchmarkResult:
        """Run a single ``(problem, planner)`` pair.

        Mirrors ``Benchmark::run`` (``benchmark.hpp:160-206``) and
        ``Benchmark::runSpecified`` (``benchmark.hpp:208-226``, which dispatches
        to ``run``).
        """
        if problem_instance_name not in self.problem_instances:
            raise KeyError(
                f"Benchmark.run: unknown problem_instance_name "
                f"{problem_instance_name!r}"
            )
        if planner_name not in self.planners:
            raise KeyError(
                f"Benchmark.run: unknown planner_name {planner_name!r}"
            )
        problem_instance = self.problem_instances[problem_instance_name]
        planner = self.planners[planner_name]
        planner.set_instance(problem_instance)

        count = 0
        success_count = 0
        number_of_CC = 0
        number_of_IC = 0
        used_time = 0.0
        used_time_for_collisions = 0.0
        used_memory = 0.0
        makespan = 0.0
        sum_of_costs = 0.0
        all_used_time = 0.0
        # The C++'s ``while`` reads from ``iterations`` and ``time_limit``
        # each iteration (in case callers mutate them), so we mirror that
        # state lookup here.
        while (count < self.iterations) or (
            self.iterations <= 0 and all_used_time < self.time_limit
        ):
            count += 1
            run = RunPlanner(planner, problem_instance)
            all_used_time += run.used_time
            if run.success:
                used_time += run.used_time
                used_memory += run.used_memory
                makespan += run.makespan
                sum_of_costs += run.sum_of_costs
                number_of_CC += run.number_of_CC
                number_of_IC += run.number_of_IC
                used_time_for_collisions += run.used_time_for_collisions
                success_count += 1
                if output_file:
                    self.print_plan(problem_instance, planner.get_plan(), output_file)

        result = BenchmarkResult(
            problem_instance_name=problem_instance_name,
            planner_name=planner_name,
            iterations=count,
            success_count=success_count,
        )
        if success_count > 0:
            # Mean over successful runs only.
            result.used_time = used_time / success_count
            result.used_memory = used_memory / success_count
            result.makespan = makespan / success_count
            result.sum_of_costs = sum_of_costs / success_count
            result.number_of_CC = number_of_CC // success_count
            result.number_of_IC = number_of_IC // success_count
            result.used_time_for_collisions = (
                used_time_for_collisions / success_count
            )
        self.results.append(result)
        return result

    def run_all(self, output_file: str = "") -> None:
        """Run every ``(planner, problem)`` pair.

        Mirrors ``Benchmark::runAll`` (``benchmark.hpp:304-313``): iterate
        over all planners and problems and ``run`` each pair.
        """
        for planner_name in list(self.planners.keys()):
            for problem_name in list(self.problem_instances.keys()):
                self.run(problem_name, planner_name, output_file)

    # -- The "incrementally" CSV producer ---------------------------------

    def run_incrementally_all_pairs(
        self,
        iterations: int,
        permutations: Optional[List[List[Tuple[int, int]]]] = None,
    ) -> None:
        """Sweep agent-count from 1 to ``max_n_agents`` and emit a CSV row per run.

        Mirrors ``Benchmark::runIncrementallyAllPairs``
        (``benchmark.hpp:236-302``). Each row is

            <problem_name>,<planner_name>,<n_agents>,<perm_id>,<count>,
            <makespan>,<sumofcosts>,<used_time>

        where ``-1.0`` makespan / sum-of-costs signal a failed run, matching
        the C++ ``printf("%10.3lf")`` formats at lines 284/290.

        The set of "running pairs" (``pairs_to_runs``) is per-iteration:
        each iteration has its own set, and a failure on iteration ``count``
        removes the pair from that iteration's set. We rebuild that per-
        iteration set in Python without mutating ``self.pairs_to_run``,
        so repeated calls are safe.

        Tasks are sampled from the saved ``all_tasks`` snapshot of the
        original instances — mutating the planner's task list in place is
        safe because each ``set_instance`` call re-binds the planner's
        ``problem`` to the freshly mutated instance.
        """
        permutations = permutations or []

        # 1. Sanity check: every pair's first problem must have
        #    ``iterations`` instances registered with suffixes
        #    ``<count>`` for count in 0..iterations.
        for prob_name, planner_name in self.pairs_to_run:
            for count in range(iterations):
                full = prob_name + str(count)
                assert full in self.problem_instances, (
                    f"run_incrementally_all_pairs: missing problem "
                    f"instance {full!r} for pair "
                    f"({prob_name!r}, {planner_name!r})."
                )

        # 2. Snapshot all tasks per problem instance.
        # We deep-copy because the C++ mutates ``problem.tasks`` in
        # place during ``runIncrementallyAllPairs`` (resize + overwrite).
        # The Python ``AgentTask`` is a ``NamedTuple`` so the in-place
        # mutation in C++ is replaced by an immutable rebind here. The
        # snapshot's purpose is to keep the original task rows available
        # for permuted rewrites across iterations / agent-counts.
        from copy import deepcopy

        all_tasks: Dict[str, List] = {
            name: deepcopy(inst.tasks)
            for name, inst in self.problem_instances.items()
        }

        if not all_tasks:
            return
        max_n_agents = len(next(iter(all_tasks.values())))

        # 3. Default identity permutation when none provided.
        if not permutations:
            permutations = [[(i, i) for i in range(max_n_agents)]]

        n_perms = len(permutations)
        pairs_to_runs: List[Set[Tuple[str, str]]] = [
            set(self.pairs_to_run) for _ in range(iterations * n_perms)
        ]

        import sys

        for n_agents in range(1, max_n_agents + 1):
            for perm_id in range(n_perms):
                for count in range(iterations):
                    prob_id = perm_id * iterations + count
                    permutation = permutations[perm_id]
                    # We iterate over a snapshot of the set so we can
                    # mutate ``pairs_to_runs[prob_id]`` (erase failed pairs)
                    # without invalidating our iterator.
                    current_pairs = list(pairs_to_runs[prob_id])
                    for pair in current_pairs:
                        prob_name, planner_name = pair
                        full = f"{prob_name}{count}"
                        problem_instance = self.problem_instances[full]
                        planner = self.planners[planner_name]
                        # Resize tasks to ``n_agents``. The C++ calls
                        # ``ProblemInstance::tasks.resize(n_agents)``
                        # which truncates the in-place vector. ``list``
                        # supports the same via slice assignment. Note
                        # Python's ``AgentTask`` is a ``NamedTuple`` (no
                        # field-writeable), so each retained task is
                        # rebuilt from the saved ``all_tasks`` snapshot.
                        from psipp_ctc.problem import AgentTask

                        saved = all_tasks[full]
                        retained: List = []
                        for i in range(n_agents):
                            src = saved[permutation[i][0]]
                            dst = saved[permutation[i][1]]
                            retained.append(
                                AgentTask(initial_vertex=src.initial_vertex,
                                          goal_vertex=dst.goal_vertex)
                            )
                        problem_instance.tasks = retained
                        planner.set_instance(problem_instance)
                        run = RunPlanner(planner, problem_instance)
                        if run.success:
                            sys.stdout.write(
                                f"{prob_name:>11s},{planner_name:>6s},"
                                f"{n_agents:4d},{perm_id:4d},{count:4d},"
                                f"{run.makespan:10.3f},{run.sum_of_costs:10.3f},"
                                f"{run.used_time:10d}\n"
                            )
                            # Successful pair stays in pairs_to_runs.
                        else:
                            sys.stdout.write(
                                f"{prob_name:>11s},{planner_name:>6s},"
                                f"{n_agents:4d},{perm_id:4d},{count:4d},"
                                f"{-1.0:10.3f},{-1.0:10.3f},{run.used_time:10d}\n"
                            )
                            # Failed pair is removed from this iteration.
                            pairs_to_runs[prob_id].discard(pair)

    def run_specified(
        self,
        problem_instance_name: str,
        planner_name: str,
        output_file: str = "",
    ) -> Optional[BenchmarkResult]:
        """Run a single pair after validating names.

        Mirror of ``Benchmark::runSpecified`` (``benchmark.hpp:208-226``).
        Returns ``None`` if either name is unknown, mirroring the C++ which
        writes a ``stderr`` message and returns without running.
        """
        if problem_instance_name not in self.problem_instances:
            import sys

            sys.stderr.write("Unknown problem instance name\n")
            return None
        if planner_name not in self.planners:
            import sys

            sys.stderr.write("Unknown planner name\n")
            return None
        return self.run(problem_instance_name, planner_name, output_file)


def sys_stdout():  # small helper to avoid binding to stdout at import time
    import sys

    return sys.stdout


__all__ = [
    "Benchmark",
    "BenchmarkResult",
    "RunPlanner",
    "get_used_memory_kb",
]

