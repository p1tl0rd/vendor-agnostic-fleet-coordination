# Architecture — PSIPP/CTC in this codebase

Long-form algorithm reference for the port. Companion to [`../HANDOFF.md`](../HANDOFF.md)
(agent handoff) and `.ua/knowledge-graph.json` (machine-readable graph).

All C++ paths below are relative to `PSIPP-CTC_src/`.

---

## 0. Why this algorithm exists

Industrial AGV/AMR fleets are vendor-locked, and classic MAPF is a poor fit: it discretizes
time and assumes instantaneous acceleration/braking, while a heterogeneous fleet has very
different kinematics per vehicle. The thesis builds a central **Master Control** that
coordinates heterogeneous robots from any vendor over **VDA 5050 / MQTT**, using continuous-time
MAPF on 2D roadmaps as the planning core.

PSIPP/CTC is fast — the source paper reports up to ~2000 agents in ~30 s, where CCBS falls
over around ~100 — but it has three documented weaknesses that this thesis targets as
contributions: bottleneck deadlock, suboptimality, and kinematics-agnostic timing (see §7).

**Offline/online split.** The whole design rests on one idea: **precompute every possible
pairwise conflict once, then answer all planning queries by table lookup.** Conflicts depend
only on geometry, never on the plan, so they can be annotated before any robot moves. This is
the time/memory trade the paper is about: pay a large one-off annotation cost, then make
online planning cheap enough to scale.

---

## 1. Stage 0 — Problem model

`src/base/problem.hpp`

- A roadmap is an **adjacency list**: `edges[v] = [target, ...]`. Therefore **an edge IS its
  index in that vertex's out-list**. There is no separate edge array.
- **Edge length is travel time at unit speed.** This single decision is why the planner is
  kinematics-agnostic (§7.3).
- A task is `AgentTask {initial_vertex, goal_vertex}`.
- A plan is `vector<vector<Move>>`, one `Move` list per agent, where
  **`Move = {edge_id, start_time}` and `start_time` is a real number.** A plan is a
  time-parameterised path in continuous time, not a sequence of integer ticks.
- **Waiting is encoded as `edge_id < 0`**, or equivalently `source == target`.
- Constants: `INF` and `EPS = 1e-8`.
- `Planner` (`src/base/planner.hpp`) is the minimal interface every planner implements:
  `setInstance`, `solve`, `succeeded`, `getPlan`.

### Dispatchers — the trick that makes waiting uniform

`CollisionCheck`, `CollisionCheck2`, `getCollisionInterval` and `getCollisionInterval2` are
default-argument **dispatchers**. They inspect the edge ids, detect which agents are waiting,
and route to the right kernel — swapping agent order when agent 1 is the waiter, and
converting the returned interval into the caller's time frame via
`lower = start_time_1 - lower`. Downstream planners therefore never special-case waiting.

### `checkPlanValidity` — the independent oracle

`problem.hpp` also carries a brute-force verifier: it expands each agent's `Move` list into
timed `(vertex, edge)` routes with explicit wait segments, then throws if a move starts before
the agent could have arrived, an edge id is out of range, the agent misses its goal, or any
pairwise segment collides. It reports **makespan** (max arrival time) and **sum of costs**.

This is the ground truth for parity: not matching output text, but `checkPlanValidity`
accepting the plan with matching metrics. Note that makespan and sum-of-costs come from the
validator, **not** from the planner (`src/base/benchmark.hpp` runs both and cross-checks).

---

## 2. Stage 1 — Geometry and continuous-time collision algebra

`src/base/geometry_2D.hpp` — the algebraic heart of the system.

Agents are **discs** of radius `r`; a conflict exists while centre distance is below `2r`.
There is **no time discretisation anywhere** — every interval is an exact closed interval of
reals.

### Primitive inequality solvers

| Routine | Solves |
|---|---|
| `solve_linear_inequation` | `A*x + B <= 0`, clamping `[lower, upper]`. Collapses to empty `(INF, -INF)` when `A ≈ 0` and `B > 0`. |
| `solve_quadratic_inequation` | `A*x² + 2*B*x + C <= 0` for `A >= 0` (`D = B² - A*C`), including degenerate linear and always-true cases. |
| `solve_vector_inequation` | `\|a*t + b\| <= r` by reduction to the quadratic with `A = \|a\|²`, `B = a·b`, `C = \|b\|² - r²`. **Every CTC kernel is built on this one.** |

### The two CTC kernels

**VEC — Vertex-Edge Conflict** (`getCollisionInterval`, vertex vs moving edge).
A stationary agent sits at `p0`; another moves `sp1 -> tp1` at unit speed over `dt1`. Reduce
to `|a*t + b| <= 2r` with `a = -(tp1 - sp1)/dt1`, clip to `[0, dt1]`, and report success iff
`lower < upper`.

**EEC — Edge-Edge Conflict** (`getCollisionInterval`, edge vs edge).
The single most important routine in the codebase. Two agents traverse their own segments over
their own `[st, tt]` windows. It returns the **union of start-time offsets `s`** for which the
two come within `2r`. Separation is minimised at five candidate points, and each candidate
`range of `s` is intersected with the linear constraints `0 <= t <= dt0` and `s <= t <= s+dt1`,
then min/max-merged:

1. `t = 0` — agent 0 at its start,
2. `t = s` — simultaneous start,
3. `t = dt1` — agent 1 at its end,
4. `t = s + dt0` — simultaneous finish,
5. the **interior perpendicular case** — the relative-position vector is orthogonal to the
   relative velocity, `t = a·(b + c*s)/|a|²`, which is a *quadratic in `s`*.

The boolean wrappers simply test overlap in continuous time, e.g. VEC collides iff
`max(st0, st1 + lower) + EPS < min(tt0, st1 + upper)`, and EEC iff
`lower + EPS < st0 - st1 < upper - EPS`.

`src/base/geometry_2D_for_generator.hpp` adapts `Point` into `boost::geometry` traits and adds
`Space2D` — the free-space model (boost polygon with holes) plus clearance-aware containment,
random task sampling, bounds and free-volume estimation.

---

## 3. Stage 2 — Offline CTC annotation (the precompute)

### Broad phase — candidate generation

`src/base/collision_report.hpp` proposes candidates only; exact intervals come later from
`Geometry2D`.

- **`Buckets2D`** — uniform spatial hash over roadmap vertices with cell size equal to the
  conflict radius. Two query modes:
  - `calculateAllPairs` — VV candidates. For each occupied bucket it scans 5 neighbour buckets
    via the fixed offsets `{(0,0), (0,1), (1,-1), (1,0), (1,1)}`, inserting every within-radius
    vertex pair into a set, deduplicating by construction.
  - `calculateCollisionVertices` — VE candidates. Walks the 3×3 bucket neighbourhood of a
    segment `s -> t` while advancing the search cell with radius-compensated stride, collecting
    every vertex within the radius. Those are exactly the vertices whose stationary interval
    must be annotated against this edge.
- **`BentleyOttoman`** *(sic — 'Ottoman' is the source's spelling)* — Bentley-Ottmann
  sweep-line. Randomly rotates all segments so none is vertical, sweeps in `x`, maintains the
  active ordering in a priority queue with emerge/vanish/cross events. Every detected crossing
  is emitted as a **guaranteed** EEC candidate, because two crossing edges must conflict at
  *some* relative time. `checkPair` flags collinear segments as crossing outright and otherwise
  solves the crossing time from intercept/slope, rejecting it if it falls outside either
  segment's x-extent. Purely geometric — it knows nothing about agent radius or timing.

### Materialisation

`src/base/problem_with_explicit_collisions.hpp` makes the annotation **permanent**. Its
constructor flattens the roadmap into **global edge indices**, runs the two broad-phase passes,
and stores an exact interval for every surviving VV / VEC / EEC pair. After that, every query
is a map lookup instead of geometric algebra.

Two sibling variants:

- **`problem_with_grid.hpp`** — regular-grid MAPF, translation-invariant. Builds the `2^K`
  neighbourhood template, tabulates EV and EE offsets relative to a single edge, mirrors them by
  the four rotations of the square lattice, and answers every query by adding an offset to the
  current cell. Contains a known `edge_id` shadowing bug and a missing
  `all_collision_edges.clear()`.
- **`problem_with_turning.hpp`** — emulates non-holonomic rotation by **graph expansion**:
  each original edge becomes a 4-vertex chain with move-offset costs, each original vertex gains
  bidirectional turning edges between angularly sorted incident edges with cost
  `angle_distance / turning_speed`. Parent annotations are remapped onto the expanded graph, and
  `edge_costs` — not geometric length — become the edge length.

### The four annotation queries

Defined on `ProblemInstance`; the base class provides brute-force versions, subclasses
override with table lookups.

| Query | Returns |
|---|---|
| `getAllVVCollisions` | Vertices in static conflict. |
| `getAllVECollisions` | For a vertex occupied during a window: every `(edge source, edge id, -upper, -lower)` — **relative entry times**, exactly what safe-interval planning needs. The negation and swap encode the reverse-time convention. |
| `getAllEVCollisions` | Every `(vertex, lower, upper)` where a stationary agent there conflicts with the given edge. |
| `getAllEECollisions` | Every `(edge source, edge id, lower, upper)` of another edge plus the conflict window. |

---

## 4. Stage 3 — Online prioritized safe-interval planning

`src/planners/PrioritizedSIPP.{hpp,cpp}` — the thesis core.

Plan agents **one at a time** in priority order. State space of the A* is
**`(vertex, safe interval)`**, not `(vertex, time)`.

### Data structures

| Structure | Meaning |
|---|---|
| `vertex_safes[v]` : `set<IntervalWithID>` | Safe intervals at vertex `v`. **Carry ids** because each interval is an A* state. |
| `edge_safes[v][e]` : `set<Interval>` | Blocked windows on edge `e` out of `v`. **No id** — they are only queried for feasibility, never used as A* states. |
| `interval_data[]` : `SafeIntervalData` | Flat A* bookkeeping: `best_time`, `previous`/`prev_vertex`/`prev_edge_id` for route reconstruction, plus `open_listed`/`close_listed`. |
| `current_states[v][e]` : `CurrentState` | Monotone `std::set` iterators (`edge_it`, `target_it`) so each edge's windows are swept once. |

`addVertexCollisionInterval` subtracts a blocked window `[start, end]` from a safe-interval set,
**keeping the identity of every surviving interval and minting a fresh id only when a window
splits one interval into two**. `addEdgeCollisionInterval` is the same subtraction without ids.

### The priority loop — read this part carefully

```
for priority_order in 0..n:
    agent = priorities[priority_order]
    if priority_order > 0:
        j = priorities[priority_order - 1]        # ONLY the previous agent is added
        for each step of j's route:
            if j waits at step:  addEdgeCollisionInterval(...)   from getAllVECollisions
            else:                 addVertexCollisionInterval(...) from getAllEVCollisions
                                  addEdgeCollisionInterval(...)  from getAllEECollisions
    run A* over (vertex, safe interval) states for `agent`
    first agent that cannot reach its goal interval ABORTS the whole run
```

Two invariants that a naive port breaks:

1. `open_listed` / `close_listed` use **`priority_order` itself as a generation counter**.
   There is deliberately **no per-agent clearing** of the interval structures — a stale
   `close_listed` from a previous agent must not be mistaken for a closed state this round.
2. Blocked windows **accumulate monotonically**. Each round adds only the previous agent's
   conflicts; every earlier agent's windows persist.

Timing inside the search: `current_time = interval_data[interval_id].best_time` and
`deadline = it->end` (the safe interval's end). Traversing edge `e` from `v` costs
`getEdgeLength(v, e)` — the edge's geometric length, i.e. unit-speed time. Reaching the goal
*interval* terminates the search. Route reconstruction re-inserts waits as `edge_id == -1`.

`heuristic_distance` is plain Euclidean distance between vertices — admissible **only**
because edge length is reused as time.

`priority_strategy` is `0` (fixed FIFO) or `1` (shuffled), and `load_planners.cpp:24`
**hardcodes it to 0** — it is not reachable from YAML.

---

## 5. Stage 4 — Roadmap generation

`src/roadmap_generators/`

- **`ProbabilisticRoadmap.cpp`** — two variants. A template `KNearestProbabilisticRoadmap`
  keeping the `K` closest points per sample, with logarithmic default
  `K = ceil((e*(1+1/dim)+eps) * log n)`; and `RadiusProbabilisticRoadmap2D`, which uses
  spatial-hash buckets to connect all points inside a radius. Both append two extra start/goal
  vertices per task.
- **`ConstrainedTriangulation.cpp`** — CGAL `Constrained_Delaunay_triangulation_2`. Task
  endpoints and random interior samples are triangulated; **each obstacle ring is offset
  inward by the agent radius and inserted as a hard constraint**, so every finite CDT edge is
  guaranteed collision-free for a disc agent.
- **`Grid.cpp`** — uniform 2D grid. Cell centres inside free space become vertices; symmetric
  edges are added between neighbours whose connecting segment stays inside free space; each task
  start/goal is appended as its own vertex wired to the four surrounding cells.

**Gotcha:** kPRM with `number_of_vertices: 0` means *no* random vertices — the graph is only
the `2 * tasks` agent start/goal vertices.

`src/base/spatial.hpp` is the continuous-space counterpart of `ProblemInstance`: a task is a
pair of real points rather than vertex ids, a plan is a list of `(waypoint, arrival time)`
pairs, and it carries `checkRoadspaceValidity`, which validates a **generated** roadmap
against the space it was generated in.

---

## 6. Stage 5 — I/O contract (the port's hardest compatibility requirement)

Two hand-written text formats, read with `fscanf`, written with `%030.15lf`. The literal paths
`"stdin"` and `"stdout"` are special-cased.

- **`base/input.hpp`** — ROADMAP problem format: vertices and edges, then agent lines. This is
  what `psipp-plan` must read.
- **`base/input_spatial.hpp`** — CONTINUOUS-SPACE problem format plus MAPF_Benchmark assets:
  `loadTextFile` (hand-written polygon format), `loadGripMap` (traces a `.map` grid into polygon
  rings), `loadScenario` (`.scen`), and the free functions `loadMAPFBGrid` / `loadMAPFBScenario`.
  Spatial problems use an octile grid plus flood fill, right-hand boundary tracing, and a
  `+0.5` cell-centre offset.

**Known asymmetry:** the field order of `printTextFile` and `loadTextFile` does not match.
Verify empirically against the Docker oracle before trusting a round-trip.

---

## 7. The three thesis improvement targets

Tagged `improvement-target` in the knowledge graph. **Features to improve, not bugs to
replicate** — keep the port modular so planner policy can be swapped.

### 7.1 Bottleneck deadlock → algorithm is not complete

Hard FIFO priority can let a high-priority robot block a single doorway forever. The A* for
the blocked robot simply never reaches its goal interval, and the run aborts. This is a direct
consequence of the priority loop in §4. Thesis direction: adaptive priority, yielding,
negotiation between robots.

### 7.2 Suboptimality

Low-priority robots are forced into very long detours, so sum-of-costs is far from minimal.
This is inherent to prioritized planning. The vendored CCBS baseline is the optimal reference.

### 7.3 Kinematics-agnostic timing

Every duration is `edge_length / 1`. Per-robot speed, acceleration and turning limits are not
modelled at all. `problem_with_turning.hpp` is the seed of the fix (graph expansion +
`turning_speed`) and of thesis Phase 3 (VDA 5050 Base/Horizon, ROS 2 + Gazebo, per-robot Nav2).

---

## 8. The comparison baseline: CCBS

`PSIPP-CTC_src/third_party/Continuous-CBS/**` — vendored, unmodified third-party.

- **`cbs.cpp`** — Conflict-Based Search high level: CBS nodes over constraints, conflict
  detection, branching, disjoint splitting, focal/high-level heuristics.
- **`sipp.cpp`** — SIPP low level: safe intervals for a single agent, arbitrary-duration moves
  on a roadmap or grid.
- **`config.cpp`**, **`const.h`** — search config: `use_cardinal`, `use_disjoint_splitting`,
  `hlh_type`, `connectedness`, `focal_weight`, `agent_size`, `timelimit`, `precision`.
- **`map.cpp`/`task.cpp`** — GraphML / XML parsers.
- `Examples/*.xml` is a useful **structural analogue** of a serialised roadmap + task + log.
  `roadmap.xml` is GraphML with `coords` (comma-separated `i,j`) and `weight` keys; it carries
  topology and geometry only, with **no conflict or interval annotations**. Its output logs
  reveal that SIPP trajectories leave the graph — path durations exceed straight-line edge
  length, and degenerate zero-length sections mark waits.

**`WrappedContinuousCBS2.cpp` is how PSIPP-CTC_src talks to it**: it serialises the roadmap
and tasks to temporary GraphML/XML, delegates single-agent planning to the vendored SIPP, and
detects/resolves conflicts through `ProblemInstance::CollisionCheck2` /
`getCollisionInterval2`. That coupling is the port's only file-level record of CCBS usage.

`src/planners/ExtendedIncreasingCostTreeSearch.cpp` — the ICTS baseline. Builds per-agent
time-expanded multi-valued decision diagrams (MDDs) bounded by a cost-to-goal budget, then
solves jointly with an earliest-start dynamic program that moves whichever robot can act
soonest. It is textually `#include`d into `src/base/load_planners.cpp`, so it **compiles**, but
no branch ever instantiates it and it has no header. **Unreachable — not a port target.**

---

## 9. Benchmark harness and metrics

`src/base/benchmark.hpp` — header-only experiment harness.

- `RunPlanner` / `RunSpatialPlanner` treat a run as successful only when `solve()` does not
  throw, `succeeded()` is true, **and** the independent `checkPlanValidity` accepts the plan.
- `Benchmark::run` repeats while `count < iterations || (iterations <= 0 && all_used_time <
  time_limit)`, then divides every metric by `success_count` — **means over successful runs
  only**. (Quirk: the `all_used_time += used_time` accumulator only affects time-limited mode.)
- The reentrant `timer_on`/`timer_off` pair starts the clock only on the outermost entry, so
  nested collision/interval calls are attributed to the top-level operation. This is what makes
  CTC annotation time separable from planning time.
- Printed columns: `success = success_count/iterations`, `ptime = used_time −
  used_time_for_collisions`, `ctime = used_time_for_collisions`, memory in kB.
- `runIncrementallyAllPairs` is the priority-order knob: it rebuilds tasks from
  `permutation[i].first/second` and sweeps agent count `1..max`. Failures emit `-1`
  makespan/sum-of-costs.
- `RoadmapBenchmarkAll` decomposes timing into roadmap `generate()` vs
  `ProblemInstanceWithExplicitCollisions` precompute vs online planning.

CSV row contract consumed by `PSIPP-CTC_src/tools/plot_*.py`:

```
problem_name, planner_name, number_of_agents, perm_id, iteration, makespan, sum_of_costs, used_time
```

`-1.0` makespan is the failure sentinel every plot script filters on. `plot_soc_ratio*` joins
against CCBS on the `perm_id*25 + iteration` subtask key to show the optimality gap;
`plot_runtime*` shows scalability; `plot_success_rate*` counts non-negative rows for failure
modes.

---

## 10. Entry points

| Binary | Python CLI | Config |
|---|---|---|
| `src/roadmap_generation.cpp` — generate roadmap, print roadspace + problem instance, run `checkRoadspaceValidity` | `psipp-generate-roadmap` | `config/roadmap_generation.yaml` (kPRM, 100 vertices, 10 neighbours) |
| `src/planner_benchmark.cpp` — build every `Problems` entry and `Planners` entry, run the full matrix | `psipp-plan` | `config/planner_benchmark.yaml` |
| `src/roadmap_benchmark.cpp` | — | `config/roadmap_benchmark.yaml` |

`makeGenerator` dispatches on YAML `type` to `PRM` / `PRMStar` / `kPRM` / `kPRMStar` / `GRID` /
`CDT`, else throws `"Unknown Generator Type"`. `makePlanner` dispatches to `PSIPP` / `CCBS`,
else throws `"Unknown Planner Type"`. `PRM` and `kPRM` read `radius` / `number_of_neighbors`
unconditionally, so omitting those keys makes yaml-cpp throw; `CDT` defaults to
`number_of_additionals = 0` and `use_corner_points = true` (the loader overrides the header's
`false` default).

`PSIPP-CTC_src/Dockerfile` adopts the prebuilt image `tunoob4ever/2psipp:latest` (tag unpinned,
no build steps); the source is bind-mounted read-only, so parity runs are
`docker run --rm -v $PWD/PSIPP-CTC_src:/repo:ro tunoob4ever/2psipp:latest bash -c '...'`.

---

## 11. Layer map

| Layer | Contents |
|---|---|
| Python port (deliverable) | `src/psipp_ctc/**`, `tests/**`, `pyproject.toml`, `config/default.yaml` — skeleton |
| C++ reference core | `src/base/**` — problem model, geometry, CTC annotation, I/O, benchmark |
| C++ reference planners | `src/planners/**` |
| C++ roadmap generators | `src/roadmap_generators/**` |
| C++ experiments | benchmark `main()`s, `src/experiments/**`, `src/test/**`, `experiments_all.sh` |
| Reference build/config | `CMakeLists.txt`, `Dockerfile`, `config/*.yaml`, `LICENSE` |
| Benchmark analysis | `tools/**` plot scripts, `simulation.py`, URDF models |
| Vendored third-party | `third_party/Continuous-CBS/**` |
| Documentation | three `README.md` files |

**No code coupling exists between the Python tree and the C++ tree.** The C++ tree is ground
truth to be read, not imported.