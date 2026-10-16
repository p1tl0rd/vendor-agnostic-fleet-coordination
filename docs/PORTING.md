# PORTING.md — C++ → Python port playbook

Operational playbook for porting `PSIPP-CTC_src/src/` to `src/psipp_ctc/`.

Three documents, three jobs — do not duplicate them:

| Document | Answers |
|---|---|
| `HANDOFF.md` | *Where are we?* — current state, invariants, parity workflow, gotchas |
| `docs/ARCHITECTURE.md` | *How does the algorithm work?* — stages, algebra, I/O contract |
| **`docs/PORTING.md`** (this file) | *How do I run the port?* — per-stage briefs, verification gate, traps |

`AGENTS.md` holds the hard rules and overrides everything here.
`PSIPP-CTC_src/` is read-only ground truth. Never edit it. Never import from it.

---

## 1. The session protocol

**One stage per session. Fresh context per session. Never "port the project".**

Each session runs this loop:

1. **Brief** — paste the stage brief from §3–§8 of this file, unmodified.
2. **Load the skill** — the agent must run `skill({ name: "cpp-to-python" })` before
   writing code. It carries the mapping tables and the repo-specific traps.
3. **Read** — `HANDOFF.md` in full, then the `ARCHITECTURE.md` sections named in the brief.
4. **Port** — the named C++ files only. No stage N+1 work.
5. **Verify** — pass the four checks in §10 with raw pasted output.
6. **Explain** — report in algorithm terms and stay available for questions (below).
7. **Record** — update `HANDOFF.md` §2 (state table) and §11, then commit.

Why one stage per session: stages 2–5 each build on an untested module if the agent
runs ahead. The port order exists because a wrong `geometry/` makes every downstream
result silently wrong — no crash, just wrong makespan.

### Who you are talking to

**The owner does not read C++.** They know the idea of PSIPP/CTC, they read Python, and
they will ask questions during the session. `AGENTS.md` has the full rules; the short
version:

- Explain in **algorithm terms**: robots, roadmap, time intervals, plans. Not classes,
  templates, or file names.
- Lead with **what changed in behaviour** and which numbers moved, then the location.
- **Define each term once per session.** No assumed knowledge of CGAL, boost::geometry,
  `fscanf`, `std::set` iterators, or template specialisation.
- To explain a trap: plain description → small concrete example → what it does to the
  robots' plan. A diff is evidence, never an explanation.
- Answer questions patiently and concretely. "It's in the C++" is not an answer.

This matters for correctness, not just manners. Every trap in §11 is designed to survive
pytest. The owner's questions are a second independent check on the same failure mode:
if you cannot explain what a step does to the plan in plain language, you do not yet know
that it is correct.

### The dependency correction

The published order `io -> geometry -> roadmap -> ctc -> planners` is right, but `io`
cannot literally go first. `base/input.hpp:12-14` is a **template specialisation of the
data model**:

```cpp
template <>
inline ProblemInstanceWithGeometry<Geometry2D>::ProblemInstanceWithGeometry(
    const std::string &file_path) {
```

and `getEdgeLength` (`base/problem_with_geometry.hpp:53-54`) calls `Geometry::distance`.
The real chain is:

```
Point  →  ProblemInstance  →  ProblemInstanceWithGeometry  →  io/load
```

So **Stage 0 = data model** precedes `io`. It is small and purely structural — no
algorithm. Without it, the agent has nothing to load into.

---

## 2. Stage map

| Stage | Python module | C++ sources (lines) | Difficulty |
|---|---|---|---|
| 0 | `problem.py` (proposed) | `base/problem.hpp` (415), `base/problem_with_geometry.hpp` (305) | Easy — structure only |
| 1 | `io/` | **1a:** `base/input.hpp` (80), `base/load_problems.{cpp,hpp}` (79+23) · **1b:** `base/input_spatial.hpp` (198), `base/spatial.hpp` (168) | **Hard contract** — 1b is split out because it depends on the geometry backend |
| 2 | `geometry/` | `base/geometry_2D.hpp` (285), `base/collision_report.hpp` (293) | **Hardest math** |
| 3 | `roadmap/` | `roadmap_generators/*.cpp` (239+116+124), `base/roadmap_generator.hpp` (22), `base/geometry_2D_for_generator.hpp` (194) | **Biggest unknown** — needs a CGAL/boost replacement |
| 4 | `ctc/` | `base/problem_with_explicit_collisions.hpp` (287) | Medium — dense, but mechanical |
| 5 | `planners/` | `planners/PrioritizedSIPP.{hpp,cpp}` (75+306), `base/planner.hpp` (20) | The thesis core |

**Explicitly out of scope for the port** (state this in every brief so the agent does
not wander):

| File | Why not |
|---|---|
| `planners/ExtendedIncreasingCostTreeSearch.cpp` (377) | ICTS baseline. `makePlanner` has no YAML branch for it → unreachable dead code |
| `planners/WrappedContinuousCBS2.cpp` (781) | CCBS integration. Shells out to the vendored solver via temp GraphML/XML. Separate decision, not part of the core port |
| `base/problem_with_grid.hpp` (358) | Lattice variant. Two known defects: `getAllEECollisions` declares a local `int edge_id` that shadows its parameter (`:346`), and it pushes to `all_collision_edges` without ever clearing it, so results accumulate across calls (`:337-354`) |
| `base/problem_with_turning.hpp` (254) | Kinematics work — Phase 3, not the port |
| `base/benchmark.hpp` (433) | Metrics harness. Port opportunistically once a planner exists; not a stage |
| `third_party/Continuous-CBS/**` | Vendored, unmodified, not ported |

---

## 3. Stage 0 — data model

**Sources:** `base/problem.hpp`, `base/problem_with_geometry.hpp`

```
Stage 0. Read HANDOFF.md in full, then docs/ARCHITECTURE.md §0-§1.
Load the cpp-to-python skill before writing code.

Task: port the data model only.
Sources: PSIPP-CTC_src/src/base/problem.hpp (core is lines 17-70 and 76-88) and
PSIPP-CTC_src/src/base/problem_with_geometry.hpp.

Deliver: Point, AgentTask, Move, ProblemInstance, ProblemInstanceWithGeometry.
Decide and write down where the module lives. Recommended: src/psipp_ctc/problem.py,
mirroring the C++ base/ directory. Do not create an Edge class.

Preserve exactly:
 - An edge has no object. Its id IS its index in the source vertex's out-list:
   edges[v] = [target, ...]. addEdge appends.
 - agent_geometry is a RADIUS. The conflict threshold is 2*radius, strict <.
 - getEdgeLength(source, edge_id) = Euclidean distance to the target vertex.
   This is why length == time: the reference moves at unit speed.
 - Move.start_time is a REAL value, not a step index. Waiting is edge_id < 0.
 - ProblemInstance.EPS = 1e-8.

Port the reentrant timer (problem.hpp:76-88) or document why you skipped it.

Tests: edge id indexes the out-list; getEdgeLength equals a hand-computed distance;
the timer accumulates nested intervals correctly.

Report: files created, the module-location decision, pytest output, four done-checks
from docs/PORTING.md §10.
```

### Trap: `setNumberOfVertex` is overridden

Base (`problem.hpp:23-27`) resizes `edges` only. The geometry subclass
(`problem_with_geometry.hpp:29-33`) resizes `coordinates` **and** clears/resizes `edges`.
Port both behaviours on the right class, or `coordinates` and `edges` silently desync.

### Trap: do not add an `Edge` dataclass

An edge has no identity beyond `(source, index)`. Inventing an `Edge` object breaks the
edge-id convention that every downstream stage and the text format depend on.

---

## 4. Stage 1 — io

**Sources:** `base/input.hpp`, `base/load_problems.{cpp,hpp}`, `base/input_spatial.hpp`

Split into two sessions — the roadmap text format first, the spatial/polygon format
second. Do not attempt both at once; the spatial side drags in `.map`/`.scen` parsing,
a boundary tracer, and a dependency on the geometry backend.

### Stage 1a — roadmap text format

```
Stage 1a. Read HANDOFF.md §3 and §8, docs/ARCHITECTURE.md §6.
Load the cpp-to-python skill.

Task: port the ROADMAP text format only.
Sources: PSIPP-CTC_src/src/base/input.hpp (80 lines),
PSIPP-CTC_src/src/base/load_problems.{cpp,hpp}.
Skip input_spatial.hpp entirely in this session.

Deliver: load_roadmap_problem(path) -> ProblemInstanceWithGeometry and
print_roadmap_problem(problem, path).

Preserve exactly:
 - the literal "stdin" / "stdout" path sentinels
 - field order: n_vertices n_edges n_agents, then coords, then edge pairs
   (source, target), then agent pairs (initial_vertex, goal_vertex), then radius
 - the %030.15lf float formatting for coordinates and radius
 - fscanf semantics: whitespace is a single token delimiter, newlines are NOT
   structural. Read tokens, do not read lines.

Tests: copy PSIPP-CTC_src/problem_instances/roadmaps/sample.txt and sample2.txt into
problem_instances/roadmaps/. Cover header, coordinates, edge insertion order, agent
pairs, and radius.

Report all four done-checks from docs/PORTING.md §10 with raw output.
```

### Stage 1b — spatial polygon format

```
Stage 1b. Read HANDOFF.md §3 and §8, docs/ARCHITECTURE.md §6.
Load the cpp-to-python skill.

Task: port the SPATIAL problem formats.
Sources: PSIPP-CTC_src/src/base/input_spatial.hpp (198 lines) and the data model in
PSIPP-CTC_src/src/base/spatial.hpp (168 lines, the SpatialProblemInstance class and its
Task struct).

Port these four, in this order:
 1. loadTextFile (input_spatial.hpp:15) — the continuous-space polygon format.
 2. loadMAPFBGrid (input_spatial.hpp:52) — octile .map grid -> free-cell grid.
 3. loadGripMap (input_spatial.hpp:106) — grid -> polygon rings, right-hand tracing.
 4. loadMAPFBScenario (input_spatial.hpp:163) — .scen task rows.

DEFER loadScenario (input_spatial.hpp:184) to Stage 3. It calls isInside(), which needs
the containment test and therefore the geometry backend. Do not invent a backend here.

Represent the space as plain Python: outer = list[Point] (a closed ring, first point
repeated at the end) and inners = list[list[Point]], one closed ring per hole. Do not
implement containment, proximity, or bounds queries in this session — those belong to
Stage 3 where Space2D lives.

Preserve exactly:
 - the "stdin" sentinel and fscanf tokenisation, same as Stage 1a
 - rings are closed by repeating the first point (input_spatial.hpp:31, 39)
 - loadMAPFBGrid keeps ONLY ONE connected component: it finds the first free cell in
   top-to-bottom, left-to-right order and flood-fills from there. Free cells outside
   that component become impassable (input_spatial.hpp:75-102).
 - the grid is padded with one impassable border cell on every side
 - free = '.', 'G', or 'S'; everything else is an obstacle
 - loadGripMap traces boundaries with the right-hand rule and applies a -1 shift to
   undo the padding (input_spatial.hpp:147, 154)
 - the FIRST ring traced becomes the outer boundary; every later ring becomes a hole
   (input_spatial.hpp:143). This is trace order, not geometric containment.
 - loadScenario offsets each grid cell by +0.5 to reach the cell centre
   (input_spatial.hpp:190, 192). Note the direction: -1 in loadGripMap, +0.5 in
   loadScenario. Two different offsets, opposite signs.

Tests: copy PSIPP-CTC_src/problem_instances/spatial/sample.txt and empty256.txt into
problem_instances/spatial/. Cover ring closure, the hole count, the single-component
flood fill on a map with an isolated free pocket, and the .scen row count.

Report all four done-checks from docs/PORTING.md §10 with raw output.
```

### Trap: three different EPS values, not two

| Constant | Value | Where |
|---|---|---|
| `Geometry2D::EPS` | `0` | `geometry_2D.hpp:23` |
| `ProblemInstance::EPS` | `1e-8` | `problem.hpp:21` |
| `SpatialProblemInstance::EPS` | `1e-6` | `spatial.hpp:27` |

Three classes, three values. `isInside` shrinks the free space by
`agent_geometry - EPS` (`spatial.hpp:29-30`), so `SpatialProblemInstance::EPS` is a
clearance margin, not a comparison tolerance. Do not unify any of them.

### Trap: `loadMAPFBGrid` silently discards part of the map

It does not load the whole grid. It finds the first free cell in scan order and
flood-fills, keeping only that one connected component (`input_spatial.hpp:80-102`).
Every free cell outside the component becomes an obstacle. On a map with an isolated
free pocket, the loaded map is smaller than the file. This is intentional — the planner
cannot serve tasks in an unreachable region — but it must be ported, not "fixed".

### Trap: `loadGripMap` decides outer vs hole by trace order

The first boundary ring the tracer closes becomes the outer boundary; all subsequent
rings become holes (`input_spatial.hpp:143-158`). It is not decided by area or
containment. Since tracing starts from the first free cell in scan order, the outer
boundary is found first — but only because of scan order. Do not replace this with a
containment test.

### Trap: `printTextFile` is not the inverse of `loadTextFile`

They are not even in the same file:

- `base/input_spatial.hpp:15-50` — `loadTextFile` reads `number_of_holes` and
  `number_of_agents` as the first two whitespace-separated tokens, then the outer ring, then per-hole rings, then `agent_geometry`, then the
  agent points.
- `experiments/simplify_grid.cpp:9-33` — `printTextFile` writes `number_of_holes`, then
  `number_of_outer_vertices`, then the rings, then `[number_of_agents, agent_geometry]`,
  then the agent points.

The agent count sits in a different position in the stream, so a `print → load` round
trip **fails by design**. This is a reference quirk, not a bug. Do not "fix" it.

Scoping consequence: `printTextFile` is driver-local to `simplify_grid`. If you are not
porting `simplify_grid`, you do not need that format — only `loadTextFile`.

### Trap: `fscanf` is not line-based

`fscanf(input_file, "%d%d%d", ...)` consumes across newlines. Python must tokenise on
whitespace (`str.split()`), not `readline()`. A line-based reader works on reference
output and breaks on hand-written instances — or the reverse. Either way it is wrong.

### Trap: `load_problems.cpp` reads mandatory YAML keys

`PRM`/`kPRM` read `radius` and `number_of_neighbors` unconditionally; yaml-cpp throws
when absent. `CDT` defaults `number_of_additionals=0` and `use_corner_points=true` —
the loader overrides the header's `false`. See `HANDOFF.md` §8 for the reference configs
that do not run as shipped.

---

## 5. Stage 2 — geometry

**Sources:** `base/geometry_2D.hpp`, `base/collision_report.hpp`

Highest risk. Everything downstream inherits its correctness. Do not let the agent
build on it in the same session.

```
Stage 2. Read HANDOFF.md §4-§5, docs/ARCHITECTURE.md §2.
Load the cpp-to-python skill.

Task: port the continuous-time collision algebra.
Sources: PSIPP-CTC_src/src/base/geometry_2D.hpp (285 lines) and
PSIPP-CTC_src/src/base/collision_report.hpp (293 lines).

Priority order:
 1. solve_linear_inequation (geometry_2D.hpp:27), solve_quadratic_inequation (:39),
    solve_vector_inequation (:69).
 2. The VEC kernel (geometry_2D.hpp:134).
 3. The EEC kernel (geometry_2D.hpp:164) — the single most important routine in the
    system. Five candidate minima: t=0 (:190), t=s (:200), t=dt1 (:210), t=s+dt0 (:221),
    interior perpendicular (:239).
 4. CollisionCheck and the getCollisionInterval dispatchers.
 5. Broad phase: Buckets2D uniform hash, BentleyOttoman sweep in collision_report.hpp.
    Purely geometric — no radius, no timing.

Preserve exactly:
 - intervals are OPEN and real-valued
 - the strict threshold: conflict iff centre distance < 2*radius
 - Geometry2D::EPS = 0, but ProblemInstance::EPS = 1e-8. They are different constants
   in different classes. Do not unify them.

Tests: port the reference's own property tests from src/test/geometry_2D_test.cpp
(152 lines). It already encodes the right invariants — reuse its sampling scheme:
 (a) 10000 random VEC cases: sample s at 10000 interior points; assert
     (lower < s < upper) matches distance(p0, p1) < 2r.
 (b) 10000 random cases: assert CollisionCheck matches brute-force sampling of the
     shared time overlap at 10001 points.
 (c) 10000 random EEC interval cases: sample s = lower + (upper-lower)/N * g for
     g in -N..2N; expect true only for 0 < g < N.
The reference uses std::random_device (unseeded). Property tests need no seed — they
check invariants, not golden values. Keep it that way.

Report all four done-checks with raw output. Report the property-test counts actually run.
```

### Trap: do not tighten or loosen the boundary convention

`lower < s && s < upper` is strict in the reference's own test, and the dispatchers
mirror intervals with `lower = start_time_1 - lower`. Boundary behaviour is observable
at 1e-8 scale. Do not "clean it up".

### Trap: `Geometry2D::EPS = 0`

Zero, while `ProblemInstance::EPS = 1e-8` and `SpatialProblemInstance::EPS = 1e-6`.
Three classes, three values, all load-bearing. If the agent proposes one shared `EPS`,
reject it. The spatial one is a clearance margin, not a comparison tolerance — see §4.

---

## 6. Stage 3 — roadmap

**Sources:** `roadmap_generators/{ProbabilisticRoadmap,ConstrainedTriangulation,Grid}.cpp`,
`base/roadmap_generator.hpp`, `base/geometry_2D_for_generator.hpp`

The biggest unknown: the reference depends on **CGAL** (ConstrainedTriangulation) and
**boost::geometry** (`geometry_2D_for_generator.hpp` is pure boost geometry —
`checkVertexInside`, `checkEdgeInside`, `getBound`, `getRandomPoints`,
`getSepareteRandomPoints`, `calculateFreeVolume`). Both are banned by `AGENTS.md`.
Choose and record a replacement backend (e.g. shapely) **before** porting, and record
the numerical-divergence risk in the module docstring.

```
Stage 3. Read HANDOFF.md §3-§5, docs/ARCHITECTURE.md §5 and §6.
Load the cpp-to-python skill.

Task: choose the geometry backend, then port roadmap generation.
Sources: PSIPP-CTC_src/src/roadmap_generators/ProbabilisticRoadmap.cpp (239 lines),
ConstrainedTriangulation.cpp (116), Grid.cpp (124), base/roadmap_generator.hpp (22),
base/geometry_2D_for_generator.hpp (194).

The spatial data model (SpatialProblemInstance, its Task struct, and the text/map/scen
loaders) was already ported in Stage 1b as plain Python. Do not rewrite it. This stage
adds the geometry backend and the queries it enables:
 - Space2D::checkVertexInside, checkEdgeInside (geometry_2D_for_generator.hpp:51,70)
 - Space2D::getBound (:100), getRandomPoints (:114), getSepareteRandomPoints (:142),
   calculateFreeVolume (:182)
 - SpatialProblemInstance::isInside (spatial.hpp:29,33) and the loadScenario deferred
   from Stage 1b (input_spatial.hpp:184)

FIRST, before writing generator code: choose one Python geometry backend, write down
why, and record the numerical-divergence risk in the module docstring. CGAL and
boost::geometry are banned in src/psipp_ctc/. This decision blocks everything else here.

Then do the generators in this order: Grid, then ProbabilisticRoadmap, then
ConstrainedTriangulation. Grid has no external dependency; CDT needs the most work.

Preserve exactly:
 - the adjacency-list output contract: edges[v] = [target, ...]
 - kPRM default K = ceil((e*(1+1/dim)+small_value) * log n) when `number_of_neighbors`
   is omitted (`e` is Euler's number, i.e. `std::exp(1.)`; `small_value` is the
   reference's slack constant — not one of the three EPS values)
 - kPRM number_of_vertices = 0 means NO random vertices: the graph is exactly the
   2*tasks agent start/goal vertices
 - both PRM variants append 2 start/goal vertices per task
 - CDT never inserts the raw ring edges as constraints. It builds offset borders at the
   agent radius, inserts an offset segment as a hard constraint only when both endpoints
   AND the whole segment pass isInside (ConstrainedTriangulation.cpp:71-82), then keeps
   only CDT edges whose segment also passes isInside (:103). That clearance filter is what
   makes every finite CDT edge collision-free for a disc — replicate the filter, not just
   the offset
 - isInside tests with clearance agent_geometry - EPS (spatial.hpp:29-30), not a bare
   containment test

Tests: generate a roadmap for a small instance with each generator; assert the
adjacency-list contract, the task-vertex count, and connectivity between every
task's start and goal.

Report all four done-checks with raw output, and state the backend decision explicitly.
```

### Trap: kPRM's RNG is unseeded in the reference

Two runs of the reference on the same instance produce different roadmaps. Therefore
kPRM parity cannot be checked by byte-equality or by matching a golden output. Define
parity as: **the Python-generated roadmap passes `checkRoadspaceValidity`
(`base/spatial.hpp:43`, used by `roadmap_generation.cpp:28` and `make_roadmaps.cpp:32`)
and the reference planner accepts it.** Grid and CDT are deterministic and can be
compared more tightly.

### Trap: the roadmap generator is the only place randomness enters

If the agent reintroduces seeded RNG "for reproducibility", the port diverges from the
oracle's behaviour. Seeding is a documented deviation, not an improvement.

---

## 7. Stage 4 — ctc

**Sources:** `base/problem_with_explicit_collisions.hpp`

The offline annotation stage. Dense but mechanical once Stage 2 is trusted. This is
where the paper's time/memory trade is realised: geometry is paid once, then every
online query is a map lookup.

```
Stage 4. Read HANDOFF.md §5 (invariants 2, 3, 7), docs/ARCHITECTURE.md §3.
Load the cpp-to-python skill.

Task: port the offline CTC annotation.
Source: PSIPP-CTC_src/src/base/problem_with_explicit_collisions.hpp (287 lines).

Preserve exactly:
 - the constructor (:118) flattens the roadmap into GLOBAL edge indices (:86), runs the
   broad phases, and stores one exact interval per surviving VV/VEC/EEC pair
 - the VEC condition at :233-234 and the EEC condition
 - the four query overrides: VV (:268), VE (:271), EV (:275), EE (:279) — they are
   pure map lookups, no geometry
 - the (-upper, -lower) negation and swap conventions in the VE/EV dispatchers
 - the VV test is a SET LOOKUP, not a distance call: `collision_pairs_v2v` is built from
   `Buckets2D(2*agent_geometry - EPS, ...)` (:143, pair test `distance < radius`), and
   `CollisionCheck(vertex_0, vertex_1)` (:208-212) just queries that set. So VV means
   `distance < 2r - EPS` here — NOT the geometric instance's strict `distance < 2r`.
   Pairs inside that EPS band disagree between the two implementations

Tests: port src/test/explicit_collision_test.cpp (72 lines) as a DIFFERENTIAL test.
It builds both the geometric instance and the annotated instance and asserts they
agree on: VV booleans; VEC booleans and intervals (EPS 1e-6); EEC booleans and
intervals; and random-time CollisionCheck samples.
NOTE: that test loads ../problem_instances/roadmaps/generated.txt, which does not
exist on disk. Generate it first with the reference (config/roadmap_generation.yaml)
or substitute an existing sample.

Report all four done-checks with raw output, including the differential test.
```

### Trap: the differential test needs a roadmap that does not exist yet

`problem_instances/roadmaps/generated.txt` is referenced by the reference's own test but
is absent. Generate it via the oracle before running the differential test, or point the
test at `sample.txt`.

### Trap: annotation is order-sensitive and accumulate-only

Blocked windows accumulate monotonically across the priority loop (`HANDOFF.md` §5
invariants 2 and 3). Stage 4 must expose the same mutation semantics — an
`add_collision_interval` that returns a fresh set instead of mutating in place will
produce correct-looking but wrong plans in Stage 5.

---

## 8. Stage 5 — planners

**Sources:** `planners/PrioritizedSIPP.{hpp,cpp}`, `base/planner.hpp`

The thesis core. Every invariant in `HANDOFF.md` §5 applies here. Do not let the agent
start this stage until Stage 4's differential test passes.

```
Stage 5. Read HANDOFF.md §4, §5 (all seven invariants), §7, docs/ARCHITECTURE.md §4.
Load the cpp-to-python skill.

Task: port PrioritizedSIPP.
Sources: PSIPP-CTC_src/src/planners/PrioritizedSIPP.hpp (75 lines) and
PrioritizedSIPP.cpp (306), plus base/planner.hpp (20) for the interface.

Preserve exactly — these are the seven invariants:
 1. open_listed/close_listed use priority_order as a GENERATION COUNTER. There is
    deliberately no per-agent clearing.
 2. Blocked windows accumulate monotonically across the priority loop.
 3. addVertexCollisionInterval mints a new id only when a blocked window SPLITS one
    interval into two. Surviving intervals keep their identity.
 4. heuristic_distance is Euclidean and admissible only because edge length == time.
 5. edge_id < 0 means waiting; both dispatchers normalise.
 6. getAllVECollisions returns RELATIVE entry times as (src, edge_id, -upper, -lower).
    The negation and the swap are a reverse-time convention. Do not "fix" them.
 7. The annotation constructor flattens to global edge indices.
Also: A* state is (vertex, safe_interval). Vertex safe intervals carry ids because they
ARE states (set<IntervalWithID>). Edge blocked windows carry NO ids — they are a
feasibility gate only, never an A* state.

Simplify: first agent that cannot reach its goal interval aborts the whole run.
priority_strategy 0 = FIFO, 1 = shuffled. The reference hardcodes 0 (load_planners.cpp:24).

Out of scope: WrappedContinuousCBS2.cpp, ExtendedIncreasingCostTreeSearch.cpp.

Tests: parity against the Docker oracle on the instances from
config/planning_incrementally.yaml. Parity means checkPlanValidity accepts the plan AND
makespan/sum-of-costs match — NOT byte-identical output.

Report all four done-checks with raw oracle output pasted.
```

### Trap: the generation counter looks like a bug

`open_listed`/`close_listed` keyed on `priority_order` with no reset reads like an
uninitialised-state bug. It is the mechanism that makes per-agent A* reuse the same flat
arrays safely. An agent that "fixes" it by clearing per agent will produce plans that
look valid and cost more.

### Trap: negative `edge_id` and the reverse-time convention

`edge_id < 0` is a wait, and `getAllVECollisions` returns negated, swapped bounds. Both
are deliberate. Two separate normalisations are required for correctness — an agent that
simplifies either one breaks collision detection in a way that only shows up as an
invalid plan at 1e-8 scale.

---

## 9. After Stage 5

Stages 0–5 are done. Remaining work, in order (short form; the reasons and the
Berlin-CDT root cause are in `HANDOFF.md` §11):

1. **Close the Berlin CDT parity gap** — the only `xfail`. The reference's A* uses a
   *loose* edge-window check, and `std::priority_queue` breaks `f_value` ties in
   heap-array order while `heapq` breaks them by insertion order. Fix the heap, not the
   loose check: mirror libstdc++'s `push_heap`/`pop_heap` comparing only `f_value`.
2. **`base/benchmark.hpp` (433)** — the metrics harness, and the producer of the CSV
   the thesis figures read. Metrics are means over *successful* runs only;
   `ptime = used_time − used_time_for_collisions`; `ctime = used_time_for_collisions`.
   CSV columns:
   `problem_name, planner_name, number_of_agents, perm_id, iteration, makespan, sum_of_costs, used_time`,
   with `-1.0` makespan as the failure sentinel.
3. **YAML config loaders + the two CLIs** — `base/load_problems.{cpp,hpp}`,
   `base/load_planners.{cpp,hpp}`, `base/load_generators.{cpp,hpp}`. There is no YAML
   config module in `src/psipp_ctc/` yet, and both CLI stubs still
   `raise SystemExit("... not implemented yet")`. Targets:
   `psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt` and
   `psipp-plan < roadmap_problem.txt > plan.txt`.
4. **Decide the CCBS baseline strategy** — port `WrappedContinuousCBS2.cpp` (781), call
   the vendored C++ solver as an external binary, or merge the vendored solver's CSVs.
   The thesis's optimality-gap and success-rate figures need a CCBS side. Project
   decision, not a port session.
5. **Retire the oracle** — delete `PSIPP-CTC_src/` and remove the `/PSIPP-CTC_src/` line
   from `.gitignore` **in the same commit** (`AGENTS.md`). Not before item 1 is closed.
6. **Then** the three improvement targets in `HANDOFF.md` §6. Not before: the port is
   the baseline those changes are measured against.

---

## 10. The verification gate

"pytest green" is necessary and not sufficient (`AGENTS.md` hard rule 6). Every stage
reports all four, **with raw pasted output**:

| # | Check | How |
|---|---|---|
| 1 | Unit tests | `pytest` — paste the summary line, not a claim |
| 2 | Field parity | Reference prints an instance → Python loads it → compare every value, not just the count |
| 3 | Byte parity | Reference print vs Python print of the same instance → `diff` (skip for kPRM — see §6) |
| 4 | Oracle parity | The Docker command below, pasted with its output |

```bash
docker run --rm -v $PWD/PSIPP-CTC_src:/repo:ro tunoob4ever/2psipp:latest \
  bash -c 'cd /repo && /app/build/roadmap_generation < problem_instances/spatial/sample.txt > /tmp/rm.txt \
           && /app/build/planner_benchmark /tmp/rm.txt'
```

The image also ships the reference's own test binaries. Verify, then use them:

```bash
docker run --rm tunoob4ever/2psipp:latest bash -c 'ls /app/build/'
# expected to include: geometry_2D_test  explicit_collision_test
```

`geometry_2D_test` reads one case from stdin:
4 points (x y each), then `dt0 dt1 r`. It then runs its three randomized property
families. `explicit_collision_test` needs `problem_instances/roadmaps/generated.txt`.

### Evidence rules

Reject any of these as proof of completion:

- "tests pass" without pasted output
- "looks equivalent" / "behaves the same" — the oracle is the arbiter
- A plan that is valid but whose makespan or sum-of-costs differs from the oracle
- Passing unit tests written after the fact to match the implementation

---

## 11. Trap index

| # | Trap | Where |
|---|---|---|
| 1 | `setNumberOfVertex` is overridden and resizes coordinates too | `problem_with_geometry.hpp:29-33` vs `problem.hpp:23-27` |
| 2 | Do not invent an `Edge` class — an edge id is an out-list index | `problem.hpp:28` |
| 3 | `printTextFile` is driver-local and NOT the inverse of `loadTextFile` | `experiments/simplify_grid.cpp:9` vs `input_spatial.hpp:15` |
| 4 | `fscanf` is whitespace-tokenised, not line-based | `input.hpp:22-41` |
| 5 | **Three** different EPS constants in three classes | `Geometry2D::EPS=0` (`geometry_2D.hpp:23`), `ProblemInstance::EPS=1e-8` (`problem.hpp:21`), `SpatialProblemInstance::EPS=1e-6` (`spatial.hpp:27`) |
| 6 | Intervals are open and real-valued; boundary at 1e-8 is observable | `geometry_2D.hpp:134,164` |
| 7 | kPRM RNG is unseeded → parity is plan validity, not bytes | `ProbabilisticRoadmap.cpp` |
| 8 | kPRM `number_of_vertices=0` → only the `2*tasks` task vertices | `HANDOFF.md` §8 |
| 9 | CDT keeps only edges whose segment passes the `isInside` clearance filter | `ConstrainedTriangulation.cpp:71-82,103` |
| 10 | `generated.txt` is referenced by the reference's own test but absent | `explicit_collision_test.cpp:14` |
| 11 | `priority_order` as a generation counter — looks like a bug, is not | `PrioritizedSIPP.cpp:83-306` |
| 12 | Negative `edge_id` = wait; `getAllVECollisions` negates and swaps | `problem.hpp:299-312` |
| 13 | Global edge-index flattening is required before annotation | `problem_with_explicit_collisions.hpp:86` |
| 14 | Reference configs do not run as shipped | `HANDOFF.md` §8 |
| 15 | `ExtendedIncreasingCostTreeSearch` and `makeSpatialPlanner` are unreachable | `load_planners.cpp`, `load_problems.hpp:20` |
| 16 | `loadMAPFBGrid` keeps only ONE connected component; the rest becomes obstacles | `input_spatial.hpp:75-102` |
| 17 | `loadGripMap` decides outer vs hole by **trace order**, not containment | `input_spatial.hpp:143-158` |
| 18 | Two opposite coordinate offsets: `-1` in `loadGripMap`, `+0.5` in `loadScenario` | `input_spatial.hpp:147,154` vs `:190,192` |
| 19 | `isInside` shrinks free space by `agent_geometry - EPS`; it is clearance, not tolerance | `spatial.hpp:29-30,33-36` |
| 20 | Explicit VV collision is a set lookup at cutoff `distance < 2r - EPS`, not the geometric strict `distance < 2r` — the two instances disagree inside that band | `problem_with_explicit_collisions.hpp:143,208-212` |

---

## 12. Anti-patterns in a brief

Never say to the agent:

- **"Port the whole project to Python."** It will cross stages and build on untested code.
- **"Make it more Pythonic" / "clean it up."** `AGENTS.md` hard rule 1: behavioural
  equivalence over style. No algorithmic improvements during a port.
- **"Fix the bug you found in the reference."** Record it, do not fix it. The oracle is
  the specification. Divergences get a comment at the divergence site.
- **"Tests pass, looks good."** Requires pasted `diff` and oracle output.
- **"Adapt the reference a little."** `PSIPP-CTC_src/` is read-only. Never edit it,
  never import from it.

Always include in every brief:

- `Load the cpp-to-python skill before writing code.`
- `Read HANDOFF.md in full.` plus the specific `ARCHITECTURE.md` sections
- The exact C++ file list with line counts
- The preserve-exactly list
- `Report all four done-checks from docs/PORTING.md §10 with raw output.`
- `Explain each step in algorithm terms — what it does to the robots, the roadmap, or
  the time intervals. The owner does not read C++. Leave the questions open.`
- `Update HANDOFF.md §2 and §11, then commit.`
