# HANDOFF — PSIPP-CTC

**Read this first if you are a new agent (or a returning human) on this repo.**

Last updated: 2026-10-16 · Stage 0 + 1a + 1b + 2 + 3 + 4 + 5 + benchmark harness done · 158 tests pass + 2 xfail (Berlin CDT parity — PSIPP and Benchmark.run)

---

## 1. What this project is

A master's thesis codebase: a **Python port of PSIPP/CTC** (Prioritized Safe Interval Path
Planning with Continuous-Time Conflicts) for vendor-agnostic multi-robot fleet coordination
via **VDA 5050 / MQTT**.

The port must reproduce the original C++ implementation's **text I/O formats and planner
behaviour exactly**. `PSIPP-CTC_src/` is the parity oracle, not part of the deliverable.

## 2. Current state — read this before planning work

| Tree | State | Meaning |
|---|---|---|
| `src/psipp_ctc/**` | **Stage 0 + 1a + 1b + 2 + 3 + 4 + 5 + benchmark harness done.** `problem.py` has data model + spatial data model + the 4-arity `get_collision_interval` (3-arg VEC kernel, 4-arg EEC kernel, 5-arg VEC time-mirror, 6-arg EEC time-mirror). `io/roadmap.py` is the byte-identical roadmap text format. `io/spatial.py` carries 4 spatial loaders + `load_scenario`. `geometry/` carries the 3 primitive solvers + VEC + EEC + moving-vs-moving + `Buckets2D` + `BentleyOttoman`. `roadmap/` carries `space_2d.py`, `grid.py`, `probabilistic_roadmap.py` (kPRM + PRM), `constrained_triangulation.py` (CDT). `ctc/explicit_collisions.py` is the offline annotation with global edge indices + table-lookup overrides for `collision_check_vv/ve/ee` and `get_collision_interval`. `planners/planner.py` is the abstract base; `planners/prioritized_sipp.py` is the PSIPP port. `benchmark.py` is the metrics harness + CSV producer (port of `base/benchmark.hpp`, spatial-only bits out of scope until item 4). 158 tests pass + 2 xfail (Berlin CDT parity — PSIPP and `Benchmark.run` both inherit the same root cause), in ~100 s. Planners left to ship: `WrappedContinuousCBS2`, `ExtendedIncreasingCostTreeSearch` (out of scope per Stage 5 brief). | The deliverable. `planners` has PSIPP. `benchmark.py` is the planner-metrics harness. |
| `PSIPP-CTC_src/**` | Complete, working C++14 implementation (OMRON SINIC X, MIT). | Ground truth. Read-only. **Delete after the port completes** (and remove the `/PSIPP-CTC-src/` line from `.gitignore` then). |
| `PSIPP-CTC_src/third_party/Continuous-CBS/**` | Vendored CCBS baseline (unmodified). | Optimality/scalability comparison point. |
| `docs/` | Thesis `.docx` + 9 papers. Git-ignored. | Read for requirements, not for code. |

**Stage 0, 1a, 1b, 2, 3, and 4 are done.** Stage 4 ports the offline CTC annotation (`problem_with_explicit_collisions.hpp`): the constructor flattens the adjacency list into global edge indices, runs the broad phases, and stores one exact interval per surviving VV / VEC / EEC pair. Online queries are then table lookups, not geometry calls. The differential test (port of `explicit_collision_test.cpp`) compares the Python geometric instance (Stage 2 kernels) against the Python explicit instance (Stage 4 tables) on the same roadmap and asserts they agree on every VV boolean, every VEC boolean + interval, every EEC boolean + interval, and a sample of random time-overlap checks. Sample `problem_instances/roadmaps/sample.txt` (substituted for the C++ test's missing `generated.txt`, per PORTING.md §7) and `sample2.txt` (100-vertex kPRM) both pass. Two known robustness fixes in the geometry kernels: `dt == 0` returns `(False, 0, 0)` in VEC and moving-vs-moving; the `A == 0` EEC interior case falls into the linear branch. Both match what the C++ WOULD return if it weren't crashing on division by zero (the planner's `lower < upper` filter then drops the NaN). Documented in `geometry_2d.py`.

## 3. The port order (do not reorder)

```
io  ->  geometry  ->  roadmap  ->  ctc  ->  planners
```

| Python module | Port from C++ (`PSIPP-CTC_src/src/`) | Difficulty / notes |
|---|---|---|
| `io/` | `base/input.hpp` (roadmap text format), `base/load_problems.cpp`, then `base/input_spatial.hpp` (continuous-space + MAPF_Benchmark grid/scenario formats) and `base/spatial.hpp` in a **separate session (Stage 1b)** | **Hard contract.** `fscanf`/`printf` with `%030.15lf`. Literal `"stdin"`/`"stdout"` path sentinel. Note the known field-order asymmetry between `printTextFile` and `loadTextFile`. 1b is split out because `loadScenario` needs the geometry backend — see `docs/PORTING.md` §4. |
| `geometry/` | `base/geometry_2D.hpp`, `base/geometry_2D_for_generator.hpp`, `base/collision_report.hpp` | **Hardest math.** Symbolic continuous-time collision algebra; `Buckets2D` spatial hash + `BentleyOttoman` sweep-line broad phase. Note `Geometry2D::EPS = 0` vs `ProblemInstance::EPS = 1e-8`. |
| `roadmap/` | `roadmap_generators/{ProbabilisticRoadmap,ConstrainedTriangulation,Grid}.cpp`, `base/problem*.hpp`, `base/spatial.hpp` | kPRM and CDT (CGAL). CDT erodes the free space by the agent radius and keeps only edges with full-segment clearance. |
| `ctc/` | `base/problem_with_explicit_collisions.hpp` (+ `problem_with_grid.hpp`, `problem_with_turning.hpp`) | The offline annotation stage. No dedicated C++ dir — it is a `ProblemInstance` subclass. Decide where it lives in Python. |
| `planners/` | `planners/PrioritizedSIPP.{cpp,hpp}`, `planners/WrappedContinuousCBS2.cpp` | `PrioritizedSIPP` is the thesis core. `WrappedContinuousCBS2` shells out to the vendored CCBS via temp GraphML/XML. |

## 4. Algorithm in one page

Full detail: **`docs/ARCHITECTURE.md`**. Machine-readable graph: **`.ua/knowledge-graph.json`**.

Pipeline, in execution order:

1. **Roadmap generation** — kPRM (K-nearest, `K = ceil((e*(1+1/dim)+small_value) * log n)`,
   where `e` is Euler's number) or CDT.
   Output is an adjacency list `edges[v] = [target, ...]`, so **an edge IS its index in the
   out-list**. Edge length is time at unit speed.
2. **Offline CTC annotation** — for every roadmap vertex/edge pair, compute the *exact closed
   real-valued time interval* during which two unit-speed disc agents come within `2r`:
   - `VV` static vertex-vertex,
   - `VEC` = Vertex-Edge Conflict: stationary agent at `p0` vs agent moving `sp1 -> tp1`;
     solves `|a*t + b| <= 2r` clipped to `[0, dt1]`,
   - `EEC` = Edge-Edge Conflict: union of **five** candidate start-offset `s` ranges where
     separation is minimal (`t=0`, `t=s`, `t=dt1`, `t=s+dt0`, and the interior perpendicular case).
   Broad phase: `Buckets2D` (uniform hash, cell = `2r - EPS`, 5 neighbour offsets) for VV + VE,
   `BentleyOttoman` sweep for crossing EE. Result is **materialised into maps** — every later
   query is a lookup. That precompute is the time/memory trade the paper is about.
3. **Online prioritized SIPP** — plan agents one at a time in priority order. Each agent's
   A* state is `(vertex, safe interval)`. Blocked windows from already-planned agents are
   subtracted from per-vertex safe-interval sets (`IntervalWithID`) and per-edge blocked-window
   sets (`Interval`, no id — never an A* state, only a feasibility gate). Waits are encoded as
   `edge_id < 0`.

A plan is `vector<vector<Move>>` where `Move = {edge_id, start_time}` with a **real** `start_time`.

## 5. Non-obvious invariants (breaking these causes silent wrong answers)

- `interval_data`'s `open_listed` / `close_listed` use **`priority_order` as a generation
  counter**. There is deliberately **no per-agent clearing** of the interval structures.
  A naive port that resets these structures per agent changes behaviour.
- Blocked intervals **accumulate monotonically** across the priority loop. Each round only
  *adds* the previous agent's conflicts (`j = priorities[priority_order - 1]`); earlier agents'
  windows persist.
- `addVertexCollisionInterval` **mints a new interval id only when a window splits one
  interval into two**; a surviving interval keeps its identity.
- `heuristic_distance` is plain Euclidean — it is admissible **only** because edge length is
  reused as time (unit-speed assumption).
- Negative `edge_id` means *waiting*; `source == target` also means waiting. Both
  `CollisionCheck` and `getCollisionInterval` are **dispatchers** that normalise waiting vs
  moving agents and mirror the time frame (`lower = start_time_1 - lower`).
- `getAllVECollisions` returns **relative entry times** as `(src, edge_id, -upper, -lower)` —
  the negation and swap encode the reverse-time convention. Do not "fix" the signs.
- The annotation constructor flattens the roadmap into **global edge indices**; per-vertex
  `edge_id` must be translated before use.

## 6. The three thesis improvement targets

These are **features to improve, not bugs to replicate.** The port must stay modular so
planner policy can be swapped. In the graph these nodes are tagged `improvement-target`.

1. **Bottleneck deadlock** — hard FIFO priority can let a high-priority robot block a doorway
   forever, so the algorithm is *not complete*. See `PrioritizedSIPP::solve`'s priority loop.
   `priority_strategy` is hardcoded to `0` (FIFO) in `load_planners.cpp`; `1` is a shuffled order.
2. **Suboptimality** — low-priority robots take huge detours; sum-of-costs far from minimal.
   CCBS is the optimal baseline it is measured against.
3. **Kinematics-agnostic timing** — unit speed everywhere. `base/problem_with_turning.hpp`
   shows the intended escape hatch (graph expansion + `turning_speed`), and is itself the
   seed of thesis Phase 3 (VDA 5050 Base/Horizon, ROS 2 + Gazebo, per-robot Nav2).

## 7. How to verify parity

Docker oracle (reference build, prebuilt image, source bind-mounted read-only):

```bash
docker run --rm -v $PWD/PSIPP-CTC_src:/repo:ro tunoob4ever/2psipp:latest bash -c \
  'cd /repo && /app/build/roadmap_generation < problem_instances/spatial/sample.txt > /tmp/rm.txt \
   && /app/build/planner_benchmark < /tmp/rm.txt'
```

(The README still says `psipp-ctc_src/`; the actual directory is `PSIPP-CTC_src/`.)

Parity is not "same output bytes" — it is **`ProblemInstance::checkPlanValidity` accepting the
plan**, plus matching makespan / sum-of-costs. `checkPlanValidity` is the independent oracle:
it expands each `Move` list into timed routes with explicit waits and throws on early starts,
out-of-range edge ids, missed goals, or any pairwise segment collision. It is also where
makespan and sum-of-costs come from — **not** from the planner.

Benchmark CSV column contract (consumed by `PSIPP-CTC_src/tools/plot_*.py`):

```
problem_name, planner_name, number_of_agents, perm_id, iteration, makespan, sum_of_costs, used_time
```

`-1.0` makespan is the failure sentinel.

## 8. Gotchas in the reference config (do not assume it runs as shipped)

Found while analyzing `PSIPP-CTC_src/config/*.yaml`:

- `compare_geometric_and_explicit.yaml` — PRM blocks omit `radius` (yaml-cpp throws) and their
  `number_of_neighbors` keys are inert.
- `planning_incrementally.yaml` — driver reads `config["Roadmaps"]`, YAML supplies `Problems`.
- `roadmap_benchmark.yaml` — `SpatialPlanners:` is inert (no `addSpatialPlanner` call exists);
  default config path is missing a `/` separator.
- `load_planners.cpp:24` **hardcodes** `priority_strategy = 0`; it is not YAML-exposed.
- kPRM with `number_of_vertices: 0` means *no* random vertices — the graph is only the
  `2 * tasks` agent start/goal vertices.
- `makePlanner` accepts **only** `CCBS` and `PSIPP`; anything else throws
  `"Unknown Planner Type"`. `priority_strategy` is passed as a hardcoded `0` in the
  `PrioritizedSIPP(0, time_limit)` constructor call.
- `ExtendedIncreasingCostTreeSearch.cpp` is textually `#include`d into `load_planners.cpp`, so
  it **compiles**, but no branch ever instantiates it. It is unreachable, not a port target.
- `makeSpatialPlanner` in `load_problems.hpp` is **declared but never defined** anywhere, and
  `SpatialPlanners:` appears in no config that is actually consumed.
- `makeGenerator` accepts `PRM` / `PRMStar` / `kPRM` / `kPRMStar` / `GRID` / `CDT`; anything else
  throws `"Unknown Generator Type"`. `PRM`/`kPRM` read `radius` / `number_of_neighbors`
  unconditionally, so omitting them makes yaml-cpp throw.

## 9. Knowledge graph tooling

`.ua/` holds a generated knowledge graph for this repo (330 nodes, 681 edges, 9 layers,
12-step guided tour):

| File | Purpose |
|---|---|
| `.ua/knowledge-graph.json` | The graph. Layers, tour, per-file and per-symbol summaries. |
| `.ua/meta.json` | Commit hash + analysed file count, for incremental updates. |
| `.ua/fingerprints.json` | Structural baseline so updates stay incremental. |
| `.ua/.understandignore` | Excludes `docs/`, C++ benchmark data, vendored solver/media. |
| `.ua/config.json` | `outputLanguage: en`. |

- Visual explorer: run `/understand-dashboard` (not auto-started).
- Full writeup of the algorithm: **`docs/ARCHITECTURE.md`**.
- Regenerate or refresh: `/understand` (incremental — reuse the existing graph), or
  `/understand --full` for a clean rebuild.
- **Re-scan caveat:** `PSIPP-CTC_src/` and `docs/*.pdf|docx` are in `.gitignore`, and the
  scanner enumerates via `git ls-files`. To re-include them in a rebuild, temporarily move
  `.gitignore` aside for the duration of the scan, then restore it. Otherwise the C++ oracle
  silently drops out of the graph.
- Structure extraction has no parser for `.xml`, `.urdf`, `.pro`, or extensionless
  `CMakeLists.txt`; those files got hand-read nodes instead of auto symbols.

## 10. Repo conventions (from `README.md` — follow these)

1. Code, comments and documentation in **English**; the thesis source documents in `docs/`
   are Vietnamese. Keep it that way.
2. Keep `src/psipp_ctc/` importable as a plain Python package — the ROS 2 wrapper
   (Phase 3) will import it, not the reverse. **No ROS dependencies in Phase 1.**
3. Dev setup: `python3 -m venv .venv && . .venv/bin/activate && pip install -e . && pytest`
4. Planned CLI, mirroring the reference tools (stdin -> stdout):
   `psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt` and
   `psipp-plan < roadmap_problem.txt > plan.txt`
5. **Explain work to the owner in algorithm terms.** They know the idea of PSIPP/CTC and
   read Python, but they do not read C++ — `PSIPP-CTC_src/` is opaque to them. No "the
   C++ does X" without saying what X does to the robots, the roadmap, or the time
   intervals. Full rules in `AGENTS.md` ("How to talk to the repo owner"); the port
   playbook repeats them in `docs/PORTING.md` §1.

## 11. Suggested next actions

**The operational playbook for the port is `docs/PORTING.md`** — per-stage session
briefs, the four-check verification gate, and a 20-row trap index. Use it to brief each
port session; this section is only the short form and follows `docs/PORTING.md` §9.

1. **Close the Berlin CDT parity gap** — the only `xfail`. The port must reproduce the
   reference on every stage-brief instance before the oracle is retired. Root cause and
   the correct fix are in the note at the end of this section. **The advice previously
   recorded here ("insertion order") was wrong and has been corrected.**
2. **Port `base/benchmark.hpp` (433)** ✅ done. `src/psipp_ctc/benchmark.py` carries
   `RunPlanner`, `Benchmark`, `BenchmarkResult`, `get_used_memory_kb`,
   `display_all_results`, and `run_incrementally_all_pairs` (the CSV producer).
   `RunSpatialPlanner` and `RoadmapBenchmarkAll` stay out of scope until the spatial
   planners are ported (item 4). 9 new tests in `tests/test_benchmark.py`.
3. **Port the YAML config loaders and wire the two CLI stubs**
   (`base/load_problems.{cpp,hpp}`, `base/load_planners.{cpp,hpp}`,
   `base/load_generators.{cpp,hpp}`). `cli/planner_benchmark.py` and
   `cli/roadmap_generation.py` still `raise SystemExit("... not implemented yet")` and
   there is no YAML config module in `src/psipp_ctc/` yet. These are the last `io/` pieces.
4. **Decide the CCBS baseline strategy.** `WrappedContinuousCBS2.cpp` (781) is out of scope
   for the core port, but the thesis needs a CCBS side for the optimality-gap and
   success-rate figures. Three options: port the adapter, keep calling the vendored C++
   solver as an external binary, or merge the vendored solver's CSVs directly. This is a
   project decision, not a port session.
5. **Retire the oracle** — delete `PSIPP-CTC_src/` and remove the `/PSIPP-CTC_src/` line
   from `.gitignore` **in the same commit** (`AGENTS.md`). Not before item 1 is closed.
6. **Then the three improvement targets** in §6. That is the thesis contribution; the port
   is the baseline those changes are measured against.

### The Berlin CDT parity deviation (item 1) — root cause

Instance: `problem_instances/roadmaps/Berlin_1024_3_CDT.txt`, 100 agents, `r = 3.0`.
The Python port's A* explores a different successor than the C++ ((5966, 4) vs (5966, 0)),
and the resulting plan contains an EE collision that `check_plan_validity` rejects.

Two separate facts, do not confuse them:

- The reference's A* uses a **loose** edge-window check: it does not verify that a whole
  edge traversal fits inside one safe window. A strict "move must fit fully inside the
  window" check was tried during the port and made agent 50 unreachable — the C++ genuinely
  depends on the loose semantics. **Do not "fix" the loose check.**
- On ties in `f_value`, `std::priority_queue<Node, std::vector<Node>, cmp>` with
  `cmp = [](a,b){ return a.f_value > b.f_value; }` (`PrioritizedSIPP.cpp:201-205`) walks a
  **binary heap**, so ties pop in heap-array order, **not** insertion order. `heapq`'s
  insertion-order tie-break is a different policy, and on this instance it is what steers
  the A* into the loose check's hole.

Correct fix: mirror libstdc++'s `std::push_heap` / `std::pop_heap` on a plain list,
comparing **only** `f_value` — no `vertex`/`interval_id` tie-break fields in the key. That
reproduces the oracle image's pop order exactly and keeps the algorithm unchanged.

Do not delete `PSIPP-CTC_src/` until this is closed and the Docker oracle agrees.