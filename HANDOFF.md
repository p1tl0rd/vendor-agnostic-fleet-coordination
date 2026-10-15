# HANDOFF — PSIPP-CTC

**Read this first if you are a new agent (or a returning human) on this repo.**

Last updated: 2026-10-05 · Knowledge graph built from commit `8a4ebcb` · 108 files analyzed

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
| `src/psipp_ctc/**` | **Skeleton only.** 4 empty subpackages + 2 CLI stubs that `raise SystemExit("not implemented yet")`. 1 smoke test asserting `__version__`. | The deliverable. Everything here still needs writing. |
| `PSIPP-CTC_src/**` | Complete, working C++14 implementation (OMRON SINIC X, MIT). | Ground truth. Read-only. **Delete after the port completes** (and remove the `/PSIPP-CTC_src/` line from `.gitignore` then). |
| `PSIPP-CTC_src/third_party/Continuous-CBS/**` | Vendored CCBS baseline (unmodified). | Optimality/scalability comparison point. |
| `docs/` | Thesis `.docx` + 9 papers. Git-ignored. | Read for requirements, not for code. |

**Nothing in Phase 1 of the README roadmap is implemented yet.** No porting has started.

## 3. The port order (do not reorder)

```
io  ->  geometry  ->  roadmap  ->  ctc  ->  planners
```

| Python module | Port from C++ (`PSIPP-CTC_src/src/`) | Difficulty / notes |
|---|---|---|
| `io/` | `base/input.hpp` (roadmap text format), `base/input_spatial.hpp` (continuous-space + MAPF_Benchmark grid/scenario formats), `base/load_problems.cpp` | **Hard contract.** `fscanf`/`printf` with `%030.15lf`. Literal `"stdin"`/`"stdout"` path sentinel. Note the known field-order asymmetry between `printTextFile` and `loadTextFile`. |
| `geometry/` | `base/geometry_2D.hpp`, `base/geometry_2D_for_generator.hpp`, `base/collision_report.hpp` | **Hardest math.** Symbolic continuous-time collision algebra; `Buckets2D` spatial hash + `BentleyOttoman` sweep-line broad phase. Note `Geometry2D::EPS = 0` vs `ProblemInstance::EPS = 1e-8`. |
| `roadmap/` | `roadmap_generators/{ProbabilisticRoadmap,ConstrainedTriangulation,Grid}.cpp`, `base/problem*.hpp`, `base/spatial.hpp` | kPRM and CDT (CGAL). CDT offsets obstacle rings inward by the agent radius. |
| `ctc/` | `base/problem_with_explicit_collisions.hpp` (+ `problem_with_grid.hpp`, `problem_with_turning.hpp`) | The offline annotation stage. No dedicated C++ dir — it is a `ProblemInstance` subclass. Decide where it lives in Python. |
| `planners/` | `planners/PrioritizedSIPP.{cpp,hpp}`, `planners/WrappedContinuousCBS2.cpp` | `PrioritizedSIPP` is the thesis core. `WrappedContinuousCBS2` shells out to the vendored CCBS via temp GraphML/XML. |

## 4. Algorithm in one page

Full detail: **`docs/ARCHITECTURE.md`**. Machine-readable graph: **`.ua/knowledge-graph.json`**.

Pipeline, in execution order:

1. **Roadmap generation** — kPRM (K-nearest, `K = ceil((e*(1+1/dim)+eps) * log n)`) or CDT.
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

## 11. Suggested next actions

**The operational playbook for the port is `docs/PORTING.md`** — per-stage session
briefs, the four-check verification gate, and a 15-row trap index. Use it to brief each
port session; this section is only the short form.

1. Port `io/` first and lock it down with round-trip tests against real reference instances
   (copy samples out of `PSIPP-CTC_src/problem_instances/` into `problem_instances/`).
   Note: `docs/PORTING.md` §1 inserts **Stage 0 = data model** before `io`, because
   `base/input.hpp` is a template specialisation of `ProblemInstanceWithGeometry`.
2. Then `geometry/` — the `getCollisionInterval` EEC kernel is the single most important
   routine in the whole system; unit-test it against hand-computed intervals before building
   anything on top. Reuse the reference's own property tests
   (`PSIPP-CTC_src/src/test/geometry_2D_test.cpp`).
3. Keep the reference available and re-run the Docker oracle after each stage.
4. Do not delete `PSIPP-CTC_src/` until the port is verified.