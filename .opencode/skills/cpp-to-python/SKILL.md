---
name: cpp-to-python
description: >
  Port C++ code from the PSIPP-CTC_src oracle to Python in src/psipp_ctc/ with
  behavioural and numerical equivalence, not syntactic transliteration. Use when
  writing, porting, reviewing, or fixing any Python module that mirrors
  PSIPP-CTC_src/src/ (io, geometry, roadmap, ctc, planners, CLIs), when touching
  the text I/O formats, the collision-interval algebra, the safe-interval A*, or
  when validating a port against the C++ reference.
license: MIT
compatibility: opencode
metadata:
  scope: src/psipp_ctc/**
  oracle: PSIPP-CTC_src/**
---

# C++ → Python port rules (PSIPP-CTC)

Goal: Python in `src/psipp_ctc/` answers identically to the C++ oracle in
`PSIPP-CTC_src/`. Equivalence means `checkPlanValidity` accepts the plan and
makespan / sum-of-costs match — not identical bytes, not "cleaner" code.

Read before touching a module: `HANDOFF.md` (state, port order, invariants) and
`docs/ARCHITECTURE.md` (algorithm). `PSIPP-CTC_src/` is read-only ground truth.

## Rules

1. **Same algorithm, same order of operations.** Translate the C++ control flow
   line group by line group. No vectorising, no early exits, no reordering of
   float arithmetic, no "obvious" cleanups. Correctness first, speed later.
2. **Preserve float expression grouping.** `(a+b)*c` and `a + b*c` differ in the
   last ulp. C++ evaluates what it evaluates; write the same expression.
3. **Preserve comparison strictness.** `<` vs `<=`, `EPS`-inclusive vs exclusive.
   `Geometry2D::EPS = 0` and `ProblemInstance::EPS = 1e-8` are two different
   constants in the original. Keep both, keep their names, do not unify them.
4. **Preserve container ordering.** `std::set` / `std::map` are sorted by
   `operator<`; iteration order is part of observable behaviour. Python `set` /
   `dict` are unordered. Use `sortedcontainers.SortedList`, `bisect`, or a
   sorted list of keys — never a bare `set` where the C++ had a `std::set`.
5. **Preserve interval identity.** `IntervalWithID::operator<` compares `start`
   only, so intervals with equal `start` are ordered arbitrarily by insertion
   order in `std::set` (and duplicates are dropped). Reproduce with
   `bisect.insort`-style insertion on a start-sorted list; do not switch to
   `(start, end)` tuple keys.
6. **`edge_id` is an index into the out-list, and negative means waiting.**
   `edges[v] = [target, ...]`, so edge identity is positional. `edge_id < 0` and
   `source == target` both encode a wait. Never renumber edges.
7. **`priority_order` doubles as a generation counter** for `open_listed` /
   `close_listed`. There is no per-agent clearing of interval structures and
   blocked windows accumulate monotonically across the priority loop. A port
   that resets per agent is wrong even when it passes small tests.
8. **Only the previous agent's conflicts are added each round**
   (`j = priorities[priority_order - 1]`). Earlier agents' windows persist.
9. **Do not "fix" sign conventions.** `getAllVECollisions` returns relative
   entry times as `(src, edge_id, -upper, -lower)`. The negation plus swap is
   the reverse-time convention. Dispatchers mirror time frames with
   `lower = start_time_1 - lower`. Reproduce verbatim.
10. **Annotations use global edge indices**; per-vertex `edge_id` must be
    translated before lookup. Keep the translation in one helper.
11. **I/O is a hard contract.** `fscanf`-based readers, `%030.15lf` output
    formatting, and the literal `"stdin"` / `"stdout"` path sentinels. Reproduce
    field order exactly, including the known `printTextFile` / `loadTextFile`
    asymmetry in `base/input.hpp` — verify round-trip against the oracle rather
    than "fixing" it.
12. **Exceptions map to exceptions.** C++ `throw` → `raise` a specific
    exception type (`ValueError` for bad input, `RuntimeError` for planner
    failure). Never swallow, never return sentinel values where C++ threw,
    unless the C++ also did that.
13. **Sentinels stay sentinels.** `INF = inf`, `EPS = 1e-8`, makespan failure
    `-1.0`, `open_listed = -1`, `prev_edge_id = -1`.
14. **Randomness is unseeded in the oracle.** `Space2D::getRandomPoints` uses
    `std::random_device` + `std::default_random_engine`, so the C++ is not
    reproducible run to run. Python may use `numpy.random.default_rng()`, but
    never assert bit-identical sample points. Parity tests must compare
    *distributional or structural* properties, not exact coordinates.
15. **No C++-isms in the public API.** No `ctypes`, no `pybind11`, no importing
    anything from `PSIPP-CTC_src/`. The Python tree stays a pure package
    (`src/psipp_ctc`) importable without ROS, without CGAL, without the oracle.
16. **English only** in code, comments, docstrings, and commit messages. The
    `docs/` thesis sources are Vietnamese; keep them that way.
17. **Note every deviation.** If a port must diverge from the C++ to be correct or
    fast, say so in a comment at the divergence site and in the PR description.

## Type and API mapping

| C++ | Python | Note |
|---|---|---|
| `int`, `long` | `int` | Arbitrary precision. C++ wraps / is UB on overflow. Not an issue here, but never rely on it. |
| `double`, `float` | `float` | Both are C `double` in CPython. `float` == C++ `double`. |
| `bool` | `bool` | — |
| `std::string` | `str` | — |
| `std::vector<T>` | `list[T]` | Fixed-size numeric data may be `numpy.ndarray`, but only if the C++ was also index-arithmetic-heavy. |
| `std::set<T>`, `std::map<K,V>` | `SortedList` / `dict` over sorted keys | **Ordering is observable.** Plain `set`/`dict` are wrong substitutes. |
| `std::unordered_map<K,V>` | `dict` | Iteration order differs. Only safe if C++ never depended on it — check. |
| `T*`, `unique_ptr<T>`, `shared_ptr<T>` | object reference or `None` | Auto lifetime via GC. Explicit `delete` has no counterpart. |
| `std::optional<T>` | `T \| None` | — |
| `Point {double x, y}` | `tuple[float, float]` or `NamedTuple` | Prefer `NamedTuple` so `.x` / `.y` survive. |
| `struct` with `operator<` | `NamedTuple` + explicit sort key | Or `dataclass(order=True)` only if the field order matches `operator<`. |
| `std::priority_queue` | `heapq` | Check the comparator direction — C++ default is max-heap, `heapq` is min-heap. |
| `std::chrono::duration` | `float` seconds | — |
| `Eigen::Matrix` | `numpy.ndarray` | Not present in the oracle; applies to thesis-side work only. |
| `cv::Mat` | `numpy.ndarray` + `cv2` | Phase 3 only. |
| `rclcpp::Node` | `rclpy.node.Node` | Phase 3 only. No ROS imports in Phase 1. |
| `boost::geometry` | `shapely` (or exact reimplementation) | CDT offsets obstacle rings inward by agent radius; shapely's `buffer` sign matters. |
| CGAL `Constrained_Delaunay_triangulation_2` | `scipy.spatial.Delaunay` + constraint filtering, or `triangle` | The port may only approximate CDT; document how, and validate edge validity against the oracle. |
| `std::mutex` / `lock_guard` | `threading.Lock` (`with`) | Not needed in Phase 1 — the planner is single-threaded. |

## Verification

Parity oracle, reference build bind-mounted read-only:

```bash
docker run --rm -v $PWD/PSIPP-CTC_src:/repo:ro tunoob4ever/2psipp:latest bash -c \
  'cd /repo && /app/build/roadmap_generation < problem_instances/spatial/sample.txt > /tmp/rm.txt \
   && /app/build/planner_benchmark < /tmp/rm.txt'
```

Per module:

- **io** — round-trip every instance in `PSIPP-CTC_src/problem_instances/`.
  Byte-compare the roadmap text. Copy samples into `problem_instances/`.
- **geometry** — unit-test `getCollisionInterval` EEC/VEC against hand-computed
  intervals and against the oracle. This is the single most important routine in
  the system; nothing gets built on top until it is right.
- **roadmap** — compare vertex count, out-adjacency, and edge lengths. For
  randomised generators compare counts and validity, not coordinates (rule 14).
- **ctc** — compare the number of annotated VV / VEC / EEC pairs and the exact
  interval endpoints with `rtol=1e-9, atol=1e-12`.
- **planners** — compare `checkPlanValidity` acceptance, makespan, sum-of-costs,
  and the CSV row contract
  `problem_name, planner_name, number_of_agents, perm_id, iteration, makespan, sum_of_costs, used_time`.

Test style:

```python
import numpy.testing as npt

def test_eec_matches_oracle():
    out = get_collision_interval_eec(p0, t0, p1, t1, r)
    npt.assert_allclose(out, EXPECTED, rtol=1e-9, atol=1e-12)
```

Integers, strings, booleans, and counts compare with `==`. Timings and geometry
compare with `numpy.testing.assert_allclose`. `-1.0` makespan compares with `==`.
Never use a loose tolerance to paper over an algorithmic divergence — if the
difference is large, the port is wrong, not the tolerance.

## Not port targets

Do not port these; they are dead or third-party code.

- `PSIPP-CTC_src/third_party/Continuous-CBS/**` — vendored, unmodified. Keep
  `WrappedContinuousCBS2`'s file-level coupling in mind only.
- `src/planners/ExtendedIncreasingCostTreeSearch.cpp` — textually `#include`d so
  it compiles, but never instantiated. Unreachable.
- `makeSpatialPlanner` in `base/load_problems.hpp` — declared, never defined.
- `SpatialPlanners:` in any YAML — inert, no consumer.
- `priority_strategy` — `load_planners.cpp:24` hardcodes `0`. Not YAML-exposed.
  Expose it in Python only as a documented extension, default `0`.

## Port order

Do not reorder, and do not start a later module before the earlier one is
tested.

```
io  ->  geometry  ->  roadmap  ->  ctc  ->  planners
```

## Checklist

- [ ] Every C++ branch, loop bound, and early return has a Python counterpart.
- [ ] Float expression grouping and comparison strictness preserved.
- [ ] No bare `set` / `dict` where the C++ used `std::set` / `std::map`.
- [ ] `EPS` constants not unified; `INF` = `inf`; failure sentinels intact.
- [ ] Edge ids positional, negative id = wait, global-index translation present.
- [ ] Generation-counter and monotone-accumulation invariants intact.
- [ ] Dispatcher time-frame mirroring (`start_time_1 - lower`) preserved.
- [ ] `getAllVECollisions` negation and swap preserved.
- [ ] Text I/O byte-compatible, including the known field-order asymmetry.
- [ ] Exceptions raised, not swallowed.
- [ ] Deviates only where documented, each with a comment at the site.
- [ ] `pytest` green; oracle comparison run and recorded in the PR.
- [ ] English prose, `pip install -e . && pytest` documented if setup changed.
