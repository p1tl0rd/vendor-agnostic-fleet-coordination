# AGENTS.md — PSIPP-CTC

Project rules for agents and humans working in this repo. Full algorithm writeup in
`docs/ARCHITECTURE.md`; current state, port order, and invariants in `HANDOFF.md`.

## C++ → Python porting

**When a task involves porting C++ to Python** — writing, reviewing, or fixing any
module in `src/psipp_ctc/` that mirrors `PSIPP-CTC_src/src/`, touching the text I/O
formats, the collision-interval algebra, the safe-interval A*, or validating a port
against the reference — load the `cpp-to-python` skill first:

```
skill({ name: "cpp-to-python" })
```

It carries the mapping tables (C++ container → Python ordered structure, `operator<`
semantics, float grouping, exception and sentinel mapping), the repo-specific traps
(edge id = out-list index, negative id = wait, `priority_order` as generation counter,
monotone blocked-interval accumulation, unseeded RNG in the oracle), the Docker
parity-oracle commands, and the per-module verification checklist.

`PSIPP-CTC_src/` is read-only ground truth. Delete it only after the port is verified,
and remove the `/PSIPP-CTC_src/` line from `.gitignore` in the same commit.

## Hard rules

1. **Behavioural equivalence over style.** Python must match the C++ oracle's plan
   validity, makespan, and sum-of-costs. No algorithmic "improvements" during a port.
2. **Port order is `io -> geometry -> roadmap -> ctc -> planners`.** Do not reorder, and
   do not build on an untested module.
3. **`src/psipp_ctc/` stays a plain Python package.** No ROS, no CGAL, no `pybind11`,
   no `ctypes`, no imports from `PSIPP-CTC_src/`. The Phase 3 ROS 2 wrapper will import
   this package, not the reverse.
4. **English only** in code, comments, docstrings, docs, and commits. The thesis sources
   in `docs/` are Vietnamese; leave them that way.
5. **Document every deviation** from the C++ with a comment at the divergence site.
6. **Verify against the oracle**, not by eyeballing. `pytest` green is necessary, not
   sufficient.

## Setup and checks

```bash
python3 -m venv .venv && . .venv/bin/activate && pip install -e . && pytest
```

CLIs mirror the reference tools, stdin to stdout:

```
psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt
psipp-plan < roadmap_problem.txt > plan.txt
```

`problem_instances/` is git-ignored and empty; copy samples out of
`PSIPP-CTC_src/problem_instances/` to populate it.
