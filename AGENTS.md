# AGENTS.md — PSIPP-CTC

Project rules for agents and humans working in this repo. Full algorithm writeup in
`docs/ARCHITECTURE.md`; current state, port order, and invariants in `HANDOFF.md`;
how to run the port, per-stage briefs, and the verification gate in `docs/PORTING.md`.

## How to talk to the repo owner

The owner is the thesis author. They know the **idea** of PSIPP/CTC, they read Python,
and they read their own Vietnamese research report in `docs/`. **They do not read C++,
and `PSIPP-CTC_src/` is opaque to them.** They stay in the loop during the port and will
ask questions.

- **Explain in algorithm terms, not code terms.** Say what happened to the robots, the
  roadmap, or the time intervals — not what the reference's classes, templates, or
  files do.
- **Never justify a decision with "the C++ does X" alone.** Follow with what X *means*:
  "the C++ stores a wait as a negative edge id — the robot stays put on a vertex."
- **Define every term the first time it appears in a session.** The paper's own terms
  (VV, VEC, EEC, safe interval) are fine once defined. Assume no knowledge of CGAL,
  boost::geometry, `fscanf`, `std::set` iterators, template specialisation, or the
  reference's abbreviations.
- **Lead progress reports with what changed in behaviour** — what a plan now looks
  like, which numbers moved — then the code location. Not the reverse.
- **To explain a trap or divergence**, use: plain description → small concrete example
  → why it changes the plan. Analogies are welcome and encouraged.
- **Code blocks are evidence, not explanation.** Pasting a diff is never a complete
  answer.
- **Answer patiently and concretely.** "It's in the C++" or a bare file path is not an
  answer. If the question reveals a misunderstanding, correct it kindly and directly.
- **When a step involves a silent trap**, say what would go wrong in the robots' plan
  if it were "fixed" — not just that the reference is unusual.

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
