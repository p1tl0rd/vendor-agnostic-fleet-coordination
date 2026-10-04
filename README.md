# PSIPP-CTC — Vendor-Agnostic Robot Fleet Coordination (Master's Thesis)

This is the working codebase for a **master's thesis**. It is not only a Python
port of a paper: the PSIPP/CTC port is the planning core of a larger fleet
coordination system.

**Thesis title (expected):** *Building a vendor-agnostic multi-robot fleet
coordination system using the VDA 5050 Base/Horizon mechanism and the PSIPP
algorithm.*

## Aim and motivation

Industrial AGV/AMR fleets are vendor-locked: each manufacturer ships proprietary
fleet software, so robots from different vendors cannot share one workspace.
Classic MAPF is a poor fit too: it discretizes time and assumes robot
acceleration/braking is instantaneous, while a heterogeneous fleet has very
different kinematics per vehicle type.

The thesis builds a **central Master Control** that coordinates heterogeneous
robots from any vendor via **VDA 5050 over MQTT**, planning with
**continuous-time MAPF (PSIPP/CTC) on 2D roadmaps**:

1. **Planning engine** (this repo): PSIPP/CTC — roadmap generation (kPRM/CDT),
   offline conflict annotation (CTC), online prioritized safe-interval path
   planning in continuous time.
2. **Order model — VDA 5050 Base/Horizon**: every order splits into
   - **Base**: released segments (`released: true`) that are absolutely safe to
     drive; Base ends at the **Decision Point**.
   - **Horizon**: the planned but not yet released continuation
     (`released: false`), sent ahead so robot controllers can plan smooth
     dynamics instead of emergency braking.
3. **Validation — ROS 2 + Gazebo**: a warehouse world with a heterogeneous fleet
   (e.g. TurtleBot3 differential-drive and an Ackermann forklift, each with its
   own URDF/Xacro), a per-robot **ROS 2 Nav2** stack for local planning/control,
   and a **`vda5050_connector`** adapter on each robot (MQTT bridge +
   controller) that translates Base into Nav2 goals and streams
   State/odometry back to the master.

## Research gap (thesis contribution)

PSIPP/CTC is extremely fast (up to ~2000 agents in ~30 s vs. CCBS falling over
at ~100) but the source paper acknowledges hard limitations:

- **Deadlock at bottlenecks** — hard FIFO priority can let a high-priority robot
  block a single doorway forever → algorithm is not complete.
- **Suboptimal** — low-priority robots are forced into very long detours, so
  sum-of-costs is far from minimal.
- **Kinematics-agnostic** — priority order and continuous timing do not account
  for per-robot speed/acceleration/turning limits of a heterogeneous fleet.

The thesis targets these limitations (adaptive priority / yielding / negotiation
between robots, mobility-aware planning). Read the reports in `docs/` for the
full problem statement.

## Key documents (`docs/`, git-ignored as reference material)

| File | Content |
| --- | --- |
| `BÁO CÁO NGHIÊN CỨU_ PRIORITIZED SAFE INTERVAL PATH PLANNING WITH CONTINUOUS-TIME CONFLICTS (PSIPP_CTC).docx` | Vietnamese research report: pipeline, SIPP/prioritized-planning/geometry theory, CTC (VEC/EEC), evaluation and limitations |
| `Hệ thống điều phối bầy đàn robot dị chủng (Vendor-Agnostic).docx` | Thesis proposal: vendor lock-in problem, VDA 5050 Base/Horizon, PSIPP integration, ROS 2 + Gazebo simulation plan |
| `Prioritized Safe Interval Path Planning for Multi-Agent Pathfinding With Continuous Time on 2D Roadmaps.pdf` | The source paper (Kasaura, Nishimura, Yonetani, RA-L 2022) |
| other `*.pdf` | Related work (CCBS, MAPF continuous time, SIPP, prioritization) |

`.docx` can be read with `soffice --headless --convert-to txt:Text --outdir <dir> <file>`.

## Repository layout

```
.
├── config/                  # YAML configuration (port of reference config files)
├── docs/                    # thesis documents + papers (git-ignored contents)
├── experiments/             # benchmark scripts and results
├── problem_instances/       # sample problems (spatial + roadmap formats)
├── psipp-ctc_src/           # original C++14 implementation — read-only reference (git-ignored)
├── src/psipp_ctc/
│   ├── cli/                 # psipp-generate-roadmap, psipp-plan entry points
│   ├── geometry/            # points, polygons, AABB/sweep-line, FRNN, Bentley-Ottmann
│   ├── io/                  # problem/plan file format parse + write
│   ├── planners/            # PSIPP/CTC, CCBS baseline
│   └── roadmap/             # kPRM, CDT generators
└── tests/                   # pytest suites
```

## Status / roadmap

- [x] Repository skeleton (Python package, CLI stubs, tests, config)
- [ ] **Phase 1 — Python port of PSIPP/CTC pipeline**
  - [ ] `io`: continuous-space problem, roadmap problem, plan formats
  - [ ] `geometry`: polygon ops, collision primitives (AABB prefilter, FRNN, Bentley-Ottmann)
  - [ ] `roadmap`: kPRM and CDT generators
  - [ ] `ctc`: offline VEC/EEC annotation
  - [ ] `planners`: continuous-time PSIPP; CCBS baseline for comparison
- [ ] **Phase 2 — VDA 5050 layer**: Base/Horizon order builder, MQTT master control
- [ ] **Phase 3 — ROS 2 + Gazebo**: heterogeneous fleet simulation, Nav2 per robot, `vda5050_connector`
- [ ] **Phase 4 — Thesis experiments**: bottleneck/deadlock and optimality improvements vs. baseline

## Reference implementation and parity oracle

`PSIPP-CTC_src/` contains the original C++14 implementation (OMRON SINIC X,
MIT-licensed; see its `README.md` for file formats and config parameters). It is
**read-only reference**: the port must reproduce its text I/O formats and
planner behavior. It will be deleted once the port is complete (also remove the
`/PSIPP-CTC_src/` line from `.gitignore` then).

A working Docker image of the reference build exists — use it as a behavior
oracle when porting:

```
docker run --rm -v $PWD/psipp-ctc_src:/repo:ro tunoob4ever/2psipp:latest bash -c \
  'cd /repo && /app/build/roadmap_generation < problem_instances/spatial/sample.txt > /tmp/rm.txt \
   && /app/build/planner_benchmark < /tmp/rm.txt'
```

Reference sample instances live in `psipp-ctc_src/problem_instances/` (spatial +
roadmaps); copy what is needed into `problem_instances/` when tests are ported.

## Development

```
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
pytest
```

Planned CLI (mirrors the reference tools, stdin → stdout):

```
psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt
psipp-plan < roadmap_problem.txt > plan.txt
```

## Notes for AI agents working on this repo

1. **Read the two `.docx` files in `docs/` first** — they define the thesis aim
   and system architecture; this README is the summary.
2. Port order: `io` → `geometry` → `roadmap` → `ctc` → `planners`. Keep the
   reference file formats and CLI behavior; check parity against
   `PSIPP-CTC_src/` (paper) and the Docker oracle above.
3. Keep `src/psipp_ctc/` importable as a plain Python package: the ROS 2 wrapper
   (Phase 3) will call it, not the reverse. No ROS dependencies in Phase 1.
4. Known PSIPP weaknesses (bottleneck deadlock, non-optimality, hard FIFO
   priority) are *features to improve*, not bugs to replicate — keep the port
   modular so planner policy (priority ordering, yielding) can be swapped.
5. Language convention: code, comments, and documentation are English; the
   source thesis documents in `docs/` are written in Vietnamese.
