# PSIPP-CTC (Python port)

Python implementation of **Prioritized Safe Interval Path Planning with
Continuous-Time Conflict annotation** (PSIPP-CTC), based on:

> Kazumi Kasaura, Mai Nishimura, and Ryo Yonetani. "Prioritized Safe Interval
> Path Planning for Multi-Agent Pathfinding With Continuous Time on 2D
> Roadmaps." IEEE Robotics and Automation Letters 7.4 (2022): 10494-10501.

Status: **skeleton only** — planning code is being ported. The original C++
reference lives in `psipp-ctc_src/` (git-ignored, to be removed once the port
is complete).

## Layout

```
.
├── config/                  # YAML configuration files
├── docs/                    # papers and notes (git-ignored files)
├── experiments/             # benchmark scripts and results
├── problem_instances/       # sample problems (spatial + roadmap formats)
├── psipp-ctc_src/           # C++ reference source (git-ignored)
├── src/psipp_ctc/           # Python package
│   ├── cli/                 # command-line entry points
│   ├── geometry/            # geometry, spatial primitives, collision checks
│   ├── io/                  # problem/plan file format parsers and writers
│   ├── planners/            # PSIPP, CCBS and related planners
│   └── roadmap/             # roadmap generation (kPRM, CDT)
└── tests/                   # unit tests
```

## Install (dev)

```
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
```

## Planned usage

```
psipp-generate-roadmap < spatial_problem.txt > roadmap_problem.txt
psipp-plan < roadmap_problem.txt > plan.txt
```

## ROS 2 (later)

A ROS 2 (colcon) package can wrap `psipp_ctc` when the port is stable; no
decision yet on where that package will live.
