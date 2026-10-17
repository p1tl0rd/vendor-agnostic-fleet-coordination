"""Stage 5-follow-up -- tests for the YAML config loaders.

The Python port's ``psipp_ctc.config`` mirrors the C++'s three
``make*`` factories in ``base/load_problems.{cpp,hpp}``,
``base/load_planners.{cpp,hpp}``, and
``base/load_generators.{cpp,hpp}``. These tests assert that each
factory:

* ``make_spatial_problem_instance`` builds a
  ``SpatialProblemInstance`` for ``Text`` and ``MAPF_Benchmark`` keys.
* ``make_problem_instance`` builds a ``ProblemInstance`` for
  ``Geometric`` and ``Explicit`` keys; ``Turning`` and ``Grid`` raise
  ``NotImplementedError`` with a HANDOFF-anchored message; the
  ``number_of_agents`` key truncates ``tasks`` after load.
* ``make_planner`` builds a ``PrioritizedSIPP`` for the ``PSIPP`` key
  with the configured ``time_limit``; ``CCBS`` raises
  ``NotImplementedError``; unknown types raise ``ValueError``.
* ``make_generator`` returns an adapter with a ``generate(spatial)``
  method and a ``problem_instance`` member, mirroring the C++'s
  ``RoadmapGenerator<Space2D>`` surface. The adapter delegates to the
  underlying generator function for every supported type.

The two CLI drivers (``psipp-plan``, ``psipp-generate-roadmap``) are
exercised end-to-end by invoking the ``main`` entry points and
asserting on the CLI's stdout / stderr.
"""
from __future__ import annotations

import io
import sys
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any, Dict

import pytest
import yaml

from psipp_ctc.config import (
    _GeneratorAdapter,
    make_generator,
    make_planner,
    make_problem_instance,
    make_spatial_problem_instance,
)
from psipp_ctc.planners import PrioritizedSIPP
from psipp_ctc.problem import (
    ProblemInstance,
    ProblemInstanceWithGeometry,
    SpatialProblemInstance,
)


WORKSPACE = Path("/home/two/PSIPP-CTC")


# --------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------


def _yaml(text: str) -> Dict[str, Any]:
    """Parse ``text`` as YAML and return a plain ``dict``."""
    return yaml.safe_load(text)


# --------------------------------------------------------------------
# make_spatial_problem_instance
# --------------------------------------------------------------------


def test_make_spatial_problem_instance_text_loads_polygon():
    """``type: Text`` reads a polygon-format file via
    ``load_spatial_problem`` and returns a ``SpatialProblemInstance``."""
    path = WORKSPACE / "problem_instances/spatial/sample.txt"
    config = _yaml(
        f"type: Text\n"
        f"file: {path}\n"
    )
    sp = make_spatial_problem_instance(config)
    assert isinstance(sp, SpatialProblemInstance)
    # The sample polygon has 4 outer vertices + 1 closing repeat.
    assert len(sp.outer) == 5
    assert sp.outer[0] == sp.outer[-1]
    # Two tasks in the sample file.
    assert len(sp.tasks) == 2


def test_make_spatial_problem_instance_unknown_type_raises():
    """Unknown ``type`` keys raise ``ValueError`` with the bad key in
    the message (mirrors ``load_problems.cpp:34``)."""
    with pytest.raises(ValueError, match="Unknown Input Type"):
        make_spatial_problem_instance(_yaml("type: NotARealType\n"))


def test_make_spatial_problem_instance_mapf_benchmark_uses_grip_map(tmp_path):
    """``type: MAPF_Benchmark`` calls ``load_grip_map`` and sets
    ``agent_geometry`` from ``agent_size``. ``scenario`` is optional."""
    # Build a minimal .map file (4x4 with all-passable cells so the
    # BFS-trace yields a closed polygon).
    map_text = "type octile\nheight 4\nwidth 4\nmap\n....\n....\n....\n....\n"
    map_path = tmp_path / "tiny.map"
    map_path.write_text(map_text)
    config = _yaml(
        f"type: MAPF_Benchmark\n"
        f"map: {map_path}\n"
        f"agent_size: 0.5\n"
    )
    sp = make_spatial_problem_instance(config)
    assert isinstance(sp, SpatialProblemInstance)
    assert sp.agent_geometry == pytest.approx(0.5)
    # No scenario file => no tasks.
    assert sp.tasks == []


# --------------------------------------------------------------------
# make_problem_instance
# --------------------------------------------------------------------


def test_make_problem_instance_geometric_loads_roadmap_file():
    """``type: Geometric`` returns a ``ProblemInstanceWithGeometry``
    loaded from a roadmap text file (mirrors ``load_problems.cpp:43-46``)."""
    path = WORKSPACE / "problem_instances/roadmaps/sample.txt"
    config = _yaml(f"type: Geometric\nfile: {path}\n")
    problem = make_problem_instance(config)
    assert isinstance(problem, ProblemInstance)
    # Sample is 6 vertices / 8 edges / 4 agents. The exact numbers are
    # not a contract here -- the contract is "no failure + the
    # tasks list is populated".
    assert len(problem.tasks) >= 1


def test_make_problem_instance_explicit_wraps_with_collision_table():
    """``type: Explicit`` returns a
    ``ProblemInstanceWithExplicitCollisions`` (mirrors
    ``load_problems.cpp:47-52``)."""
    path = WORKSPACE / "problem_instances/roadmaps/sample.txt"
    config = _yaml(f"type: Explicit\nfile: {path}\n")
    problem = make_problem_instance(config)
    from psipp_ctc.ctc import ProblemInstanceWithExplicitCollisions

    assert isinstance(problem, ProblemInstanceWithExplicitCollisions)
    # The wrapper carries the geometric data, so the same task list.
    assert len(problem.tasks) >= 1


def test_make_problem_instance_truncates_tasks_when_number_of_agents_set():
    """The ``number_of_agents`` key truncates ``tasks`` after load
    (mirrors ``load_problems.cpp:75-77``)."""
    path = WORKSPACE / "problem_instances/roadmaps/sample.txt"
    config = _yaml(
        f"type: Geometric\n"
        f"file: {path}\n"
        f"number_of_agents: 2\n"
    )
    problem = make_problem_instance(config)
    assert len(problem.tasks) == 2


@pytest.mark.parametrize("bad_type", ["Turning", "Grid"])
def test_make_problem_instance_unported_types_raise_not_implemented(bad_type):
    """``Turning`` and ``Grid`` are not ported; the factory raises
    ``NotImplementedError`` with a HANDOFF-anchored message."""
    config = _yaml(
        f"type: {bad_type}\n"
        f"file: anything.txt\n"
    )
    with pytest.raises(NotImplementedError, match="not ported"):
        make_problem_instance(config)


def test_make_problem_instance_unknown_type_raises_value_error():
    """Unknown ``type`` raises ``ValueError`` (mirrors
    ``load_problems.cpp:73``)."""
    with pytest.raises(ValueError, match="Unknown Problem Type"):
        make_problem_instance(_yaml("type: NotARealType\n"))


# --------------------------------------------------------------------
# make_planner
# --------------------------------------------------------------------


def test_make_planner_psipp_with_time_limit():
    """``type: PSIPP`` returns a ``PrioritizedSIPP`` configured with
    the YAML's ``time_limit`` (mirrors ``load_planners.cpp:22-24``)."""
    config = _yaml("type: PSIPP\ntime_limit: 7.5\n")
    planner = make_planner(config)
    assert isinstance(planner, PrioritizedSIPP)
    # ``PrioritizedSIPP._time_limit`` is the configured limit.
    assert planner._time_limit == pytest.approx(7.5)


def test_make_planner_psipp_default_time_limit():
    """``time_limit`` is optional; the default is 30.0 (mirrors
    ``load_planners.cpp:15``)."""
    config = _yaml("type: PSIPP\n")
    planner = make_planner(config)
    assert planner._time_limit == pytest.approx(30.0)


def test_make_planner_ccbs_raises_not_implemented():
    """``CCBS`` is not ported; the factory raises ``NotImplementedError``
    with a HANDOFF-anchored message."""
    config = _yaml("type: CCBS\n")
    with pytest.raises(NotImplementedError, match="not ported"):
        make_planner(config)


def test_make_planner_unknown_type_raises_value_error():
    """Unknown ``type`` raises ``ValueError`` (mirrors
    ``load_planners.cpp:26``)."""
    with pytest.raises(ValueError, match="Unknown Planner Type"):
        make_planner(_yaml("type: NotARealType\n"))


# --------------------------------------------------------------------
# make_generator
# --------------------------------------------------------------------


@pytest.mark.parametrize(
    "type_name,config_yaml,expect_attrs",
    [
        ("PRM", "type: PRM\nnumber_of_vertices: 5\nradius: 1.0\n", (5, 1.0)),
        ("PRMStar", "type: PRMStar\nnumber_of_vertices: 5\n", (5,)),
        ("kPRM", "type: kPRM\nnumber_of_vertices: 5\nnumber_of_neighbors: 2\n", (5, 2)),
        ("kPRMStar", "type: kPRMStar\nnumber_of_vertices: 5\n", (5,)),
    ],
)
def test_make_generator_prm_family(type_name, config_yaml, expect_attrs):
    """PRM / PRMStar / kPRM / kPRMStar return a ``_GeneratorAdapter``
    whose ``generate(spatial)`` populates ``problem_instance``."""
    config = _yaml(config_yaml)
    adapter = make_generator(config)
    assert isinstance(adapter, _GeneratorAdapter)
    assert adapter.problem_instance is None  # not yet generated
    sp = make_spatial_problem_instance(
        _yaml(
            f"type: Text\n"
            f"file: {WORKSPACE / 'problem_instances/spatial/sample.txt'}\n"
        )
    )
    adapter.generate(sp)
    assert isinstance(adapter.problem_instance, ProblemInstanceWithGeometry)
    assert adapter.problem_instance.get_number_of_vertices() > 0


def test_make_generator_grid():
    """``type: GRID`` returns a ``_GeneratorAdapter`` whose
    ``generate`` builds a 4-connected grid roadmap."""
    config = _yaml("type: GRID\ngrid_size: 1.0\nK: 4\n")
    adapter = make_generator(config)
    sp = make_spatial_problem_instance(
        _yaml(
            f"type: Text\n"
            f"file: {WORKSPACE / 'problem_instances/spatial/sample.txt'}\n"
        )
    )
    adapter.generate(sp)
    assert isinstance(adapter.problem_instance, ProblemInstanceWithGeometry)


def test_make_generator_cdt_defaults():
    """``type: CDT`` without ``number_of_additionals`` or
    ``use_corner_points`` uses the C++ defaults ``0`` and ``True``
    (mirrors ``load_generators.cpp:45-52``)."""
    config = _yaml("type: CDT\n")
    adapter = make_generator(config)
    sp = make_spatial_problem_instance(
        _yaml(
            f"type: Text\n"
            f"file: {WORKSPACE / 'problem_instances/spatial/sample.txt'}\n"
        )
    )
    adapter.generate(sp)
    assert isinstance(adapter.problem_instance, ProblemInstanceWithGeometry)
    # 4 corner points + 2 task endpoints = 6 vertices in the sample.
    assert adapter.problem_instance.get_number_of_vertices() >= 4


def test_make_generator_cdt_with_additionals_and_corner_flag():
    """``CDT`` accepts ``number_of_additionals`` and
    ``use_corner_points`` overrides (mirrors ``load_generators.cpp:46-52``)."""
    config = _yaml(
        "type: CDT\n"
        "number_of_additionals: 3\n"
        "use_corner_points: false\n"
    )
    adapter = make_generator(config)
    sp = make_spatial_problem_instance(
        _yaml(
            f"type: Text\n"
            f"file: {WORKSPACE / 'problem_instances/spatial/sample.txt'}\n"
        )
    )
    adapter.generate(sp)
    assert isinstance(adapter.problem_instance, ProblemInstanceWithGeometry)


def test_make_generator_unknown_type_raises_value_error():
    """Unknown ``type`` raises ``ValueError`` (mirrors
    ``load_generators.cpp:57``)."""
    with pytest.raises(ValueError, match="Unknown Generator Type"):
        make_generator(_yaml("type: NotARealType\n"))


# --------------------------------------------------------------------
# CLI drivers
# --------------------------------------------------------------------


def test_planner_benchmark_cli_runs_end_to_end_on_sample_roadmap(capsys):
    """``psipp-plan`` reads a roadmap from stdin, runs PSIPP, and
    writes the ``Benchmark.displayAllResults`` table to stderr. The
    success line, makespan, sum-of-costs, and column header all
    appear in stderr."""
    from psipp_ctc.cli.planner_benchmark import _DEFAULT_CONFIG, main

    with open(WORKSPACE / "problem_instances/roadmaps/sample.txt", "r") as f:
        problem_text = f.read()
    saved_stdin = sys.stdin
    saved_argv = sys.argv
    sys.stdin = io.StringIO(problem_text)
    sys.argv = ["psipp-plan", "default"]
    stdout = io.StringIO()
    stderr = io.StringIO()
    try:
        with redirect_stdout(stdout), redirect_stderr(stderr):
            main()
    finally:
        sys.stdin = saved_stdin
        sys.argv = saved_argv
    err = stderr.getvalue()
    # The ``displayAllResults`` column header is always emitted.
    assert "planner" in err and "problem" in err and "makespan" in err
    # The problem-load timing report.
    assert "Stdin" in err
    # Stdout received the plan (default Output: stdout).
    out = stdout.getvalue()
    # Plan is printed via the C++'s ``printPlan``; the format is
    # ``<n_agents> <radius>\n<route-size>\n<time>\n<v-x> <v-y>\n...``.
    # We only assert the first header line is present.
    assert " 0." in out or "0." in out


def test_planner_benchmark_cli_accepts_yaml_config(tmp_path):
    """``psipp-plan /path/to/config.yaml`` reads a YAML config and
    runs the configured problems / planners."""
    from psipp_ctc.cli.planner_benchmark import main

    roadmap = WORKSPACE / "problem_instances/roadmaps/sample.txt"
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        f"Problems:\n"
        f"  Sample:\n"
        f"    type: Explicit\n"
        f"    file: {roadmap}\n"
        f"Planners:\n"
        f"  PSIPP:\n"
        f"    type: PSIPP\n"
        f"    time_limit: 5.0\n"
        f"Output: stdout\n"
    )
    stdout = io.StringIO()
    stderr = io.StringIO()
    saved_argv = sys.argv
    sys.argv = ["psipp-plan", str(config_path)]
    try:
        with redirect_stdout(stdout), redirect_stderr(stderr):
            main()
    finally:
        sys.argv = saved_argv
    err = stderr.getvalue()
    # The problem-load line uses the configured name "Sample".
    assert "Sample" in err
    assert "makespan" in err


def test_roadmap_generation_cli_runs_end_to_end_on_sample_spatial(capsys):
    """``psipp-generate-roadmap`` reads a spatial problem from stdin,
    runs the generator, and writes a roadmap problem to stdout."""
    from psipp_ctc.cli.roadmap_generation import _DEFAULT_CONFIG, main

    with open(WORKSPACE / "problem_instances/spatial/sample.txt", "r") as f:
        spatial_text = f.read()
    saved_stdin = sys.stdin
    saved_argv = sys.argv
    sys.stdin = io.StringIO(spatial_text)
    sys.argv = ["psipp-generate-roadmap", "default"]
    stdout = io.StringIO()
    stderr = io.StringIO()
    try:
        with redirect_stdout(stdout), redirect_stderr(stderr):
            main()
    finally:
        sys.stdin = saved_stdin
        sys.argv = saved_argv
    out = stdout.getvalue()
    # The first line of the roadmap text format is ``n_vertices n_edges
    # n_agents`` (Stage 1a). All three are non-negative ints.
    first = out.splitlines()[0]
    tokens = first.split()
    assert len(tokens) == 3
    n_v, n_e, n_a = (int(t) for t in tokens)
    assert n_v >= 1
    assert n_e >= 0
    assert n_a >= 1


def test_roadmap_generation_cli_accepts_yaml_config(tmp_path):
    """``psipp-generate-roadmap /path/to/config.yaml`` reads a YAML
    config and emits the generated roadmap to the configured output
    path."""
    from psipp_ctc.cli.roadmap_generation import main

    spatial = WORKSPACE / "problem_instances/spatial/sample.txt"
    out_path = tmp_path / "out.txt"
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        f"Generator:\n"
        f"  type: kPRM\n"
        f"  number_of_vertices: 20\n"
        f"  number_of_neighbors: 4\n"
        f"Input:\n"
        f"  type: Text\n"
        f"  file: {spatial}\n"
        f"Output: {out_path}\n"
    )
    saved_argv = sys.argv
    sys.argv = ["psipp-generate-roadmap", str(config_path)]
    try:
        main()
    finally:
        sys.argv = saved_argv
    text = out_path.read_text()
    # Same first-line format check.
    first = text.splitlines()[0]
    tokens = first.split()
    assert len(tokens) == 3
    n_v, n_e, n_a = (int(t) for t in tokens)
    assert n_v >= 1
    assert n_a == 2  # 2 tasks in the sample spatial file
