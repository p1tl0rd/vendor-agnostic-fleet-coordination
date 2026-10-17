"""Tests for the ROADMAP text format.

Coverage map:
  - header: n_vertices, n_edges, n_agents
  - coordinates: %030.15f formatting and exact values
  - edges: insertion order (out-list order IS the file order)
  - agent pairs: initial_vertex, goal_vertex
  - radius: %030.15f formatting

Round-trip is the main check: load -> print -> diff against the original.
The ``sample_ref.txt`` and ``sample2_ref.txt`` files in
``problem_instances/roadmaps/`` are gold-standard reference prints.
"""

from __future__ import annotations

import io
import os
import pathlib
import subprocess
import sys
import tempfile

import numpy as np
import pytest

from psipp_ctc.io import load_roadmap_problem, print_roadmap_problem
from psipp_ctc.problem import (
    AgentTask,
    Point,
    ProblemInstanceWithGeometry,
)


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SAMPLE_DIR = REPO_ROOT / "problem_instances" / "roadmaps"
SAMPLE = SAMPLE_DIR / "sample.txt"
SAMPLE2 = SAMPLE_DIR / "sample2.txt"
# Gold-standard reference prints from the C++ reference. The
# reference prints are what a Python print of the same instance
# must match byte-for-byte.
SAMPLE_REF_PRINT = SAMPLE_DIR / "sample_ref.txt"
SAMPLE2_REF_PRINT = SAMPLE_DIR / "sample2_ref.txt"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def sample():
    return load_roadmap_problem(SAMPLE)


@pytest.fixture(scope="module")
def sample2():
    return load_roadmap_problem(SAMPLE2)


# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------


def test_sample_header(sample):
    """Header reads n_vertices=4, n_edges=6, n_agents=2."""
    assert sample.get_number_of_vertices() == 4
    assert sum(len(out) for out in sample.edges) == 6
    assert len(sample.tasks) == 2


def test_sample2_header(sample2):
    """sample2: 100 vertices, 2000 edges, 2 agents, radius 1.0."""
    assert sample2.get_number_of_vertices() == 100
    assert sum(len(out) for out in sample2.edges) == 2000
    assert len(sample2.tasks) == 2
    assert sample2.agent_geometry == 1.0


# ---------------------------------------------------------------------------
# Coordinates
# ---------------------------------------------------------------------------


def test_sample_coordinates(sample):
    """Coordinates are the 4 corners of the unit square, in id order."""
    expected = [
        Point(0.0, 0.0),
        Point(0.0, 1.0),
        Point(1.0, 0.0),
        Point(1.0, 1.0),
    ]
    assert list(sample.coordinates) == expected


def test_sample2_coordinates_first_three(sample2):
    """sample2 begins with the 4 corners of a 6x6 square, then PRM samples.

    The first 4 coordinates are the corners; after that, coordinates are
    uniform samples drawn from a PRM, so we only check the prefix here.
    The full-parsing test is the round-trip test below.
    """
    assert sample2.coordinates[0] == Point(2.0, 2.0)
    assert sample2.coordinates[1] == Point(8.0, 8.0)
    assert sample2.coordinates[2] == Point(2.0, 8.0)
    assert sample2.coordinates[3] == Point(8.0, 2.0)
    # 100 coords total; all must be Points (no parse failure).
    assert len(sample2.coordinates) == 100
    assert all(isinstance(p, Point) for p in sample2.coordinates)


# ---------------------------------------------------------------------------
# Edge insertion order
# ---------------------------------------------------------------------------


def test_sample_edges_bucket_order(sample):
    """Edges bucketed into the source's out-list in declaration order.

    sample.txt declares the 6 edges as 0->1, 1->0, 0->2, 1->3, 0->3, 1->2.
    After bucketing, edges[0] = [1, 2, 3] (positions 0, 2, 4 in the file)
    and edges[1] = [0, 3, 2] (positions 1, 3, 5). An out-list index is
    therefore the position WITHIN the source's out-list, not the position
    in the file (see HANDOFF.md §5 invariant 6, PORTING.md §3 trap 2).
    """
    assert sample.edges[0] == [1, 2, 3]
    assert sample.edges[1] == [0, 3, 2]
    assert sample.edges[2] == []
    assert sample.edges[3] == []


def test_sample2_total_edge_count_matches_header(sample2):
    """Sum of out-list sizes equals the n_edges field in the header.

    Catches "n_edges mismatch" silently loading with the wrong count.
    """
    total = sum(len(out) for out in sample2.edges)
    assert total == 2000


# ---------------------------------------------------------------------------
# Agent tasks
# ---------------------------------------------------------------------------


def test_sample_agents(sample):
    """Agent 0: 0 -> 3, agent 1: 1 -> 2."""
    assert sample.tasks == [
        AgentTask(0, 3),
        AgentTask(1, 2),
    ]


def test_sample2_agents(sample2):
    """sample2 has 2 agents whose pairs sit in the same line range as sample."""
    assert len(sample2.tasks) == 2
    # Each task's vertices must be in-range for the 100-vertex graph.
    for task in sample2.tasks:
        assert 0 <= task.initial_vertex < 100
        assert 0 <= task.goal_vertex < 100


# ---------------------------------------------------------------------------
# Agent radius
# ---------------------------------------------------------------------------


def test_sample_radius(sample):
    """sample.txt's radius is 0.05 -- a typical disc-agent size."""
    assert sample.agent_geometry == 0.05


def test_sample2_radius(sample2):
    """sample2.txt's radius is 1.0 -- a large disc."""
    assert sample2.agent_geometry == 1.0


# ---------------------------------------------------------------------------
# Print formatting -- %030.15f width and precision
# ---------------------------------------------------------------------------


def test_print_header_format(sample, tmp_path):
    """Print writes n_vertices n_edges n_agents on the first line."""
    out_path = tmp_path / "out.txt"
    print_roadmap_problem(sample, str(out_path))
    first_line = out_path.read_text().splitlines()[0]
    assert first_line == "4 6 2"


def test_print_coordinate_format_uses_030_15f(sample, tmp_path):
    """Coordinates are written with width=30, precision=15, zero-padded.

    0.0 -> "0000000000000000000000000.000000000000000" (30 chars total, 15
    after the decimal point). A bare "0.0" or "0.000000" would be wrong.
    """
    out_path = tmp_path / "out.txt"
    print_roadmap_problem(sample, str(out_path))
    text = out_path.read_text()
    lines = text.splitlines()
    # Line 0 is the header. Lines 1..4 are the coordinates.
    coord_lines = lines[1:5]
    for line in coord_lines:
        # Each line is "<x> <y>" with two fixed-width 30-char floats.
        assert len(line) == 61, f"expected 30+1+30=61, got {len(line)}: {line!r}"
        x_str, y_str = line.split(" ")
        assert len(x_str) == 30
        assert len(y_str) == 30
        # Every coord in sample is a small integer or .0; 15-digit precision
        # round-trips them exactly.
        assert x_str.endswith("000000000000000"), f"bad precision: {x_str!r}"


def test_print_radius_format_uses_030_15f(sample, tmp_path):
    """Radius line is the last line, width 30, precision 15."""
    out_path = tmp_path / "out.txt"
    print_roadmap_problem(sample, str(out_path))
    text = out_path.read_text()
    last_line = text.splitlines()[-1]
    assert len(last_line) == 30, f"radius line width: {len(last_line)}: {last_line!r}"
    # 0.05 with 15 fractional digits: 0.050000000000000
    assert last_line == "00000000000000.050000000000000"


# ---------------------------------------------------------------------------
# Round-trip -- the hard test
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("instance_name", ["sample", "sample2"])
def test_round_trip_byte_identical(instance_name, request, tmp_path):
    """Load -> print -> load -> print -> diff the two prints.

    This is the print/load cycle that the planner CLI relies on: the
    reference's ``print`` writes the long form (``%030.15lf``) and the input
    files in ``problem_instances/roadmaps/`` use the short form
    (``0.0 0.0``), so a direct diff against the original would not be empty
    even with a perfect port. The IO contract for this stage is that
    ``load(print(load(x))) == load(x)`` and that the two print outputs are
    byte-identical.

    For the cross-language byte parity check (``ref print vs py print of the
    same instance``), see ``test_python_print_matches_reference_oracle``
    below. The short-form input here exercises the parser; the second load
    + print exercises the printer.
    """
    original = SAMPLE_DIR / f"{instance_name}.txt"
    src_problem = load_roadmap_problem(str(original))

    first_print = tmp_path / f"{instance_name}.first.txt"
    print_roadmap_problem(src_problem, str(first_print))

    # Reload the long-form first print and reprint it. If the printer
    # and the loader are inverse, the second print is byte-identical to
    # the first.
    reloaded = load_roadmap_problem(str(first_print))
    second_print = tmp_path / f"{instance_name}.second.txt"
    print_roadmap_problem(reloaded, str(second_print))

    first_bytes = first_print.read_bytes()
    second_bytes = second_print.read_bytes()
    assert first_bytes == second_bytes, (
        f"print(load(x)) != print(load(print(load(x)))) for {instance_name}; "
        f"first differs at byte "
        f"{next(i for i, (a, b) in enumerate(zip(first_bytes, second_bytes)) if a != b)}"
    )


@pytest.mark.parametrize(
    "instance_name,ref_print_path",
    [("sample", SAMPLE_REF_PRINT), ("sample2", SAMPLE2_REF_PRINT)],
)
def test_python_print_matches_reference_oracle(
    instance_name, ref_print_path, tmp_path
):
    """Python print of an instance must match the C++ reference print.

    The reference files in ``problem_instances/roadmaps/sample_ref.txt`` and
    ``sample2_ref.txt`` were produced by the C++ oracle (the docker command
    documented in the module docstring of ``io/roadmap.py``). They are the
    gold standard for check #3 in PORTING.md §10.

    If this test fails, either the format string changed or the field order
    changed -- both are silent bugs that the reference text would not catch
    on its own.
    """
    src_problem = load_roadmap_problem(str(SAMPLE_DIR / f"{instance_name}.txt"))
    py_print = tmp_path / f"{instance_name}.py.txt"
    print_roadmap_problem(src_problem, str(py_print))

    py_bytes = py_print.read_bytes()
    ref_bytes = ref_print_path.read_bytes()
    assert py_bytes == ref_bytes, (
        f"Python print != C++ reference print for {instance_name} "
        f"(python {len(py_bytes)} bytes, reference {len(ref_bytes)} bytes); "
        f"first diff at byte "
        f"{next(i for i, (a, b) in enumerate(zip(py_bytes, ref_bytes)) if a != b)}"
    )


# ---------------------------------------------------------------------------
# stdin / stdout sentinels
# ---------------------------------------------------------------------------


def test_load_stdin_sentinel():
    """``load_roadmap_problem("stdin")`` reads from sys.stdin."""
    payload = "4 6 2\n0.0 0.0\n0.0 1.0\n1.0 0.0\n1.0 1.0\n" \
              "0 1\n1 0\n0 2\n1 3\n0 3\n1 2\n" \
              "0 3\n1 2\n" \
              "0.05\n"
    saved_stdin = sys.stdin
    sys.stdin = io.StringIO(payload)
    try:
        problem = load_roadmap_problem("stdin")
    finally:
        sys.stdin = saved_stdin
    assert problem.get_number_of_vertices() == 4
    assert problem.agent_geometry == 0.05


def test_print_stdout_sentinel(sample, capsys):
    """``print_roadmap_problem(p, "stdout")`` writes to sys.stdout."""
    print_roadmap_problem(sample, "stdout")
    captured = capsys.readouterr()
    # The stdout output should equal the round-trip output of the same
    # problem to a temp file -- the sentinel route is a write to sys.stdout,
    # not a special format.
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        tmp_path = f.name
    try:
        print_roadmap_problem(sample, tmp_path)
        with open(tmp_path, "r") as f:
            file_output = f.read()
    finally:
        os.unlink(tmp_path)
    assert captured.out == file_output


# ---------------------------------------------------------------------------
# Tokenisation -- newlines are not structural
# ---------------------------------------------------------------------------


def test_whitespace_is_single_delimiter(tmp_path):
    """Tabs, multiple spaces, mixed newlines all tokenise the same way.

    Mirrors fscanf: a single token is consumed by ``%d`` or ``%lf``
    regardless of how the whitespace around it is shaped. The reference
    produces this by accident; the port must preserve it deliberately.
    """
    minimal_a = "4 6 2\n0.0 0.0\n0.0 1.0\n1.0 0.0\n1.0 1.0\n" \
                "0 1\n1 0\n0 2\n1 3\n0 3\n1 2\n" \
                "0 3\n1 2\n0.05\n"
    # Same data, different whitespace: tabs, no newlines, multiple spaces,
    # a stray blank line.
    minimal_b = "4\t6\t2\n\n" \
                "0.0   0.0\n0.0\t1.0\n1.0 0.0\n1.0 1.0\n" \
                "0 1\n1 0\n0 2\n1 3\n0 3\n1 2\n" \
                "0 3\n1 2\n0.05"

    path_a = tmp_path / "a.txt"
    path_b = tmp_path / "b.txt"
    path_a.write_text(minimal_a)
    path_b.write_text(minimal_b)

    pa = load_roadmap_problem(str(path_a))
    pb = load_roadmap_problem(str(path_b))
    assert pa.get_number_of_vertices() == pb.get_number_of_vertices() == 4
    assert pa.coordinates == pb.coordinates
    assert pa.edges == pb.edges
    assert pa.tasks == pb.tasks
    assert pa.agent_geometry == pb.agent_geometry


# ---------------------------------------------------------------------------
# Bad input -- errors, not silent breakage
# ---------------------------------------------------------------------------


def test_truncated_file_raises_value_error(tmp_path):
    """A file that ends mid-section raises ValueError, not a partial instance.

    The C++ constructor with bad input leaves the instance partially built
    (or returns from the function with a NULL file). The port raises
    ``ValueError`` per PORTING.md §4 (exceptions map to exceptions).
    """
    bad = tmp_path / "bad.txt"
    bad.write_text("4 6 2\n0.0 0.0\n0.0 1.0\n")  # only 2 of 4 coords
    with pytest.raises(ValueError):
        load_roadmap_problem(str(bad))


def test_out_of_range_edge_source_raises_value_error(tmp_path):
    """An edge whose source is >= n_vertices is rejected."""
    bad = tmp_path / "bad.txt"
    # n_vertices=2, but we declare an edge from vertex 7
    bad.write_text("2 1 1\n0.0 0.0\n0.0 1.0\n7 0\n0 1\n0.5\n")
    with pytest.raises(ValueError):
        load_roadmap_problem(str(bad))
