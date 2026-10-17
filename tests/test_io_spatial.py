"""Tests for the SPATIAL problem formats.

Coverage map:
  load_spatial_problem:
    - ring closure (first point repeated)
    - hole count
    - agent geometry + task parsing
  load_mapfb_grid:
    - header parsing (type, height, width, map)
    - padding (one impassable border cell on every side)
    - free chars ('.', 'G', 'S')
    - the single-component flood fill (isolated pockets are dropped)
  load_grip_map:
    - outer ring closure
    - the -1 shift to undo the padding
    - inner rings present when the map has internal obstacles
  load_mapfb_scenario:
    - row count
    - coordinate parsing
"""

from __future__ import annotations

import pathlib

import pytest

from psipp_ctc.io import (
    load_grip_map,
    load_mapfb_grid,
    load_mapfb_scenario,
    load_spatial_problem,
)
from psipp_ctc.io.spatial import _open_for_read  # noqa: F401  (import-time check)
from psipp_ctc.problem import Point, SpatialTask


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SPATIAL_DIR = REPO_ROOT / "problem_instances" / "spatial"
SAMPLE = SPATIAL_DIR / "sample.txt"
EMPTY256 = SPATIAL_DIR / "empty256.txt"
ARENA = SPATIAL_DIR / "arena.map"


# ---------------------------------------------------------------------------
# load_spatial_problem -- the continuous-space polygon format
# ---------------------------------------------------------------------------


class TestLoadSpatialProblem:
    """The hand-written polygon format.

    Format reminder: n_holes n_agents, n_outer_vertices, V points, [per
    hole: n_inner_vertices, points], agent_geometry, [per agent: ix iy
    gx gy]. Rings are closed by repeating the first point.
    """

    def test_sample_outer_ring_count(self):
        """sample.txt has 4 outer vertices -> closed ring of 5 points."""
        p = load_spatial_problem(str(SAMPLE))
        assert len(p.outer) == 5
        # The 4 declared vertices are the corners of a 10x10 square.
        assert p.outer[0] == Point(0.0, 0.0)
        assert p.outer[1] == Point(0.0, 10.0)
        assert p.outer[2] == Point(10.0, 10.0)
        assert p.outer[3] == Point(10.0, 0.0)

    def test_sample_outer_ring_is_closed(self):
        """The outer ring repeats the first point at the end."""
        p = load_spatial_problem(str(SAMPLE))
        assert p.outer[-1] == p.outer[0]

    def test_sample_hole_count(self):
        """sample.txt has 0 holes."""
        p = load_spatial_problem(str(SAMPLE))
        assert p.inners == []
        assert len(p.inners) == 0

    def test_sample_agent_geometry(self):
        """sample.txt's agent radius is 1.0."""
        p = load_spatial_problem(str(SAMPLE))
        assert p.agent_geometry == 1.0

    def test_sample_tasks(self):
        """sample.txt has 2 tasks: (2,2)->(8,8) and (2,8)->(8,2)."""
        p = load_spatial_problem(str(SAMPLE))
        assert p.tasks == [
            SpatialTask(Point(2.0, 2.0), Point(8.0, 8.0)),
            SpatialTask(Point(2.0, 8.0), Point(8.0, 2.0)),
        ]

    def test_empty256_outer_ring_count(self):
        """empty256.txt: 4 outer vertices, 0 holes, 0 agents, radius 0.5."""
        p = load_spatial_problem(str(EMPTY256))
        assert len(p.outer) == 5
        assert p.outer[0] == Point(0.0, 0.0)
        assert p.outer[1] == Point(0.0, 256.0)
        assert p.outer[2] == Point(256.0, 256.0)
        assert p.outer[3] == Point(256.0, 0.0)
        assert p.outer[-1] == p.outer[0]

    def test_empty256_no_holes_no_tasks(self):
        p = load_spatial_problem(str(EMPTY256))
        assert p.inners == []
        assert p.tasks == []
        assert p.agent_geometry == 0.5

    def test_holes_close_their_rings(self, tmp_path):
        """A polygon with one hole; the hole ring is closed too."""
        # outer = unit square, 1 hole = a 0.4x0.4 square in the middle
        path = tmp_path / "hole.txt"
        path.write_text(
            "1 0\n"            # 1 hole, 0 agents
            "4\n"              # 4 outer vertices
            "0. 0.\n"
            "0. 1.\n"
            "1. 1.\n"
            "1. 0.\n"
            "4\n"              # 4 inner vertices
            "0.3 0.3\n"
            "0.3 0.7\n"
            "0.7 0.7\n"
            "0.7 0.3\n"
            "0.05\n"           # radius
        )
        p = load_spatial_problem(str(path))
        assert len(p.outer) == 5
        assert p.outer[-1] == p.outer[0]
        assert len(p.inners) == 1
        assert len(p.inners[0]) == 5
        assert p.inners[0][-1] == p.inners[0][0]
        assert p.tasks == []


# ---------------------------------------------------------------------------
# load_mapfb_grid -- octile .map -> padded free-cell grid
# ---------------------------------------------------------------------------


class TestLoadMAPFBGrid:
    """Octile ``.map`` format. Padded (H+2) x (W+2) with one impassable
    border. Only one connected component of free cells is kept."""

    def test_arena_grid_is_padded(self):
        """arena.map is 49x49; the loaded grid is 51x51 with one border."""
        grid = load_mapfb_grid(str(ARENA))
        assert len(grid) == 51
        assert len(grid[0]) == 51

    def test_arena_border_is_impassable(self):
        """Every cell on the border row/column is an obstacle."""
        grid = load_mapfb_grid(str(ARENA))
        for x in range(51):
            assert grid[0][x] is False, f"top border at x={x}"
            assert grid[50][x] is False, f"bottom border at x={x}"
        for y in range(51):
            assert grid[y][0] is False, f"left border at y={y}"
            assert grid[y][50] is False, f"right border at y={y}"

    def test_arena_free_chars(self):
        """At least one cell inside the border is free."""
        grid = load_mapfb_grid(str(ARENA))
        any_free = any(grid[y][x] for y in range(1, 50) for x in range(1, 50))
        assert any_free

    def test_arena_total_free_count_after_floodfill(self):
        """The kept component's free-cell count is reasonable for arena.

        arena.map is mostly free inside the T border except for a few
        internal T columns. The flood fill keeps the LARGEST component,
        which for arena is most of the free space. We don't pin a
        specific number (the tracer might decide differently from the
        C++), but it must be a substantial fraction of the interior.
        """
        grid = load_mapfb_grid(str(ARENA))
        free = sum(
            1 for y in range(1, 50) for x in range(1, 50) if grid[y][x]
        )
        # Interior is 49*49 = 2401 cells; the kept component is the
        # majority of them (the C++ and Python both flood-fill from the
        # top-left free cell, which catches the whole connected region).
        # We assert at least 50% to be robust to small differences in
        # how the tracer classifies interior T columns.
        assert free >= 1200, f"free count {free} is suspiciously small"

    def test_flood_fill_drops_isolated_pocket(self, tmp_path):
        """A map with two disconnected free components keeps only the
        first one in scan order (PORTING.md §4 trap 16).

        Layout (5x5 interior):
          row 0: .....   <- top component
          row 1: .....
          row 2: FFFFF   <- horizontal wall, separates the two
          row 3: .....   <- bottom component (the "pocket")
          row 4: .....
        The flood fill seeds at the top-left free cell and fills the
        top component; the bottom component is dropped.
        """
        path = tmp_path / "two_components.map"
        path.write_text(
            "type octile\n"
            "height 5\n"
            "width 5\n"
            "map\n"
            ".....\n"
            ".....\n"
            "FFFFF\n"
            ".....\n"
            ".....\n"
        )
        grid = load_mapfb_grid(str(path))
        # Grid is (5+2) x (5+2) = 7x7.
        assert len(grid) == 7
        assert len(grid[0]) == 7
        # Top region (rows 1, 2 in grid coords) is kept.
        assert grid[1][1] is True
        assert grid[1][5] is True
        assert grid[2][1] is True
        assert grid[2][5] is True
        # Wall (row 3) is an obstacle in the kept grid.
        assert grid[3][1] is False
        assert grid[3][5] is False
        # Bottom region (rows 4, 5 in grid coords) is dropped.
        assert grid[4][1] is False
        assert grid[4][5] is False
        assert grid[5][1] is False
        assert grid[5][5] is False

    def test_grid_finds_kept_component_via_scan_order(self, tmp_path):
        """The first free cell in scan order is the seed, even when a
        later free region is larger.

        Layout (5x5 interior):
          row 0: F...F   <- small free strip at columns 2,3,4
          row 1: FFFFF   <- wall
          row 2: FFFFF   <- wall
          row 3: .....   <- larger region, will be dropped
          row 4: .....
        The first free cell in scan order is (1, 2). BFS keeps the
        small strip; the larger bottom region is dropped.
        """
        path = tmp_path / "scan_order.map"
        path.write_text(
            "type octile\n"
            "height 5\n"
            "width 5\n"
            "map\n"
            "F...F\n"
            "FFFFF\n"
            "FFFFF\n"
            ".....\n"
            ".....\n"
        )
        grid = load_mapfb_grid(str(path))
        # Top strip: cells (1, 2), (1, 3), (1, 4) are kept. (1, 5) is
        # an obstacle in the input map (F at the rightmost column of
        # the strip), so it stays an obstacle in the kept grid.
        assert grid[1][2] is True
        assert grid[1][3] is True
        assert grid[1][4] is True
        assert grid[1][5] is False  # obstacle, never free
        # The wall rows (2, 3 in grid coords) are obstacles.
        assert grid[2][2] is False
        assert grid[3][2] is False
        # The bottom region is dropped.
        assert grid[4][1] is False
        assert grid[4][5] is False
        assert grid[5][1] is False
        assert grid[5][5] is False

    def test_invalid_header_raises(self, tmp_path):
        """A malformed header raises ValueError, not silent garbage."""
        bad = tmp_path / "bad.map"
        bad.write_text("not a map file\nfoo bar\nbaz\n....\n")
        with pytest.raises(ValueError):
            load_mapfb_grid(str(bad))


# ---------------------------------------------------------------------------
# load_grip_map -- .map -> SpatialProblemInstance via right-hand tracing
# ---------------------------------------------------------------------------


class TestLoadGripMap:
    """Trace the boundary of every free region as polygon rings.

    The first ring traced (in scan order) is the outer boundary; every
    later ring is an inner (hole). Each ring is closed by repeating
    the first point. Every coordinate has a -1 shift applied to undo
    the grid's one-cell padding.
    """

    def test_arena_first_ring_is_outer(self):
        """For arena.map, the first ring is the outer boundary."""
        p = load_grip_map(str(ARENA))
        assert len(p.outer) > 0
        # Closed ring: first point == last point.
        assert p.outer[-1] == p.outer[0]

    def test_arena_shift_removes_padding(self):
        """After the -1 shift, all coordinates are in [0, 48] (arena is 49x49)."""
        p = load_grip_map(str(ARENA))
        for pt in p.outer:
            assert 0 <= pt.x <= 48
            assert 0 <= pt.y <= 48
        for inner in p.inners:
            for pt in inner:
                assert 0 <= pt.x <= 48
                assert 0 <= pt.y <= 48

    def test_arena_has_internal_holes(self):
        """arena.map has T columns inside the free space -- they become
        inner rings (holes). Without internal obstacles, the inners
        list would be empty."""
        p = load_grip_map(str(ARENA))
        # We don't pin a specific count, but arena.map has visible
        # internal T columns in the first few rows, so there must be
        # at least one inner ring.
        assert len(p.inners) >= 1, (
            "expected at least one inner ring from arena's T columns; "
            f"got outer={len(p.outer)}, inners={len(p.inners)}"
        )

    def test_arena_all_rings_are_closed(self):
        """Every ring (outer + inners) is closed."""
        p = load_grip_map(str(ARENA))
        for ring in [p.outer] + p.inners:
            assert ring[0] == ring[-1]

    def test_arena_agent_geometry_and_tasks_unset(self):
        """load_grip_map only sets the rings. agent_geometry and tasks
        come from a .scen file in a separate load."""
        p = load_grip_map(str(ARENA))
        assert p.agent_geometry == 0.0
        assert p.tasks == []

    def test_outer_comes_first_in_trace_order(self, tmp_path):
        """The first ring traced is the outer; later rings are inners.

        We can't easily craft a map with an isolated pocket AND a
        separate outer to test the order on the pocket (the pocket
        would be dropped by the flood fill). But we can test that
        a map with internal T columns puts the outer ring first.
        """
        # 7x7 with a T border and a T column in the middle.
        path = tmp_path / "border_and_column.map"
        rows = [
            "TTTTTTT",
            "T.....T",
            "T..T..T",
            "T..T..T",
            "T..T..T",
            "T.....T",
            "TTTTTTT",
        ]
        path.write_text(
            "type octile\nheight 7\nwidth 7\nmap\n" + "\n".join(rows) + "\n"
        )
        p = load_grip_map(str(path))
        # The outer is the perimeter of the free space; the inner is
        # the rectangle of the T column in the middle.
        assert len(p.outer) > 0
        assert len(p.inners) >= 1
        # The outer ring encloses a larger area than the inner.
        outer_bbox = _bbox(p.outer)
        if p.inners:
            inner_bbox = _bbox(p.inners[0])
            assert outer_bbox[2] >= inner_bbox[2]
            assert outer_bbox[3] >= inner_bbox[3]


def _bbox(ring):
    """Return (min_x, min_y, max_x, max_y) for a list of Point."""
    xs = [p.x for p in ring]
    ys = [p.y for p in ring]
    return min(xs), min(ys), max(xs), max(ys)


# ---------------------------------------------------------------------------
# load_mapfb_scenario -- .scen task rows
# ---------------------------------------------------------------------------


class TestLoadMAPFBScenario:
    """``.scen`` task rows. Tab-separated, ``version`` line skipped.

    Each task row has 9 fields; only ``(start_x, start_y, goal_x,
    goal_y)`` are returned.
    """

    def test_synthetic_scenario_row_count(self, tmp_path):
        """A 5-row .scen file returns 5 tasks."""
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0\tmap.map\t8\t8\t0\t0\t1\t1\t4.0\n"
            "0\tmap.map\t8\t8\t1\t0\t0\t1\t4.0\n"
            "0\tmap.map\t8\t8\t2\t0\t3\t1\t4.0\n"
            "1\tmap.map\t8\t8\t3\t0\t2\t1\t4.0\n"
            "1\tmap.map\t8\t8\t4\t0\t5\t1\t4.0\n"
        )
        tasks = load_mapfb_scenario(str(path))
        assert len(tasks) == 5

    def test_synthetic_scenario_coordinates(self, tmp_path):
        """Coordinates are parsed as ints in the right order:
        (start_x, start_y, goal_x, goal_y)."""
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0\tmap.map\t8\t8\t0\t0\t1\t1\t4.0\n"
            "0\tmap.map\t8\t8\t2\t3\t4\t5\t5.0\n"
        )
        tasks = load_mapfb_scenario(str(path))
        assert tasks == [
            (0, 0, 1, 1),
            (2, 3, 4, 5),
        ]

    def test_scenario_skips_version_line(self, tmp_path):
        """The 'version N' header line is NOT counted as a task."""
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0\tm\t8\t8\t0\t0\t1\t1\t4.0\n"
        )
        assert len(load_mapfb_scenario(str(path))) == 1

    def test_scenario_accepts_space_separated_rows(self, tmp_path):
        """The C++ uses whitespace-skipping fscanf, so any whitespace
        works between fields. The Python port uses ``split()`` to match.
        """
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0 m 8 8 0 0 1 1 4.0\n"
        )
        tasks = load_mapfb_scenario(str(path))
        assert tasks == [(0, 0, 1, 1)]

    def test_scenario_rejects_malformed_row(self, tmp_path):
        """A row with the wrong number of fields raises ValueError."""
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0 m 8 8 0 0 1 1\n"  # only 8 fields, want 9
        )
        with pytest.raises(ValueError):
            load_mapfb_scenario(str(path))

    def test_scenario_returns_only_start_and_goal(self, tmp_path):
        """The 9th field (optimal_length) is discarded, not stored."""
        path = tmp_path / "test.scen"
        path.write_text(
            "version 1\n"
            "0\tm\t8\t8\t1\t2\t3\t4\t99.5\n"
        )
        tasks = load_mapfb_scenario(str(path))
        # Tuple is 4 elements; the length 99.5 is not part of it.
        assert tasks == [(1, 2, 3, 4)]
        assert len(tasks[0]) == 4


# ---------------------------------------------------------------------------
# Oracle parity -- byte-diff against the C++ reference prints
# ---------------------------------------------------------------------------
#
# The C++ reference prints the four spatial loaders via a small driver
# compiled inside the docker image. The output format used here is a
# Python-portable text form:
#
#   outer <N>           | grid <H> <W>
#   <x> <y>             | <row of H chars, 0=obstacle 1=free>
#   ...                 | ...
#   inners <M>          | (for text/grip)
#   ring <K>            |
#   <x> <y>             |
#   ...                 |
#   agent_geometry <r>  | (for text/grip)
#   tasks <T>           |
#   <ix> <iy>           |
#   <gx> <gy>           |
#
# The reference files in ``problem_instances/spatial/`` were
# generated with the C++'s print_spatial driver built inside the
# Docker image.
#
# Scen has no committed reference file because the reference ships no
# .scen fixtures; the byte-parity check for scen is run ad-hoc (see the
# stage 1b report) and asserted here by parsing parity on synthetic rows.


def _dump_spatial(p) -> str:
    """Print a SpatialProblemInstance in the C++ driver's format."""
    lines = [f"outer {len(p.outer)}"]
    for pt in p.outer:
        lines.append(f"{pt.x:.15f} {pt.y:.15f}")
    lines.append(f"inners {len(p.inners)}")
    for ring in p.inners:
        lines.append(f"ring {len(ring)}")
        for pt in ring:
            lines.append(f"{pt.x:.15f} {pt.y:.15f}")
    lines.append(f"agent_geometry {p.agent_geometry:.15f}")
    lines.append(f"tasks {len(p.tasks)}")
    for t in p.tasks:
        lines.append(f"{t.initial_point.x:.15f} {t.initial_point.y:.15f}")
        lines.append(f"{t.goal_point.x:.15f} {t.goal_point.y:.15f}")
    return "\n".join(lines) + "\n"


def _dump_grid(grid) -> str:
    """Print a free-cell grid in the C++ driver's format."""
    lines = [f"{len(grid)} {len(grid[0])}"]
    for row in grid:
        lines.append("".join("1" if c else "0" for c in row))
    return "\n".join(lines) + "\n"


class TestOracleParity:
    """Byte-diff against the C++ reference prints committed as fixtures.

    These are check #3 in PORTING.md §10 -- ``ref print vs py print of
    the same instance`` -- for the four spatial loaders. The grid and
    grip fixtures come from arena.map; the text fixtures come from
    sample.txt and empty256.txt.
    """

    def test_text_sample_matches_oracle(self):
        py = _dump_spatial(load_spatial_problem(str(SAMPLE)))
        ref = (SPATIAL_DIR / "sample_text_ref.txt").read_text()
        assert py == ref

    def test_text_empty256_matches_oracle(self):
        py = _dump_spatial(load_spatial_problem(str(EMPTY256)))
        ref = (SPATIAL_DIR / "empty256_text_ref.txt").read_text()
        assert py == ref

    def test_grid_arena_matches_oracle(self):
        py = _dump_grid(load_mapfb_grid(str(ARENA)))
        ref = (SPATIAL_DIR / "arena_grid_ref.txt").read_text()
        assert py == ref

    def test_grip_arena_matches_oracle(self):
        py = _dump_spatial(load_grip_map(str(ARENA)))
        ref = (SPATIAL_DIR / "arena_grip_ref.txt").read_text()
        assert py == ref
