"""Stage 1b -- the SPATIAL problem formats.

Port of
  PSIPP-CTC_src/src/base/input_spatial.hpp  (lines 14-50, 52-104, 106-161, 163-182)
  PSIPP-CTC_src/src/base/spatial.hpp       (the SpatialProblemInstance data
                                            members, lines 11-26)

Four functions, ported in the order they appear in ``input_spatial.hpp``:

1. ``load_spatial_problem(path)``  -- the hand-written continuous-space
   polygon format. ``input_spatial.hpp:15-50`` (a template specialisation
   of ``SpatialProblemInstance<Space2D>::loadTextFile``).
2. ``load_mapfb_grid(path)``       -- octile ``.map`` grid -> a padded
   free-cell grid with a single connected component.
   ``input_spatial.hpp:52-104``.
3. ``load_grip_map(path)``         -- ``.map`` -> ``SpatialProblemInstance``
   with outer + inner polygon rings via right-hand boundary tracing.
   ``input_spatial.hpp:106-161`` (a template specialisation of
   ``SpatialProblemInstance<Space2D>::loadGripMap``).
4. ``load_mapfb_scenario(path)``   -- ``.scen`` task rows.
   ``input_spatial.hpp:163-182`` (a free function).

``loadScenario`` (``input_spatial.hpp:184-197``) is **deferred to Stage 3**
because it calls ``isInside()``, which needs the Space2D backend. This
session deliberately does not invent a backend.

Data model
----------
The space is represented as plain Python lists, exactly as the brief
specifies:

* ``outer`` -- ``list[Point]``, a closed ring (first point repeated at the
  end). Built by ``load_spatial_problem`` from the file's outer vertices,
  or by ``load_grip_map`` from the right-hand trace.
* ``inners`` -- ``list[list[Point]]``, one closed ring per hole. Order is
  the trace order, NOT geometric containment -- the first ring traced
  becomes the outer boundary, every later ring becomes a hole
  (``input_spatial.hpp:143-158``).

Containment, proximity and bounds queries are NOT implemented here; they
live in Stage 3 where ``Space2D`` is ported.

Preserve exactly
----------------
* ``"stdin"`` sentinel and ``fscanf`` tokenisation, same as Stage 1a
  (``input_spatial.hpp:18``, ``spatial.hpp:18``). Whitespace is a single
  delimiter, newlines are not structural.
* Rings are closed by repeating the first point (``input_spatial.hpp:31,
  39, 149, 156``).
* ``load_mapfb_grid`` keeps only ONE connected component. The first free
  cell in scan order (top-to-bottom, left-to-right) is the seed of a
  BFS flood fill; everything else becomes impassable
  (``input_spatial.hpp:75-102``). This is intentional -- the planner
  cannot serve tasks in an unreachable region -- and is not a bug to
  "fix".
* The grid is padded with one impassable border cell on every side, so a
  ``H x W`` file becomes a ``(H+2) x (W+2)`` array
  (``input_spatial.hpp:59-62``).
* Free cell chars are ``'.'``, ``'G'``, ``'S'``; everything else is an
  obstacle (``input_spatial.hpp:67``).
* ``load_grip_map`` traces boundaries with the right-hand rule and
  applies a -1 shift to undo the padding (``input_spatial.hpp:147, 154``)
  -- opposite sign from ``loadScenario``'s +0.5 cell-centre offset
  (``input_spatial.hpp:190, 192``). The two different offsets are not a
  bug; they are two different coordinate frames.
* The first ring traced becomes the outer boundary, every later ring
  becomes a hole (``input_spatial.hpp:143-158``). This is trace order,
  not geometric containment. The outer comes first only because the
  outer is found first in scan order; do not replace this with a
  containment test.

Known asymmetry
---------------
``printTextFile`` (in ``experiments/simplify_grid.cpp:9-33``) is NOT the
inverse of ``loadTextFile`` -- the agent count sits in a different
position in the stream (``input_spatial.hpp:24`` vs
``simplify_grid.cpp:11-16``). This is a reference quirk. We do NOT port
``printTextFile`` because it is driver-local to ``simplify_grid`` and no
other code path uses its output format. If you ever need to round-trip a
spatial problem, you must write the file by hand to match the
``loadTextFile`` format.
"""

from __future__ import annotations

import contextlib
import sys
from collections import deque
from typing import IO, Iterator, List, Tuple, Union

from psipp_ctc.problem import Point, SpatialProblemInstance, SpatialTask

PathLike = Union[str, "object"]


# Tokens that close a coordinate. The C++ uses ``Geometry2D::scanPoint``,
# which is a thin wrapper around ``fscanf("%lf %lf", &x, &y)`` -- two
# whitespace-separated doubles. We do the same.
def _read_point(tokens: Iterator[str]) -> Point:
    return Point(float(next(tokens)), float(next(tokens)))


@contextlib.contextmanager
def _open_for_read(path: Union[str, PathLike]) -> Iterator[IO[str]]:
    """Resolve the path sentinel and yield a readable text stream.

    Only ``load_spatial_problem`` accepts the ``"stdin"`` sentinel; the
    other loaders use ``open()`` because the C++ originals use ``fopen``
    unconditionally. Sentinel streams are NOT closed on exit.
    """
    if str(path) == "stdin":
        yield sys.stdin
        return
    with open(path, "r", encoding="utf-8") as f:
        yield f


# ---------------------------------------------------------------------------
# 1. load_spatial_problem -- the continuous-space polygon format
# ---------------------------------------------------------------------------


def load_spatial_problem(path: Union[str, PathLike]) -> SpatialProblemInstance:
    """Read a continuous-space polygon problem.

    Port of ``SpatialProblemInstance<Space2D>::loadTextFile``
    (``input_spatial.hpp:14-50``).

    Format (whitespace-tokenised, ``fscanf``-style):

        number_of_holes number_of_agents
        number_of_outer_vertices
        x_0 y_0
        x_1 y_1
        ...
        x_{V-1} y_{V-1}
        [per hole:]
          number_of_inner_vertices
          x_0 y_0
          ...
        agent_geometry  (one double)
        [per agent:]
          ix iy gx gy  (four doubles)

    Rings are closed by appending the first point to the end
    (``input_spatial.hpp:31, 39``). The outer ring's first point is taken
    BEFORE the holes are read; the closing duplicate of each inner ring is
    taken at the end of that ring's section, matching the C++ exactly.

    Returns a populated :class:`SpatialProblemInstance`. The four geometry
    methods on it are ``NotImplementedError`` until Stage 3.
    """
    with _open_for_read(path) as f:
        tokens = iter(f.read().split())

    def need_int() -> int:
        try:
            return int(next(tokens))
        except StopIteration as exc:
            raise ValueError(
                "spatial file ended early while reading an integer field"
            ) from exc

    def need_float() -> float:
        try:
            return float(next(tokens))
        except StopIteration as exc:
            raise ValueError(
                "spatial file ended early while reading a float field"
            ) from exc

    number_of_holes = need_int()
    number_of_agents = need_int()
    if number_of_holes < 0 or number_of_agents < 0:
        raise ValueError(
            f"spatial header has negative count: "
            f"holes={number_of_holes} agents={number_of_agents}"
        )

    # -- Outer ring -------------------------------------------------------
    number_of_outer_vertices = need_int()
    if number_of_outer_vertices < 0:
        raise ValueError(
            f"outer vertex count is negative: {number_of_outer_vertices}"
        )
    outer: List[Point] = [
        _read_point(tokens) for _ in range(number_of_outer_vertices)
    ]
    if number_of_outer_vertices > 0:
        # Close the ring by repeating the first point. Reference:
        # ``input_spatial.hpp:31``. If the ring is empty (a degenerate
        # case), we leave it empty -- the brief does not require
        # duplication of an empty ring.
        outer.append(outer[0])

    # -- Holes ------------------------------------------------------------
    inners: List[List[Point]] = []
    for _ in range(number_of_holes):
        number_of_inner_vertices = need_int()
        if number_of_inner_vertices < 0:
            raise ValueError(
                f"inner vertex count is negative: "
                f"{number_of_inner_vertices}"
            )
        ring: List[Point] = [
            _read_point(tokens) for _ in range(number_of_inner_vertices)
        ]
        if number_of_inner_vertices > 0:
            ring.append(ring[0])
        inners.append(ring)

    # -- Agent geometry + tasks -------------------------------------------
    agent_geometry = need_float()
    tasks: List[SpatialTask] = []
    for _ in range(number_of_agents):
        initial = _read_point(tokens)
        goal = _read_point(tokens)
        tasks.append(SpatialTask(initial, goal))

    return SpatialProblemInstance(
        outer=outer,
        inners=inners,
        agent_geometry=agent_geometry,
        tasks=tasks,
    )


# ---------------------------------------------------------------------------
# 2. load_mapfb_grid -- octile .map -> padded free-cell grid
# ---------------------------------------------------------------------------


def load_mapfb_grid(path: Union[str, PathLike]) -> List[List[bool]]:
    """Read an octile ``.map`` file and return a padded free-cell grid.

    Port of the free function ``loadMAPFBGrid``
    (``input_spatial.hpp:52-104``).

    The result is a ``(H+2) x (W+2)`` boolean grid: ``True`` for free
    cells (the char was ``'.'``, ``'G'``, or ``'S'``), ``False`` for
    obstacles and the impassable border. The border pad is one cell wide
    on every side (``input_spatial.hpp:59-62``); the original map cells
    sit at index ``[1..H] x [1..W]``.

    Single connected component
    -------------------------
    Only the free region containing the first free cell in scan order
    (top-to-bottom, left-to-right) is kept. Every other free region is
    turned into obstacles. This is intentional and matches the C++
    ``assert(start_y <= height)`` (``input_spatial.hpp:86``) -- the
    reference raises an assertion failure on a fully-obstructed map.
    The Python port raises :class:`ValueError` instead.

    Returns
    -------
    list of list of bool
        ``grid[y][x]`` is True iff cell (x, y) is free in the kept
        connected component. Indexing is row-major: ``grid[0]`` is the
        top row (the padding row of obstacles), and ``grid[1][1]`` is
        the top-left cell of the original map.
    """
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    # The C++ uses fscanf with literal-match format strings
    # ("type octile\\nheight %d\\nwidth %d\\nmap\\n"). Python doesn't have
    # a literal-match scanner that ignores token boundaries, so we read
    # the header line-by-line and switch to per-row char reads after the
    # "map" marker. This is equivalent because the literal tokens ("type",
    # "octile", "height", "width", "map") are fixed strings that appear
    # on their own lines in every reference .map file; the C++ is also
    # matching them literally, just with embedded whitespace skipped.

    lines = text.splitlines()

    # Find the header. The convention is: line 0 is "type octile", line 1
    # is "height H", line 2 is "width W", line 3 is "map", line 4+ is the
    # data. Some .map files have a trailing "map" with no newline; we
    # accept the last few lines being shorter as long as the keywords
    # are present in order.
    if len(lines) < 4:
        raise ValueError(f"map file too short: {len(lines)} lines")

    # type octile
    parts = lines[0].split()
    if len(parts) != 2 or parts[0] != "type" or parts[1] != "octile":
        raise ValueError(
            f"unexpected map header line 0: {lines[0]!r} (want 'type octile')"
        )

    # height H
    parts = lines[1].split()
    if len(parts) != 2 or parts[0] != "height":
        raise ValueError(
            f"unexpected map header line 1: {lines[1]!r} (want 'height H')"
        )
    height = int(parts[1])

    # width W
    parts = lines[2].split()
    if len(parts) != 2 or parts[0] != "width":
        raise ValueError(
            f"unexpected map header line 2: {lines[2]!r} (want 'width W')"
        )
    width = int(parts[1])

    # map
    if lines[3].strip() != "map":
        raise ValueError(
            f"unexpected map header line 3: {lines[3]!r} (want 'map')"
        )

    # The data rows. The C++ reads ``width`` chars per row, then skips
    # the row's trailing newlines via ``getc`` until the next non-\n
    # char. Python's ``splitlines()`` already removed the newlines, so
    # each subsequent line is a row of the map (possibly with trailing
    # carriage returns stripped, since ``splitlines()`` handles \\r\\n
    # and \\n uniformly).
    data_lines = lines[4:]

    if len(data_lines) < height:
        raise ValueError(
            f"map data has {len(data_lines)} rows, header says {height}"
        )

    # Build the padded row_grid. False = obstacle (the C++ default
    # initialiser). Indices [1..H] x [1..W] hold the actual map cells.
    row_grid: List[List[bool]] = [
        [False] * (width + 2) for _ in range(height + 2)
    ]
    for i in range(height):
        row_text = data_lines[i]
        # Some reference .map files have a trailing carriage return that
        # ``splitlines`` strips; we trust the row's length modulo
        # whitespace. The C++ reads exactly ``width`` chars and then
        # walks to the next newline; we mimic by slicing to ``width``.
        if len(row_text) < width:
            raise ValueError(
                f"map row {i} has {len(row_text)} chars, header says {width}"
            )
        for j in range(width):
            c = row_text[j]
            row_grid[i + 1][j + 1] = c == "." or c == "G" or c == "S"

    # -- Single-component flood fill ------------------------------------
    # The C++ searches scan order (y from 1..H, x from 1..W) for the
    # first free cell, then BFS-flood-fills. We do the same.
    grid: List[List[bool]] = [
        [False] * (width + 2) for _ in range(height + 2)
    ]
    start_y = start_x = -1
    for sy in range(1, height + 1):
        for sx in range(1, width + 1):
            if row_grid[sy][sx]:
                start_y, start_x = sy, sx
                break
        if start_y != -1:
            break
    if start_y == -1:
        # No free cell at all -- the C++ asserts; we raise.
        raise ValueError("map has no free cell to start the flood fill")

    # 4-neighbour BFS, identical to the C++ ``std::queue`` loop.
    dx = (1, 0, -1, 0)
    dy = (0, 1, 0, -1)
    queue: deque[Tuple[int, int]] = deque()
    queue.append((start_y, start_x))
    grid[start_y][start_x] = True
    while queue:
        y, x = queue.popleft()
        for v in range(4):
            new_y = y + dy[v]
            new_x = x + dx[v]
            if row_grid[new_y][new_x] and not grid[new_y][new_x]:
                grid[new_y][new_x] = True
                queue.append((new_y, new_x))

    return grid


# ---------------------------------------------------------------------------
# 3. load_grip_map -- .map -> SpatialProblemInstance via right-hand tracing
# ---------------------------------------------------------------------------


def load_grip_map(path: Union[str, PathLike]) -> SpatialProblemInstance:
    """Read a ``.map`` and trace its boundaries as polygon rings.

    Port of ``SpatialProblemInstance<Space2D>::loadGripMap``
    (``input_spatial.hpp:106-161``).

    The function calls :func:`load_mapfb_grid` first to get the padded
    free-cell grid, then runs a right-hand wall-following tracer on every
    "left edge" of a free region (a free cell whose west neighbour is
    blocked) in scan order. The first ring traced becomes ``outer``;
    every later ring becomes an ``inner`` (hole). This is **trace order,
    not geometric containment** -- see PORTING.md §4 trap 17.

    Each ring is closed by appending the first point to the end
    (``input_spatial.hpp:149, 156``). The ``-1`` shift applied to every
    coordinate (``input_spatial.hpp:147, 154``) undoes the grid's
    one-cell padding, so the result lives in the original ``.map``
    coordinate frame. The sign is opposite to ``loadScenario``'s
    ``+0.5`` cell-centre offset (``input_spatial.hpp:190, 192``) -- two
    different coordinate frames, not a bug.

    Returns a :class:`SpatialProblemInstance` with ``outer`` and
    ``inners`` populated. ``agent_geometry`` is ``0.0`` (unset) and
    ``tasks`` is empty -- those come from a separate ``.scen`` file in
    Stage 3, via :func:`load_spatial_problem` later.
    """
    grid = load_mapfb_grid(path)
    height = len(grid) - 2
    width = len(grid[0]) - 2
    if height <= 0 or width <= 0:
        raise ValueError(
            f"map is empty after the single-component flood fill: "
            f"{height}x{width}"
        )

    visited: List[List[bool]] = [
        [False] * (width + 2) for _ in range(height + 2)
    ]
    found_outer = False

    # Direction encoding: 0=right, 1=bottom, 2=left, 3=top. Same as
    # ``input_spatial.hpp:115-117``.
    dx = (1, 0, -1, 0)
    dy = (0, 1, 0, -1)
    # ``sx, sy`` are the cell offsets for "diagonally front-left" and
    # "front" of the current position in a given direction; they are
    # NOT the offsets to the left cell. The exact geometric meaning is
    # in the C++; we copy the arrays verbatim.
    sx = (0, -1, -1, 0)
    sy = (0, 0, -1, -1)

    rings: List[List[Point]] = []

    for start_y in range(1, height + 1):
        for start_x in range(1, width + 1):
            # Start-of-trace condition: cell to the west is blocked, this
            # cell is free, and this cell is not yet visited.
            # ``input_spatial.hpp:120-122``.
            west_blocked = not grid[start_y][start_x - 1]
            here_free = grid[start_y][start_x]
            if not (west_blocked and here_free) or visited[start_y][start_x]:
                continue

            # Right-hand wall-following. The starting direction is
            # always 1 (bottom), matching ``input_spatial.hpp:127``.
            xs: List[int] = []
            ys: List[int] = []
            x, y = start_x, start_y
            direction = 1
            # The C++ uses a do-while so the body runs at least once.
            # We translate to a guarded loop with a one-shot entry.
            first_step = True
            while first_step or not (x == start_x and y == start_y):
                first_step = False
                if direction == 1:
                    # Only mark the cell visited while we're moving
                    # bottom. The mark prevents re-tracing the same ring
                    # from a different starting cell on the same
                    # boundary. ``input_spatial.hpp:129``.
                    visited[y][x] = True
                # Step in the current direction.
                x += dx[direction]
                y += dy[direction]
                ldir = (direction + 3) % 4
                rdir = (direction + 1) % 4
                # Wall test: "front-left cell blocked" or "front cell
                # blocked" -- the C++ uses these two to decide whether
                # to turn left, turn right, or continue. The negation
                # matters: ``!grid[front-left]`` means blocked, so we
                # turn LEFT when front-left is blocked (counterintuitive
                # but it is what the C++ does and what
                # ``checkRoadspaceValidity`` will verify).
                if not grid[y + sy[ldir]][x + sx[ldir]]:
                    xs.append(x)
                    ys.append(y)
                    direction = ldir
                elif grid[y + sy[direction]][x + sx[direction]]:
                    xs.append(x)
                    ys.append(y)
                    direction = rdir
                # Else: continue forward (no record, no turn).

            # Apply the -1 shift (``input_spatial.hpp:147, 154``) to
            # undo the grid padding. The point type is the same 2D
            # ``Point`` used everywhere else.
            ring = [Point(x - 1, y - 1) for x, y in zip(xs, ys)]
            if ring:
                ring.append(ring[0])
            rings.append(ring)
            found_outer = True  # only used to silence an unused warning

    # The C++ assigns the first traced ring to ``space.outer()`` and the
    # rest to ``space.inners()``. We do the same on a plain list.
    outer = rings[0] if rings else []
    inners = rings[1:] if len(rings) > 1 else []

    return SpatialProblemInstance(
        outer=outer,
        inners=inners,
        agent_geometry=0.0,
        tasks=[],
    )


# ---------------------------------------------------------------------------
# 4. load_mapfb_scenario -- .scen task rows
# ---------------------------------------------------------------------------


def load_mapfb_scenario(
    path: Union[str, PathLike],
) -> List[Tuple[int, int, int, int]]:
    """Read a MAPF-Benchmark ``.scen`` file and return task rows.

    Port of the free function ``loadMAPFBScenario``
    (``input_spatial.hpp:163-182``).

    The C++ scans with ``fscanf("%d%999s%d%d%d%d%d%d%lf", ...)`` until
    EOF, which silently consumes the leading ``version 1`` line as well
    as every task row. The 999-char map-name field is discarded; the
    9th field (optimal_length) is also discarded. We read the file line
    by line, skip the version line, and parse each task row as
    tab-separated.

    Returns
    -------
    list of tuple of (int, int, int, int)
        Each tuple is ``(start_x, start_y, goal_x, goal_y)`` in the
        original ``.map`` grid coordinates. The cell-centre offset
        ``+0.5`` (input_spatial.hpp:190, 192) is NOT applied here -- it
        is part of ``loadScenario``, which is deferred to Stage 3.
    """
    tasks: List[Tuple[int, int, int, int]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("version"):
                continue
            # The C++ skips whitespace before the first int; here we
            # split on any whitespace, which is the same effect.
            parts = line.split()
            # The reference format has 9 fields: bucket, map_name, width,
            # height, start_x, start_y, goal_x, goal_y, length. We
            # require the 9 to keep the loader strict; a malformed row
            # raises.
            if len(parts) != 9:
                raise ValueError(
                    f"scenario row has {len(parts)} fields, want 9: "
                    f"{line!r}"
                )
            # Indices: 0=bucket, 1=map_name, 2=width, 3=height,
            # 4=start_x, 5=start_y, 6=goal_x, 7=goal_y, 8=length.
            try:
                start_x = int(parts[4])
                start_y = int(parts[5])
                goal_x = int(parts[6])
                goal_y = int(parts[7])
            except ValueError as exc:
                raise ValueError(
                    f"scenario row has non-integer coordinate: {line!r}"
                ) from exc
            tasks.append((start_x, start_y, goal_x, goal_y))
    return tasks


def load_scenario(
    path: Union[str, PathLike],
    spatial: SpatialProblemInstance,
) -> None:
    """Append tasks from a ``.scen`` file to ``spatial`` (Stage 3).

    Port of ``SpatialProblemInstance<Space2D>::loadScenario``
    (``input_spatial.hpp:184-197``). For each task row, applies the
    ``+0.5`` cell-centre offset, checks both endpoints via
    ``spatial.is_inside``, and appends a ``SpatialTask`` iff both
    pass. Tasks whose endpoints fall outside the (cleared) free
    space are dropped (matching the C++ silently).

    The ``+0.5`` offset here and the ``-1`` offset in
    :func:`load_grip_map` are the two opposite signs called out in
    PORTING.md §4 trap 18.
    """
    from psipp_ctc.problem import SpatialTask
    for start_x, start_y, goal_x, goal_y in load_mapfb_scenario(path):
        initial = Point(start_x + 0.5, start_y + 0.5)
        goal = Point(goal_x + 0.5, goal_y + 0.5)
        if spatial.is_inside(initial) and spatial.is_inside(goal):
            spatial.tasks.append(SpatialTask(initial, goal))
