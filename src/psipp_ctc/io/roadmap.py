"""Stage 1a -- the ROADMAP text format.

Port of
  PSIPP-CTC_src/src/base/input.hpp  (lines 12-45: constructor, lines 47-79: print)

This is the format that ``psipp-plan`` reads and that the planner harness
serialises. It is a token-based text format -- whitespace (space, tab,
newline) is one delimiter, newlines are NOT structural. The reference uses
``fscanf``; the port splits the file on whitespace and consumes tokens in
order, which is equivalent because ``fscanf`` skips whitespace between
conversions.

Format (whitespace-tokenised):

    n_vertices n_edges n_agents
    x_0 y_0
    x_1 y_1
    ...
    x_{V-1} y_{V-1}
    s_0 t_0
    s_1 t_1
    ...
    s_{E-1} t_{E-1}
    iv_0 gv_0
    iv_1 gv_1
    ...
    iv_{A-1} gv_{A-1}
    r

Edges are bucketed into the source vertex's out-list in declaration order
via ``ProblemInstance.add_edge``. An edge's identity is its ``(source,
out-list-index)`` pair; order of insertion IS the edge id (HANDOFF.md §5,
PORTING.md §3 trap 2).

Path sentinels
--------------
The literal strings ``"stdin"`` and ``"stdout"`` are special-cased to mean
the process standard streams, mirroring ``input.hpp:16`` and ``input.hpp:51``.
This is a C convention that survives the port because the planner CLI shells
the data through stdin/stdout, and external tools (Docker, pipes) need the
literal path "stdin" to be addressable as a filename.

Float formatting
----------------
Coordinates and the agent radius are written with ``%030.15lf`` (width 30,
zero-padded, 15 digits after the decimal point). In Python that is
``f"{x:030.15f}"``. Width 30 is the C ``printf`` width and must be preserved
exactly so a ``diff`` against the reference's output is empty for the same
input instance. (Sample width 30 chosen so the line is always a fixed length
-- useful for some downstream tools, not for us.)

Preserve exactly
----------------
* fscanf-style tokenisation: read with ``str.split()`` over the whole file,
  consume tokens one at a time. Do NOT read line-by-line.
* Field order: header (3 ints), coordinates (V pairs of double), edges (E
  pairs of int), agent tasks (A pairs of int), radius (1 double).
* Edge output order: outer loop is the source vertex in id order; inner loop
  is the out-list in insertion order. This produces bucket-sorted output even
  if the input file interleaved edges across sources.
* The literal ``"stdin"`` / ``"stdout"`` path sentinels. Equality on the raw
  string, not a normalised form.
* ``%030.15f`` for every coordinate and the agent radius -- the agent radius
  is the last token, NOT inside any section.
"""

from __future__ import annotations

import contextlib
import sys
from typing import IO, Iterator, Union

from psipp_ctc.problem import (
    AgentTask,
    Point,
    ProblemInstanceWithGeometry,
)

PathLike = Union[str, "object"]  # PathLike: str or os.PathLike


@contextlib.contextmanager
def _open_for_read(path: Union[str, PathLike]) -> Iterator[IO[str]]:
    """Resolve the path sentinel and yield a readable text stream.

    The literal ``"stdin"`` is the only sentinel for input; the reference's
    ``input.hpp:16`` checks ``file_path == "stdin"``. Anything else opens as
    a normal file.

    Sentinel streams are **not closed** on exit: the C++ reference explicitly
    skips ``fclose`` on the standard streams (``input.hpp:42-44``), and
    closing ``sys.stdin`` / ``sys.stdout`` is a process-wide side effect that
    would break the rest of the program -- including pytest's own capture
    machinery.
    """
    if str(path) == "stdin":
        yield sys.stdin
        return
    with open(path, "r", encoding="utf-8") as f:
        yield f


@contextlib.contextmanager
def _open_for_write(path: Union[str, PathLike]) -> Iterator[IO[str]]:
    """Resolve the path sentinel and yield a writable text stream.

    The literal ``"stdout"`` is the only sentinel for output; the reference's
    ``input.hpp:51`` checks ``file_path == "stdout"``. Anything else opens as
    a normal file in write mode.

    Same as ``_open_for_read``: sentinel streams are not closed on exit.
    """
    if str(path) == "stdout":
        yield sys.stdout
        return
    with open(path, "w", encoding="utf-8") as f:
        yield f


def load_roadmap_problem(path: Union[str, PathLike]) -> ProblemInstanceWithGeometry:
    """Read a roadmap problem file and return a populated geometry instance.

    The C++ side is the ``ProblemInstanceWithGeometry<Geometry2D>``
    constructor specialisation at ``input.hpp:12-45``. Returns ``None`` on
    open failure in C++; we raise :class:`OSError` / :class:`ValueError`
    instead -- the brief (PORTING.md §10) requires exceptions where the C++
    would silently produce a broken instance.

    Tokenisation: ``str.split()`` on the full file content. This matches
    ``fscanf`` whitespace-skipping semantics for ``%d`` and ``%lf`` conversions
    (PORTING.md §4 trap 4).
    """
    with _open_for_read(path) as f:
        # ``split()`` with no args splits on any run of whitespace, including
        # newlines, which is exactly what ``fscanf("%d%d%d", ...)`` consumes
        # through a mix of spaces, tabs and newlines.
        tokens = f.read().split()

    it = iter(tokens)

    def need_int() -> int:
        try:
            return int(next(it))
        except StopIteration as exc:
            raise ValueError(
                "roadmap file ended early while reading an integer field"
            ) from exc

    def need_float() -> float:
        try:
            return float(next(it))
        except StopIteration as exc:
            raise ValueError(
                "roadmap file ended early while reading a float field"
            ) from exc

    n_vertices = need_int()
    n_edges = need_int()
    n_agents = need_int()
    if n_vertices < 0 or n_edges < 0 or n_agents < 0:
        raise ValueError(
            f"roadmap header has negative count: "
            f"n_vertices={n_vertices} n_edges={n_edges} n_agents={n_agents}"
        )

    problem = ProblemInstanceWithGeometry()
    # ``set_number_of_vertex`` on the geometry subclass resizes BOTH
    # ``coordinates`` and ``edges`` together (problem_with_geometry.hpp:29-33).
    problem.set_number_of_vertex(n_vertices)

    # -- Coordinates -----------------------------------------------------
    # fscanf "%lf%lf" repeated ``n_vertices`` times; the reference constructs
    # ``Point(x, y)`` and assigns to ``coordinates[vertex_id]``.
    for vertex_id in range(n_vertices):
        x = need_float()
        y = need_float()
        problem.set_point(vertex_id, Point(x, y))

    # -- Edges -----------------------------------------------------------
    # fscanf "%d%d" repeated ``n_edges`` times; the reference appends to
    # ``edges[start]`` so the out-list order IS the input file's order. We
    # do the same with ``add_edge``.
    for _ in range(n_edges):
        source = need_int()
        target = need_int()
        if not (0 <= source < n_vertices):
            raise ValueError(
                f"edge source {source} out of range [0, {n_vertices})"
            )
        if not (0 <= target < n_vertices):
            raise ValueError(
                f"edge target {target} out of range [0, {n_vertices})"
            )
        problem.add_edge(source, target)

    # -- Agent tasks -----------------------------------------------------
    # fscanf "%d%d" repeated ``n_agents`` times; stored as
    # ``AgentTask(initial_vertex, goal_vertex)``. We rebuild the list rather
    # than rely on whatever the constructor left behind, to match the C++
    # ``tasks.resize(number_of_agents)`` semantics.
    problem.tasks = [
        AgentTask(need_int(), need_int()) for _ in range(n_agents)
    ]

    # -- Agent geometry (radius) ----------------------------------------
    # A single trailing ``%lf``. The reference assigns to the geometry
    # member ``agent_geometry`` of the instance.
    problem.agent_geometry = need_float()

    # If the file has trailing tokens, the C++ silently ignores them
    # (fscanf returns). We do the same -- ``split()`` just leaves the
    # extras in the list, which we never look at.

    return problem


def print_roadmap_problem(
    problem: ProblemInstanceWithGeometry, path: Union[str, PathLike]
) -> None:
    """Serialise a roadmap problem to a file (or stdout).

    Mirrors the ``ProblemInstanceWithGeometry<Geometry2D>::print``
    specialisation at ``input.hpp:47-79``. Field order is identical to the
    reader so a round-trip is exact for the same instance.

    Float formatting is ``%030.15f``: width 30, zero-padded on the left,
    15 digits after the decimal point. For ``0.0`` the result is
    ``0000000000000.000000000000000`` (a 30-character run of zeros except
    the lone ``.``); for a real coordinate it preserves 15 fractional digits
    so the round-trip is bit-exact for any IEEE-754 double.
    """
    n_vertices = problem.get_number_of_vertices()
    # n_edges is the total edge count, summed across all out-lists. The C++
    # walks ``edges[i].size()`` in a separate pass before the format string.
    n_edges = sum(len(out) for out in problem.edges)
    n_agents = len(problem.tasks)

    with _open_for_write(path) as f:
        # Header: three ints on one line. The C++ uses "%lu %d %lu"; the
        # Python equivalents are just str(int), which prints decimal with
        # no padding. The two ``%lu`` slots take ``edges.size()`` and
        # ``tasks.size()`` (both ``size_t``); the middle ``%d`` takes the
        # signed ``number_of_edges``. We pass plain ints, which is fine.
        f.write(f"{n_vertices} {n_edges} {n_agents}\n")

        # Coordinates. The reference uses ``%030.15lf %030.15lf\n``; in
        # Python the format spec is ``030.15f`` (same width, same precision,
        # zero-padded). f-mode prints the decimal point explicitly and
        # truncates the exponent for values representable as fixed point.
        for vertex_id in range(n_vertices):
            p = problem.get_point(vertex_id)
            f.write(f"{p.x:030.15f} {p.y:030.15f}\n")

        # Edges. The reference iterates by source vertex in id order, then
        # by out-list position. The printed line is ``f"{source} {target}"``;
        # since ``source`` is just the outer loop index, an output is a
        # bucket-sorted view of the adjacency list. Round-tripping the same
        # data produces the same adjacency list, but the output line order
        # may differ from the input if the input was interleaved.
        for source in range(n_vertices):
            for target in problem.edges[source]:
                f.write(f"{source} {target}\n")

        # Agent tasks. ``%d %d\n`` for each task.
        for task in problem.tasks:
            f.write(f"{task.initial_vertex} {task.goal_vertex}\n")

        # Agent geometry (radius). Same format as coordinates.
        f.write(f"{problem.agent_geometry:030.15f}\n")
