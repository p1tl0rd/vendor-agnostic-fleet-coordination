"""The Grid roadmap generator.

A uniform 2D grid is laid across the BBox of the outer ring; the cell
centres inside the free polygon become vertices, and a symmetric edge
is added between two cell centres iff the connecting segment stays
inside the free space. Each task's start and goal are appended as their
own vertex wired to the four cells whose centre is closest to it.

Adjacency-list contract
-----------------------
The output is a ``ProblemInstanceWithGeometry`` whose ``edges[v]`` is a
list of target vertex ids in insertion order. Each grid-cell vertex has
its neighbours added in the order defined by the C++ ``neighbors`` list
(``[(1,0), (0,1), (-1,0), (0,-1)]`` then diagonal extensions for
``K >= 3`` -- K is 4 here, so only the four cardinal offsets are
iterated, and the loop uses ``neighbors.size() / 2`` so the last pair is
visited once). The C++ adds each edge twice (one direction per cell
sweep), so the adjacency list has each undirected edge present in
both directions in the out-lists of the two endpoints. The Python port
mirrors this.

K is the connectivity parameter. ``K=4`` gives the 4-connected grid
(cardinal neighbours). ``K=8`` is supported (the neighbours list is
precomputed for any K) but the default is K=4.
"""

from __future__ import annotations

import math
from typing import List, Tuple

from psipp_ctc.problem import AgentTask, Point, ProblemInstanceWithGeometry
from psipp_ctc.roadmap.space_2d import (
    _make_polygon,
    check_edge_inside,
    check_vertex_inside,
    get_bound,
)

# Default connectivity for the Grid generator. K=4 gives the 4-connected
# lattice (cardinal neighbours). The C++ constructor allows K=8 too;
# Default to 4-connectivity (cardinal neighbours only).
DEFAULT_K: int = 4


def _build_neighbour_offsets(K: int) -> List[Tuple[int, int]]:
    """Precompute the K-neighbour lattice offsets.

    Start with the four cardinal offsets, then for each step from 3 to
    K, interleave every adjacent pair and also add the diagonal between
    them. For K=4 this is just the four cardinal offsets. For K=8 it
    adds the four diagonals at step K=3 (which become a single new pair
    interleaved between the cardinals).
    """
    neighbours: List[Tuple[int, int]] = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    for k in range(3, K + 1):
        new: List[Tuple[int, int]] = []
        for i in range(len(neighbours)):
            new.append(neighbours[i])
            j = (i + 1) % len(neighbours)
            new.append(
                (neighbours[i][0] + neighbours[j][0],
                 neighbours[i][1] + neighbours[j][1])
            )
        neighbours = new
    return neighbours


def generate_grid(
    spatial,
    grid_size: float,
    K: int = DEFAULT_K,
) -> ProblemInstanceWithGeometry:
    """Generate a Grid roadmap for ``spatial``.

    Returns a fresh ``ProblemInstanceWithGeometry`` whose vertices are:

    * one vertex per grid cell whose centre is inside the free polygon
      (with ``agent_geometry - EPS`` clearance);
    * two vertices per task (the task's start and goal coordinates),
      appended after the grid vertices, with their id assigned
      sequentially.

    Edges are added between neighbouring cell centres (K-connectivity)
    and between each task vertex and the four cells whose centre is
    closest to the task's point.
    """
    if grid_size <= 0:
        raise ValueError(f"grid_size must be > 0; got {grid_size}")

    neighbours = _build_neighbour_offsets(K)
    # The C++ uses ``neighbors.size() / 2`` pairs (each pair is visited
    # once per cell so the edge is added in one direction from the
    # source cell's sweep; the reverse direction is added when the
    # other cell is swept). We do the same.
    pair_count = len(neighbours) // 2

    x_bound = spatial.get_bound(0)
    y_bound = spatial.get_bound(1)
    x_width = int(math.floor((x_bound[1] - x_bound[0]) / grid_size + 0.5))
    y_width = int(math.floor((y_bound[1] - y_bound[0]) / grid_size + 0.5))

    problem = ProblemInstanceWithGeometry()
    problem.agent_geometry = spatial.agent_geometry

    # Build a (i, j) -> vertex_id map. The C++ uses
    # std::map<std::pair<int, int>, int>; we use a plain dict because
    # the keys are well-defined integers and we only need
    # find/insert/iterate -- no ordering observable from outside.
    vertex_ids: dict = {}

    for i in range(x_width):
        x = x_bound[0] + (i + 0.5) * grid_size
        for j in range(y_width):
            y = y_bound[0] + (j + 0.5) * grid_size
            p = Point(x, y)
            if spatial.is_inside(p):
                v_id = problem.get_number_of_vertices()
                vertex_ids[(i, j)] = v_id
                # Resize coordinates + edges together (the
                # geometry-subclass ``set_number_of_vertex`` trap).
                problem.coordinates.append(p)
                problem.edges.append([])

    # Pre-allocate the two-per-task slots in coordinates/edges so that
    # the task vertices are appended contiguously (the C++ does the
    # same with the size-then-fill pattern: each task contributes a
    # start and a goal entry).
    tasks = list(spatial.tasks)
    for _ in tasks:
        problem.coordinates.append(Point(0.0, 0.0))
        problem.coordinates.append(Point(0.0, 0.0))
        problem.edges.append([])
        problem.edges.append([])
    # Index of the first pre-allocated task slot -- after the grid
    # cells, before any task vertex has been assigned its real point.
    x_y_vertex_count = len(problem.coordinates) - 2 * len(tasks)

    # Add cell-to-cell edges (K-connectivity). The C++ iterates over
    # every cell, takes each neighbour offset's first half, and adds
    # the edge if the cell and its neighbour both exist and the
    # connecting segment passes is_edge_inside.
    for (i, j), v_id in vertex_ids.items():
        for v in range(pair_count):
            di, dj = neighbours[v]
            ni, nj = i + di, j + dj
            neighbour_id = vertex_ids.get((ni, nj))
            if neighbour_id is None:
                continue
            if spatial.is_edge_inside(
                problem.coordinates[v_id],
                problem.coordinates[neighbour_id],
            ):
                problem.add_edge(v_id, neighbour_id)
                problem.add_edge(neighbour_id, v_id)

    # Append task vertices, one per task. The C++ places each task
    # vertex at the end and wires it to the four grid cells whose
    # centres fall in the 2x2 cell square containing the task's
    # coordinates. The wiring is one-way: the task->cell edge is
    # added only in the start->cell direction for the initial, and
    # only the cell->task edge for the goal. The reverse direction
    # is NOT added in the C++ -- tasks are leaves of the agent
    # sub-tree. Verified by the test that checks the task-vertex
    # count and the start-cell-edge count.
    problem.tasks = []
    base_task = x_y_vertex_count
    for agent, task in enumerate(tasks):
        start_id = base_task + 2 * agent
        goal_id = start_id + 1
        problem.coordinates[start_id] = task.initial_point
        problem.coordinates[goal_id] = task.goal_point
        problem.tasks.append(AgentTask(start_id, goal_id))

        # The C++ uses dx = [0,0,1,1], dy = [0,1,0,1] to enumerate
        # the four cells in the 2x2 square that contains the task's
        # point. The square is found by floor((x - min) / size - 0.5).
        start_i = int(math.floor(
            (task.initial_point.x - x_bound[0]) / grid_size - 0.5
        ))
        start_j = int(math.floor(
            (task.initial_point.y - y_bound[0]) / grid_size - 0.5
        ))
        for di, dj in [(0, 0), (0, 1), (1, 0), (1, 1)]:
            cell_id = vertex_ids.get((start_i + di, start_j + dj))
            if cell_id is None:
                continue
            if spatial.is_edge_inside(
                problem.coordinates[start_id], problem.coordinates[cell_id]
            ):
                problem.add_edge(start_id, cell_id)

        goal_i = int(math.floor(
            (task.goal_point.x - x_bound[0]) / grid_size - 0.5
        ))
        goal_j = int(math.floor(
            (task.goal_point.y - y_bound[0]) / grid_size - 0.5
        ))
        for di, dj in [(0, 0), (0, 1), (1, 0), (1, 1)]:
            cell_id = vertex_ids.get((goal_i + di, goal_j + dj))
            if cell_id is None:
                continue
            if spatial.is_edge_inside(
                problem.coordinates[cell_id], problem.coordinates[goal_id]
            ):
                problem.add_edge(cell_id, goal_id)

    return problem
