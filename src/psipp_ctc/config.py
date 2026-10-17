"""Stage 5-follow-up -- YAML config loaders.

Ports the three ``make*`` factories that the C++ reference's
``planner_benchmark`` and ``roadmap_generation`` drivers call to turn a
parsed ``YAML::Node`` into runtime objects:

  PSIPP-CTC_src/src/base/load_problems.cpp    -- makeSpatialProblemInstance
                                                  + makeProblemInstance
  PSIPP-CTC_src/src/base/load_planners.cpp   -- makePlanner
  PSIPP-CTC_src/src/base/load_generators.cpp -- makeGenerator

The Python port uses ``pyyaml`` (already a project dependency) to parse
the config. ``yaml.safe_load`` is the closest match to the C++'s
``YAML::LoadFile`` -- it disallows arbitrary Python object construction
and reads mappings as plain ``dict`` and sequences as ``list``, which
is exactly what the C++ factories do with ``config["x"].as<T>()``.

Mapping table
-------------
The C++ has ``Text`` and ``MAPF_Benchmark`` spatial problem types and
``Geometric``, ``Explicit``, ``Turning``, ``Grid`` problem types. Of
those, ``Turning`` and ``Grid`` are not ported in this tree
(``ProblemInstanceWithTurning`` and ``ProblemInstanceWithGrid`` are
absent, see HANDOFF.md §1, PORTING.md §14); the factories raise
``NotImplementedError`` with a clear message when those keys are
encountered. ``CCBS`` is similarly out of scope for the planner
factory.

Roadmap generator types (``PRM``, ``PRMStar``, ``kPRM``, ``kPRMStar``,
``GRID``, ``CDT``) are all ported (Stage 3). The C++ exposes each
generator as a class with a ``generate(spatial)`` method and a
``problem_instance`` member; the Python port exposes them as free
functions, so ``make_generator`` wraps the function in a tiny
adapter class with the same surface (``generator.generate(spatial)``
then ``generator.problem_instance``).
"""
from __future__ import annotations

from typing import Any, Callable, List, Optional

from psipp_ctc.io import (
    load_grip_map,
    load_roadmap_problem,
    load_scenario,
    load_spatial_problem,
)
from psipp_ctc.planners import PrioritizedSIPP
from psipp_ctc.planners.planner import Planner
from psipp_ctc.problem import (
    ProblemInstance,
    ProblemInstanceWithGeometry,
    SpatialProblemInstance,
)
from psipp_ctc.roadmap import (
    generate_cdt,
    generate_grid,
    generate_kprm,
    generate_prm,
)


# ---------------------------------------------------------------------------
# Spatial problem (continuous-space, pre-roadmap)
# ---------------------------------------------------------------------------


def make_spatial_problem_instance(config: Any) -> SpatialProblemInstance:
    """Build a ``SpatialProblemInstance`` from a YAML config node.

    Mirrors ``makeSpatialProblemInstance`` in
    ``load_problems.cpp:16-37``. The C++ reads ``config["type"]`` and
    branches on ``"Text"`` (polygon file) or ``"MAPF_Benchmark"``
    (``.map`` + optional ``.scen``).

    For ``MAPF_Benchmark`` the C++ mutates ``agent_geometry`` from
    ``config["agent_size"]`` and, if a ``scenario`` key is present,
    calls ``loadScenario`` in place to append the ``.scen`` tasks
    (``load_problems.cpp:26-32``). The Python port reproduces the
    in-place semantics via ``load_scenario`` (``io/spatial.py:571``).
    """
    input_type = config["type"]
    if input_type == "Text":
        return load_spatial_problem(config["file"])
    if input_type == "MAPF_Benchmark":
        problem = load_grip_map(config["map"])
        problem.agent_geometry = float(config["agent_size"])
        if "scenario" in config and config["scenario"] is not None:
            load_scenario(config["scenario"], problem)
        return problem
    raise ValueError(f"Unknown Input Type: {input_type!r}")


# ---------------------------------------------------------------------------
# Problem instance (roadmap-bearing)
# ---------------------------------------------------------------------------


def make_problem_instance(config: Any) -> ProblemInstance:
    """Build a ``ProblemInstance`` from a YAML config node.

    Mirrors ``makeProblemInstance`` in ``load_problems.cpp:39-78``. The
    C++ reads ``config["type"]`` and branches on ``"Geometric"``,
    ``"Explicit"``, ``"Turning"``, ``"Grid"``. Of these, ``Geometric``
    and ``Explicit`` are ported; ``Turning`` and ``Grid`` are not in
    this tree (``HANDOFF.md §11`` follow-ups).

    After the type-specific build the C++ truncates ``tasks`` to
    ``config["number_of_agents"]`` if the key is present
    (``load_problems.cpp:75-77``). The Python port reproduces this
    truncation via ``list`` slicing; the explicit-collisions subclass
    inherits the new tasks via reference because ``ProblemInstance``
    stores ``tasks`` as a plain attribute (no defensive copy).
    """
    problem_type = config["type"]
    if problem_type == "Geometric":
        problem = load_roadmap_problem(config["file"])
    elif problem_type == "Explicit":
        # C++ constructs a ``ProblemInstanceWithGeometry<Geometry2D>``
        # from the file then wraps it in
        # ``ProblemInstanceWithExplicitCollisions``
        # (``load_problems.cpp:47-52``). The Python port does the same.
        from psipp_ctc.ctc import ProblemInstanceWithExplicitCollisions

        geometric = load_roadmap_problem(config["file"])
        problem = ProblemInstanceWithExplicitCollisions(geometric)
    elif problem_type in ("Turning", "Grid"):
        # Not in this tree; see HANDOFF.md §11.
        raise NotImplementedError(
            f"Problem type {problem_type!r} is not ported in this tree "
            f"(HANDOFF.md §11). The corresponding C++ subclasses "
            f"``ProblemInstanceWithTurning`` / ``ProblemInstanceWithGrid`` "
            f"are not implemented."
        )
    else:
        raise ValueError(f"Unknown Problem Type: {problem_type!r}")
    if "number_of_agents" in config and config["number_of_agents"] is not None:
        n = int(config["number_of_agents"])
        # ``ProblemInstance.tasks.resize(n)`` in the C++ truncates in
        # place. Python's ``list`` supports the same via slice
        # assignment. ``AgentTask`` is a ``NamedTuple`` so no field
        # mutation is needed.
        problem.tasks = problem.tasks[:n]
    return problem


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


def make_planner(config: Any) -> Planner:
    """Build a ``Planner`` from a YAML config node.

    Mirrors ``makePlanner`` in ``load_planners.cpp:12-28``. The C++
    reads ``config["type"]`` and branches on ``"CCBS"`` or
    ``"PSIPP"``; ``time_limit`` defaults to ``30.0`` when absent
    (``load_planners.cpp:14-18``). The C++ hardcodes
    ``priority_strategy = 0`` (FIFO) regardless of config
    (``load_planners.cpp:24``) and ``load_planners.cpp:24``'s comment
    notes ``priority_strategy`` is the first constructor argument; we
    pass it explicitly to match.
    """
    planner_type = config["type"]
    time_limit = float(config.get("time_limit", 30.0)) if config.get("time_limit") is not None else 30.0
    if planner_type == "PSIPP":
        return PrioritizedSIPP(priority_strategy=0, time_limit=time_limit)
    if planner_type == "CCBS":
        # ``WrappedContinuousCBS2`` is not ported (HANDOFF.md §11 item 4).
        raise NotImplementedError(
            "Planner type 'CCBS' is not ported in this tree "
            "(HANDOFF.md §11). The C++ wraps the vendored "
            "Continuous-CBS solver via ``WrappedContinuousCBS2``; "
            "the Python baseline strategy is still under design."
        )
    raise ValueError(f"Unknown Planner Type: {planner_type!r}")


# ---------------------------------------------------------------------------
# Generator (roadmap generator)
# ---------------------------------------------------------------------------


class _GeneratorAdapter:
    """Adapter that gives a Python generator function a C++-like surface.

    The C++ ``RoadmapGenerator<Space2D>`` base class
    (``base/roadmap_generator.hpp``) carries a ``problem_instance``
    member populated by ``generate(spatial)``. The Python port's
    generator entry points (Stage 3) are free functions that return a
    fresh ``ProblemInstanceWithGeometry``; this adapter calls the
    function and stores the result on ``self.problem_instance`` so the
    caller can write::

        generator = make_generator(config)
        generator.generate(spatial)
        print_roadmap_problem(generator.problem_instance, output_path)

    which mirrors the C++::

        auto generator = makeGenerator(config["Generator"]);
        generator->generate(*problem_instance);
        generator->problem_instance.print(output_file);
    """

    def __init__(self, generate_fn: Callable[[SpatialProblemInstance], ProblemInstanceWithGeometry]) -> None:
        self._generate_fn = generate_fn
        self.problem_instance: Optional[ProblemInstanceWithGeometry] = None

    def generate(self, spatial: SpatialProblemInstance) -> None:
        self.problem_instance = self._generate_fn(spatial)


def make_generator(config: Any) -> _GeneratorAdapter:
    """Build a roadmap generator from a YAML config node.

    Mirrors ``makeGenerator`` in ``load_generators.cpp:12-58``. The C++
    branches on ``PRM``, ``PRMStar``, ``kPRM``, ``kPRMStar``, ``GRID``,
    ``CDT`` and reads the per-type parameters from the config node.

    All six types are ported (Stage 3) so the factory can fully cover
    the C++ surface. Defaults match the C++:

    * ``CDT`` defaults ``number_of_additionals = 0`` and
      ``use_corner_points = true`` (``load_generators.cpp:45-52``).
    """
    generator_type = config["type"]
    if generator_type == "PRM":
        n_vertices = int(config["number_of_vertices"])
        radius = float(config["radius"])
        return _GeneratorAdapter(lambda s: generate_prm(s, n_vertices, radius))
    if generator_type == "PRMStar":
        n_vertices = int(config["number_of_vertices"])
        return _GeneratorAdapter(lambda s: generate_prm(s, n_vertices))
    if generator_type == "kPRM":
        n_vertices = int(config["number_of_vertices"])
        n_neighbors = int(config["number_of_neighbors"])
        return _GeneratorAdapter(lambda s: generate_kprm(s, n_vertices, n_neighbors))
    if generator_type == "kPRMStar":
        n_vertices = int(config["number_of_vertices"])
        return _GeneratorAdapter(lambda s: generate_kprm(s, n_vertices))
    if generator_type == "GRID":
        grid_size = float(config["grid_size"])
        K = int(config["K"])
        return _GeneratorAdapter(lambda s: generate_grid(s, grid_size, K))
    if generator_type == "CDT":
        n_add = int(config.get("number_of_additionals", 0) or 0)
        use_corners = bool(config.get("use_corner_points", True))
        return _GeneratorAdapter(lambda s: generate_cdt(s, n_add, use_corners))
    raise ValueError(f"Unknown Generator Type: {generator_type!r}")


__all__ = [
    "make_generator",
    "make_planner",
    "make_problem_instance",
    "make_spatial_problem_instance",
]
