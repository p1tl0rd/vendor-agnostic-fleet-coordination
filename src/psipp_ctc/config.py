"""YAML config loaders.

The ``make_*`` factories that the ``psipp-plan`` and
``psipp-generate-roadmap`` CLIs call to turn a parsed YAML config
into runtime objects: ``make_spatial_problem_instance``,
``make_problem_instance``, ``make_planner``, ``make_generator``.

The Python port uses ``pyyaml`` (already a project dependency) to
parse the config. ``yaml.safe_load`` is the closest match to
``yaml-cpp`` -- it disallows arbitrary Python object construction
and reads mappings as plain ``dict`` and sequences as ``list``.

Mapping table
-------------
The supported spatial problem types are ``Text`` and
``MAPF_Benchmark``. The supported problem-instance types are
``Geometric`` and ``Explicit``. ``Turning`` and ``Grid`` are not
in this tree; the factories raise ``NotImplementedError`` with a
clear message when those keys are encountered. ``CCBS`` is
similarly out of scope for the planner factory.

Roadmap generator types (``PRM``, ``PRMStar``, ``kPRM``,
``kPRMStar``, ``GRID``, ``CDT``) are all available. The
``make_generator`` factory wraps the underlying function in a
small adapter class with a ``generate(spatial)`` method and a
``problem_instance`` member, so the caller can write::

    generator = make_generator(config)
    generator.generate(spatial)
    print_roadmap_problem(generator.problem_instance, output_path)
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

    Reads ``config["type"]`` and branches on ``"Text"`` (polygon file)
    or ``"MAPF_Benchmark"`` (``.map`` + optional ``.scen``).

    For ``MAPF_Benchmark``, sets ``agent_geometry`` from
    ``config["agent_size"]`` and, if a ``scenario`` key is present,
    calls ``load_scenario`` in place to append the ``.scen`` tasks.
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

    Reads ``config["type"]`` and branches on ``"Geometric"``,
    ``"Explicit"``, ``"Turning"``, ``"Grid"``. Of these, ``Geometric``
    and ``Explicit`` are supported; ``Turning`` and ``Grid`` are
    not in this tree.

    After the type-specific build, truncates ``tasks`` to
    ``config["number_of_agents"]`` if the key is present. The
    explicit-collisions subclass inherits the new tasks via
    reference because ``ProblemInstance`` stores ``tasks`` as a
    plain attribute.
    """
    problem_type = config["type"]
    if problem_type == "Geometric":
        problem = load_roadmap_problem(config["file"])
    elif problem_type == "Explicit":
        # Build a geometric instance from the file and wrap it in
        # the explicit-collisions subclass.
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

    Reads ``config["type"]`` and branches on ``"CCBS"`` or
    ``"PSIPP"``; ``time_limit`` defaults to ``30.0`` when absent.
    Always passes ``priority_strategy = 0`` (FIFO), the default the
    C++ hardcodes.
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

    The Python's generator entry points are free functions that
    return a fresh ``ProblemInstanceWithGeometry``; this adapter
    calls the function and stores the result on
    ``self.problem_instance`` so the caller can write::

        generator = make_generator(config)
        generator.generate(spatial)
        print_roadmap_problem(generator.problem_instance, output_path)
    """

    def __init__(self, generate_fn: Callable[[SpatialProblemInstance], ProblemInstanceWithGeometry]) -> None:
        self._generate_fn = generate_fn
        self.problem_instance: Optional[ProblemInstanceWithGeometry] = None

    def generate(self, spatial: SpatialProblemInstance) -> None:
        self.problem_instance = self._generate_fn(spatial)


def make_generator(config: Any) -> _GeneratorAdapter:
    """Build a roadmap generator from a YAML config node.

    Branches on ``PRM``, ``PRMStar``, ``kPRM``, ``kPRMStar``, ``GRID``,
    ``CDT`` and reads the per-type parameters from the config node.

    ``CDT`` defaults ``number_of_additionals = 0`` and
    ``use_corner_points = true``.
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
