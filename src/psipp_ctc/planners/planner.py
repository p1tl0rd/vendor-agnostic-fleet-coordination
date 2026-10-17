"""The planner interface.

A tiny abstract base: ``set_instance`` to attach a problem, ``solve``
to run the planner, ``succeeded`` to query the result, and ``get_plan``
to retrieve the moves. The base uses the standard ``abc.ABC`` machinery.

The ``Planner`` is decoupled from any specific ``ProblemInstance``
subclass -- the explicit-collisions subclass (``ctc/``) is the one
PSIPP actually uses, but the geometric instance works too if the
online-query methods are defined. The argument is typed as
``ProblemInstance`` for documentation; nothing here requires the
explicit-collisions materialisation.
"""

from __future__ import annotations

import abc
from typing import List

from psipp_ctc.problem import Plan, ProblemInstance


class Planner(abc.ABC):
    """Base class for all MAPF planners in this codebase.

    Subclasses implement ``solve`` and report success via
    ``succeeded``. The planner is single-shot: ``set_instance`` is
    called once, then ``solve``, then ``succeeded`` / ``get_plan``
    may be inspected exactly once.

    Attributes
    ----------
    problem : ProblemInstance | None
        The instance set by ``set_instance``. ``None`` before that
        call. Subclasses should fail loudly in ``solve`` if this is
        ``None``.
    """

    def __init__(self) -> None:
        self.problem: ProblemInstance | None = None

    def set_instance(self, problem: ProblemInstance) -> None:
        """Attach the problem instance to plan on.

        Called exactly once before ``solve``.
        """
        self.problem = problem

    @abc.abstractmethod
    def solve(self) -> None:
        """Run the planner on the attached instance.

        The result is stored on the instance and reported via
        ``succeeded`` / ``get_plan``.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def succeeded(self) -> bool:
        """Return True iff the last ``solve`` produced a valid plan."""
        raise NotImplementedError

    @abc.abstractmethod
    def get_plan(self) -> Plan:
        """Return the plan produced by the last ``solve``.

        Returns an empty plan if the solver has not run or has
        failed.
        """
        return []  # type: ignore[return-value]


# Backwards-compat alias for the small number of internal call sites
# that already use the lower-case C++ style.
PlannerPtr = Planner
