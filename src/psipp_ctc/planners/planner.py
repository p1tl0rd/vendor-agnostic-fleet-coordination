"""Stage 5 -- the planner interface.

Port of
  PSIPP-CTC_src/src/base/planner.hpp  (20 lines)

The reference is a tiny abstract base: ``setInstance`` to attach a
problem, ``solve`` to run the planner, ``succeeded`` to query the
result, and ``getPlan`` to retrieve the moves. The Python port keeps
the same surface but uses the standard ``abc.ABC`` machinery instead
of C++ virtual functions.

The ``Planner`` is decoupled from any specific ``ProblemInstance``
subclass -- the explicit-collisions subclass (``Stage 4``) is the one
PSIPP actually uses, but the geometric instance works too if the
online-query methods are defined. We type the argument as
``ProblemInstance`` for documentation; nothing here requires the
explicit-collisions materialisation.
"""

from __future__ import annotations

import abc
from typing import List

from psipp_ctc.problem import Plan, ProblemInstance


class Planner(abc.ABC):
    """Base class for all MAPF planners in this codebase.

    Mirrors the C++ ``Planner`` (``base/planner.hpp``). Subclasses
    implement ``solve`` and report success via ``succeeded``. The
    planner is single-shot: ``set_instance`` is called once, then
    ``solve``, then ``succeeded`` / ``get_plan`` may be inspected
    exactly once.

    The C++ ``solve()`` is a no-op in the base ("No real planner is
    specified" print) and the abstract bit is in ``succeeded``. In
    Python we make ``solve`` itself abstract -- there is no useful
    default and abstract-method machinery surfaces the contract
    clearly at instantiation time.

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

        Mirrors ``Planner::setInstance`` (``base/planner.hpp:7``).
        Called exactly once before ``solve``. The reference stores
        the instance as a ``shared_ptr``; Python uses plain
        reference-counting, so lifetime is automatic.
        """
        self.problem = problem

    @abc.abstractmethod
    def solve(self) -> None:
        """Run the planner on the attached instance.

        The result is stored on the instance (``success`` /
        ``getPlan()`` in the C++; the same fields in Python via
        the subclass). Mirrors ``Planner::solve``
        (``base/planner.hpp:11-15``).
        """
        raise NotImplementedError

    @abc.abstractmethod
    def succeeded(self) -> bool:
        """Return True iff the last ``solve`` produced a valid plan.

        Mirrors ``Planner::succeeded`` (``base/planner.hpp:16``).
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_plan(self) -> Plan:
        """Return the plan produced by the last ``solve``.

        Mirrors ``Planner::getPlan`` (``base/planner.hpp:17``).
        Returns an empty plan if the solver has not run or has
        failed -- the C++ returns the field directly, which is
        initialised to an empty vector.
        """
        return []  # type: ignore[return-value]


# Backwards-compat alias for the small number of internal call sites
# that already use the lower-case C++ style.
PlannerPtr = Planner
