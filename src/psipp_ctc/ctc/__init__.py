"""Stage 4 -- offline CTC annotation.

The explicit-collisions materialisation. Construction flattens the
roadmap into global edge indices, runs the broad phases (Buckets2D
for VV and VEC, BentleyOttoman for EE), and stores one exact
interval per surviving pair. Online queries are then pure map
lookups; the geometry kernels are no longer called.

Mirrors ``PSIPP-CTC_src/src/base/problem_with_explicit_collisions.hpp``.
"""

from psipp_ctc.ctc.explicit_collisions import (
    ProblemInstanceWithExplicitCollisions,
)

__all__ = ["ProblemInstanceWithExplicitCollisions"]
