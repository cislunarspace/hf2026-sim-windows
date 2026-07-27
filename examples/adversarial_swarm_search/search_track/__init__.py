"""Search-track submodule (placeholder).

Re-exports the reusable pieces from spec 016/017 (search-track FSM and
comm adapter). The spec 019 swarm controller builds on these — concrete
implementations land in subsequent phases (US4/US5).
"""

from . import (  # noqa: F401
    config,
    state,
)
from .swarm_controller import SwarmController  # noqa: F401
