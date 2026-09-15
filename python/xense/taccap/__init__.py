"""TacCap customer API: Gripper, GripperState and logging.

Device components, controller tuning and wire protocol APIs live in ``advanced``.
Existing named imports remain compatible, including ``FollowerGripper`` and
``ControlLoop``. New applications should start with ``Gripper``.
"""
from . import advanced
from ._version import __version__ as __version__
from .gripper import Gripper
from .gripper_types import (
    BusyError,
    FaultError,
    GripperDevice,
    GripperDiagnostics,
    GripperError,
    GripperInfo,
    GripperLimits,
    GripperResult,
    GripperState,
    GripperTiming,
    OperationInterrupted,
)

log = advanced.log
ProtocolError = advanced.ProtocolError
IoError = advanced.IoError
TimeoutError = advanced.TimeoutError

# Retain wildcard compatibility too; old names are resolved on demand rather
# than making every protocol constant a primary package attribute.
__all__ = [
    "BusyError",
    "FaultError",
    "Gripper",
    "GripperDevice",
    "GripperDiagnostics",
    "GripperError",
    "GripperInfo",
    "GripperLimits",
    "GripperResult",
    "GripperState",
    "GripperTiming",
    "OperationInterrupted",
    "advanced",
]
__all__.extend(advanced.__all__)


def __getattr__(name):
    if name in advanced.__all__:
        return getattr(advanced, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
