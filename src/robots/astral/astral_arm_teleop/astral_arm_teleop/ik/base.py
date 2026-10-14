"""IK solver abstract interface for Astral dual-arm solvers."""

from abc import ABC, abstractmethod
from typing import Any, Dict, List


class IKSolverBase(ABC):
    """RobotMain dual-arm IK base.

    Internal full state ``q_full`` (14-DoF)::

      q_full[0:7]   left  (Joint_la_1 .. Joint_la_7)
      q_full[7:14]  right (Joint_ra_1 .. Joint_ra_7)

    Single-arm solve only updates that arm; the other side is held.
    """

    @abstractmethod
    def solve_ik(
        self,
        arm: str,
        pos: List[float],
        rpy: List[float],
        lock_joint7: bool = False,
    ) -> Dict[str, Any]:
        """Solve one arm.

        Args:
            arm: ``\"L\"`` | ``\"R\"``
            pos: [x, y, z] in ``base_link`` (m)
            rpy: [roll, pitch, yaw] in ``base_link`` (rad)
            lock_joint7: reserved

        Returns:
            dict with ``ik_ok``, ``ik_err``, ``arm``, ``ik_method``;
            on success ``q_full`` for that arm is updated.
        """
        ...

    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @property
    @abstractmethod
    def q_full(self) -> List[float]:
        ...

    @property
    @abstractmethod
    def method_name(self) -> str:
        ...
