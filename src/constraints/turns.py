"""Turning / angle constraint.

From interview notes, turning angle should be within 0-90 degrees and
smaller turning cost is preferred.

In this grid demo:
- we restrict heading changes to <= max_turn_angle_deg (hard)
- we add a turn cost when heading changes (soft)
- we can require the *turning point cell* to be landable
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

from .base import Cell, Constraint, EdgeCostResult
from ..environment import heading_delta_deg


class TurnAngleConstraint(Constraint):
    name = "turn_angle"

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg

    def edge_cost(self, env, from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        # Start state may have heading=-1.
        if heading < 0:
            return EdgeCostResult(0.0, False, None)

        ang = heading_delta_deg(heading, new_heading)
        max_ang = float(self.cfg.get("max_turn_angle_deg", 90.0))
        if ang > max_ang + 1e-6:
            return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"angle_deg": ang, "max": max_ang})

        if ang > 1e-6:
            # Require turning point to be landable
            if bool(self.cfg.get("require_turn_landable", True)) and (not env.is_landable(from_cell)):
                return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"reason": "turn_not_landable", "angle_deg": ang})

            # Turn cost
            w = float(self.cfg.get("turn_cost_weight", 0.0))
            if w > 0:
                # normalize by max_ang
                return EdgeCostResult(extra_cost=w * (ang / max_ang), hard_blocked=False, meta={"turn_angle_deg": ang})
            return EdgeCostResult(0.0, False, meta={"turn_angle_deg": ang})

        return EdgeCostResult(0.0, False, None)
