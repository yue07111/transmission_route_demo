"""Constraint interface.

Each constraint can:
- rasterize static effects onto the grid (hard obstacles + soft cost heatmap)
- provide dynamic per-edge costs (e.g., crossing angle, corridor direction reward)
- expose helpers for post-processing validation

Design goal: keep the planner algorithm-agnostic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]


@dataclass
class EdgeCostResult:
    extra_cost: float = 0.0
    hard_blocked: bool = False
    meta: Optional[Dict[str, Any]] = None


class Constraint:
    """Base class."""

    name: str = "constraint"

    def rasterize(
        self,
        env: "Environment",
        costmap: np.ndarray,
        hard_mask: np.ndarray,
        landable: Optional[np.ndarray] = None,
    ) -> None:
        """Apply static effects.

        - costmap: float32 cost multiplier/additive map (implementation-defined)
        - hard_mask: bool True means forbidden cell
        - landable: optional bool map (can be updated by constraints that affect tower landing)
        """
        return

    def edge_cost(self, env: "Environment", from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        """Dynamic per-edge check/cost.

        heading/new_heading are direction indices in [0..7] or -1 for 'no heading'.
        """
        return EdgeCostResult(0.0, False, None)

    def post_validate(self, env: "Environment", path_cells: list[Cell]) -> Dict[str, Any]:
        """Optional: validate final polyline / path. Return dict for reporting."""
        return {"name": self.name, "ok": True}


class ConstraintManager:
    def __init__(self, constraints: list[Constraint]):
        self.constraints = constraints

    def rasterize_all(self, env: "Environment") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (costmap, hard_mask, landable_mask)."""
        costmap = np.zeros((env.grid_h, env.grid_w), dtype=np.float32)
        hard = np.zeros((env.grid_h, env.grid_w), dtype=bool)
        landable = np.ones((env.grid_h, env.grid_w), dtype=bool)

        for c in self.constraints:
            c.rasterize(env, costmap, hard, landable)

        return costmap, hard, landable

    def edge_cost(self, env: "Environment", from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        extra = 0.0
        meta: Dict[str, Any] = {}
        for c in self.constraints:
            r = c.edge_cost(env, from_cell, to_cell, heading, new_heading)
            if r.hard_blocked:
                return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"blocked_by": c.name, **(r.meta or {})})
            extra += r.extra_cost
            if r.meta:
                meta[c.name] = r.meta
        return EdgeCostResult(extra_cost=extra, hard_blocked=False, meta=meta or None)

    def post_validate_all(self, env: "Environment", path_cells: list[Cell]) -> list[Dict[str, Any]]:
        return [c.post_validate(env, path_cells) for c in self.constraints]


# Avoid circular import for type checking.
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..environment import Environment
