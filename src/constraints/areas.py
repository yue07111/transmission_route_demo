"""Circular area constraints (enter/cross + damping + entry penalty).

This matches the idea in `路径优化场景约束条件定义_v1.0`:
- non-enterable / non-crossable => hard obstacle
- otherwise: entry penalty when entering; per-step cost while inside (damping/cross_cost)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .base import Cell, Constraint, EdgeCostResult


@dataclass
class CircleArea:
    id: str
    center: Tuple[float, float]  # world meters
    radius_m: float

    is_enterable: bool = True
    is_crossable: bool = True

    # Soft costs
    damping: float = 0.0  # per-step additive multiplier-ish
    entry_penalty: float = 0.0
    cross_cost: float = 0.0

    note: str = ""


class AreaDampingConstraint(Constraint):
    name = "area_damping"

    def __init__(self, areas: list[CircleArea], cfg: Dict[str, Any]):
        self.areas = areas
        self.cfg = cfg

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        # Add static per-cell cost inside enterable areas; mark hard obstacles for non-crossable.
        # We use a simple inside-test on cell centers.
        yy, xx = np.indices(costmap.shape)
        # Convert to world center
        wx = (xx + 0.5) * env.resolution_m
        wy = (yy + 0.5) * env.resolution_m

        for a in self.areas:
            dx = wx - a.center[0]
            dy = wy - a.center[1]
            dist = np.sqrt(dx * dx + dy * dy)
            inside = dist <= a.radius_m

            if (not a.is_enterable) or (not a.is_crossable):
                hard_mask |= inside
                if landable is not None:
                    landable &= ~inside
                continue

            # Soft penalty inside
            per_cell = float(a.cross_cost) + float(a.damping) * float(self.cfg.get("damping_cell_cost", 1.0))
            costmap[inside] += per_cell

            # Optional: soft buffer around area (to encourage clearance)
            buffer_m = float(self.cfg.get("soft_buffer_m", 0.0))
            if buffer_m > 0:
                in_buffer = (dist > a.radius_m) & (dist <= a.radius_m + buffer_m)
                # Linear ramp penalty
                ramp = (a.radius_m + buffer_m - dist) / max(buffer_m, 1e-6)
                costmap[in_buffer] += float(self.cfg.get("buffer_cost", 1.0)) * ramp[in_buffer]

    def edge_cost(self, env, from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        # Entry penalty: if moving from outside->inside for any area.
        # Also block if to_cell is hard (already handled by env mask, but keep safe).
        if env.is_blocked(to_cell):
            return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"reason": "blocked_cell"})

        fx, fy = env.cell_center_world(from_cell)
        tx, ty = env.cell_center_world(to_cell)

        extra = 0.0
        entered: list[str] = []
        for a in self.areas:
            # Skip hard ones (already rasterized)
            if (not a.is_enterable) or (not a.is_crossable):
                continue
            f_in = (fx - a.center[0]) ** 2 + (fy - a.center[1]) ** 2 <= a.radius_m ** 2
            t_in = (tx - a.center[0]) ** 2 + (ty - a.center[1]) ** 2 <= a.radius_m ** 2
            if (not f_in) and t_in and a.entry_penalty > 0:
                extra += float(a.entry_penalty)
                entered.append(a.id)
        meta = {"entered": entered} if entered else None
        return EdgeCostResult(extra_cost=extra, hard_blocked=False, meta=meta)

    def post_validate(self, env, path_cells: list[Cell]) -> Dict[str, Any]:
        # Count entered areas and total length in each.
        entered = set()
        inside_counts: Dict[str, int] = {a.id: 0 for a in self.areas}
        for c in path_cells:
            x, y = env.cell_center_world(c)
            for a in self.areas:
                if (x - a.center[0]) ** 2 + (y - a.center[1]) ** 2 <= a.radius_m ** 2:
                    inside_counts[a.id] += 1
                    entered.add(a.id)
        return {
            "name": self.name,
            "entered_areas": sorted(list(entered)),
            "cells_in_areas": inside_counts,
        }
