"""Parallel corridor (existing line) reuse constraint.

Goal:
- Prefer routing near an existing corridor (reward / reduced cost)
- Prefer moves whose direction aligns with corridor tangent (directional reward)
- Respect a 'density limit': areas of already-high corridor density get penalized

In the full工程 this would be more sophisticated (line voltage等级、整体平移复用等);
here we implement the shaping terms so other algorithms can reuse the same cost field.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .base import Cell, Constraint, EdgeCostResult
from ..geometry import point_polyline_distance, polyline_tangent_at_nearest


@dataclass
class CorridorLine:
    id: str
    points: List[Tuple[float, float]]
    note: str = ""


class CorridorReuseConstraint(Constraint):
    name = "corridor_reuse"

    def __init__(self, primary: CorridorLine, other_lines: List[CorridorLine], cfg: Dict[str, Any]):
        self.primary = primary
        self.other_lines = other_lines
        self.cfg = cfg
        self.density_map: Optional[np.ndarray] = None

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        # Precompute density map: number of corridor lines within corridor_threshold_m.
        threshold = float(self.cfg.get("corridor_threshold_m", 300.0))
        yy, xx = np.indices(costmap.shape)
        wx = (xx + 0.5) * env.resolution_m
        wy = (yy + 0.5) * env.resolution_m

        density = np.zeros_like(costmap, dtype=np.float32)
        all_lines = [self.primary] + list(self.other_lines)
        # Brute-force but fine for demo grid sizes.
        for line in all_lines:
            # distance per cell
            # We'll compute in python loops over cells would be slow; instead we sample distances with a moderate step.
            # For demo sizes (~160*100), a nested loop is acceptable.
            for y in range(env.grid_h):
                for x in range(env.grid_w):
                    p = (float(wx[y, x]), float(wy[y, x]))
                    d = point_polyline_distance(p, line.points)
                    if d <= threshold:
                        density[y, x] += 1.0

        self.density_map = density

        # Penalty for exceeding density limit
        max_density = float(self.cfg.get("max_density", 2.0))
        dense_penalty = float(self.cfg.get("dense_penalty", 5.0))
        over = density > max_density
        if np.any(over):
            costmap[over] += dense_penalty * (density[over] - max_density)

        # Optional: static mild reward near primary corridor (as negative cost, clamped later)
        static_reward = float(self.cfg.get("static_reward", 0.0))
        if static_reward != 0:
            sigma = float(self.cfg.get("distance_sigma_m", threshold / 2.0))
            for y in range(env.grid_h):
                for x in range(env.grid_w):
                    p = (float(wx[y, x]), float(wy[y, x]))
                    d = point_polyline_distance(p, self.primary.points)
                    if d <= threshold:
                        w = math.exp(-(d * d) / (2.0 * sigma * sigma))
                        costmap[y, x] += static_reward * w

    def edge_cost(self, env, from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        # Directional reward: prefer to move parallel to corridor tangent when near corridor.
        threshold = float(self.cfg.get("corridor_threshold_m", 300.0))
        reward_factor = float(self.cfg.get("directional_reward_factor", 0.0))
        if reward_factor == 0.0:
            return EdgeCostResult(0.0, False, None)

        # Convert move to unit vector. We don't assume any particular motion model
        # beyond "from_cell" and "to_cell" being neighbors.
        if new_heading < 0:
            return EdgeCostResult(0.0, False, None)
        p0 = env.cell_center_world(from_cell)
        p1 = env.cell_center_world(to_cell)
        vx = p1[0] - p0[0]
        vy = p1[1] - p0[1]
        n = math.hypot(vx, vy)
        if n < 1e-9:
            return EdgeCostResult(0.0, False, None)
        v = (vx / n, vy / n)

        # Distance to primary corridor at the *to* cell
        pt = env.cell_center_world(to_cell)
        d = point_polyline_distance(pt, self.primary.points)
        if d > threshold:
            return EdgeCostResult(0.0, False, None)

        tangent = polyline_tangent_at_nearest(pt, self.primary.points)
        align = abs(v[0] * tangent[0] + v[1] * tangent[1])  # 0..1

        sigma = float(self.cfg.get("distance_sigma_m", threshold / 2.0))
        w = math.exp(-(d * d) / (2.0 * sigma * sigma))

        # reward reduces cost (negative). We'll clamp total step cost later to stay positive.
        reward = -reward_factor * w * align

        return EdgeCostResult(extra_cost=reward, hard_blocked=False, meta={"corridor_d": d, "align": align, "reward": reward})

    def post_validate(self, env, path_cells: list[Cell]) -> Dict[str, Any]:
        threshold = float(self.cfg.get("corridor_threshold_m", 300.0))
        near = 0
        for c in path_cells:
            d = point_polyline_distance(env.cell_center_world(c), self.primary.points)
            if d <= threshold:
                near += 1
        return {
            "name": self.name,
            "cells_near_primary_corridor": near,
            "ratio_near": near / max(1, len(path_cells)),
        }
