"""Road access reward constraint.

Interview notes mention leveraging existing roads for construction access.
We model roads as polylines and reduce cost near them (soft reward).

Implementation:
- static reward field near roads: costmap += negative values (clamped later)
- optional directional preference to move parallel to road tangent
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .base import Cell, Constraint, EdgeCostResult
from ..geometry import point_polyline_distance, polyline_tangent_at_nearest


@dataclass
class RoadLine:
    id: str
    points: List[Tuple[float, float]]
    note: str = ""


class RoadRewardConstraint(Constraint):
    name = "road_reward"

    def __init__(self, roads: List[RoadLine], cfg: Dict[str, Any]):
        self.roads = roads
        self.cfg = cfg

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        threshold = float(self.cfg.get("road_threshold_m", 250.0))
        static_reward = float(self.cfg.get("static_reward", -0.5))
        if static_reward == 0.0:
            return

        sigma = float(self.cfg.get("distance_sigma_m", threshold / 2.0))
        yy, xx = np.indices(costmap.shape)
        wx = (xx + 0.5) * env.resolution_m
        wy = (yy + 0.5) * env.resolution_m

        for y in range(env.grid_h):
            for x in range(env.grid_w):
                p = (float(wx[y, x]), float(wy[y, x]))
                d_best = float('inf')
                for r in self.roads:
                    d_best = min(d_best, point_polyline_distance(p, r.points))
                if d_best <= threshold:
                    w = math.exp(-(d_best * d_best) / (2.0 * sigma * sigma))
                    costmap[y, x] += static_reward * w

    def edge_cost(self, env, from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        directional = float(self.cfg.get("directional_reward_factor", 0.0))
        if directional == 0.0 or not self.roads:
            return EdgeCostResult(0.0, False, None)

        threshold = float(self.cfg.get("road_threshold_m", 250.0))
        sigma = float(self.cfg.get("distance_sigma_m", threshold / 2.0))

        # move unit vector
        p0 = env.cell_center_world(from_cell)
        p1 = env.cell_center_world(to_cell)
        vx, vy = p1[0] - p0[0], p1[1] - p0[1]
        n = math.hypot(vx, vy)
        if n < 1e-9:
            return EdgeCostResult(0.0, False, None)
        v = (vx / n, vy / n)

        pt = env.cell_center_world(to_cell)
        # nearest road tangent
        best_d = float('inf')
        best_t = (1.0, 0.0)
        for r in self.roads:
            d = point_polyline_distance(pt, r.points)
            if d < best_d:
                best_d = d
                best_t = polyline_tangent_at_nearest(pt, r.points)

        if best_d > threshold:
            return EdgeCostResult(0.0, False, None)

        align = abs(v[0] * best_t[0] + v[1] * best_t[1])
        w = math.exp(-(best_d * best_d) / (2.0 * sigma * sigma))
        reward = -directional * w * align
        return EdgeCostResult(extra_cost=reward, hard_blocked=False, meta={"road_d": best_d, "align": align, "reward": reward})
