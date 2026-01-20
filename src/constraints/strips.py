"""Strip (rectangle) crossing constraints.

We represent linear obstacles (existing lines, pipelines, etc.) as oriented rectangles.
When a move segment intersects a rectangle:
- if is_crossable=False => hard block
- else require crossing angle >= min_cross_angle_deg
- add cross_cost + optional angle-dependent cost

This follows the rules in `路径优化场景约束条件定义_v1.0` section 3.1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .base import Cell, Constraint, EdgeCostResult
from ..geometry import segment_intersects_oriented_rect, smallest_undirected_angle_deg


@dataclass
class StripRect:
    id: str
    center: Tuple[float, float]
    length_m: float
    width_m: float
    angle_deg: float

    is_crossable: bool = True
    min_cross_angle_deg: float = 30.0
    cross_cost: float = 0.0
    note: str = ""

    def axis_dir(self) -> Tuple[float, float]:
        th = math.radians(self.angle_deg)
        return (math.cos(th), math.sin(th))


class StripCrossingConstraint(Constraint):
    name = "strip_crossing"

    def __init__(self, strips: list[StripRect], cfg: Dict[str, Any]):
        self.strips = strips
        self.cfg = cfg

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        # Optionally mark non-crossable rectangles as hard obstacles.
        # For crossable ones, we can also add a mild static cost inside to discourage
        # staying inside the strip.
        yy, xx = np.indices(costmap.shape)
        wx = (xx + 0.5) * env.resolution_m
        wy = (yy + 0.5) * env.resolution_m

        for s in self.strips:
            # Transform grid points into strip-local frame: rotate by -angle, translate by -center.
            th = math.radians(s.angle_deg)
            c = math.cos(-th)
            si = math.sin(-th)
            dx = wx - s.center[0]
            dy = wy - s.center[1]
            lx = c * dx - si * dy
            ly = si * dx + c * dy
            hx = s.length_m / 2.0
            hy = s.width_m / 2.0
            inside = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)

            if not s.is_crossable:
                hard_mask |= inside
                if landable is not None:
                    landable &= ~inside
            else:
                per_cell = float(self.cfg.get("crossable_strip_cell_cost", 0.0))
                if per_cell > 0:
                    costmap[inside] += per_cell

    def edge_cost(self, env, from_cell: Cell, to_cell: Cell, heading: int, new_heading: int) -> EdgeCostResult:
        if env.is_blocked(to_cell):
            return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"reason": "blocked_cell"})

        p0 = env.cell_center_world(from_cell)
        p1 = env.cell_center_world(to_cell)
        vx = p1[0] - p0[0]
        vy = p1[1] - p0[1]

        extra = 0.0
        crossed: list[str] = []
        min_angle_violation: list[Tuple[str, float, float]] = []  # (id, angle, min)

        for s in self.strips:
            th = math.radians(s.angle_deg)
            if segment_intersects_oriented_rect(p0, p1, s.center, s.length_m, s.width_m, th):
                if not s.is_crossable:
                    return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True, meta={"blocked_by": s.id})

                # crossing angle between move direction and strip axis direction
                axis = s.axis_dir()
                ang = smallest_undirected_angle_deg((vx, vy), axis)
                if ang < float(s.min_cross_angle_deg):
                    min_angle_violation.append((s.id, ang, float(s.min_cross_angle_deg)))
                    # treat as hard for demo (as spec)
                    return EdgeCostResult(extra_cost=float("inf"), hard_blocked=True,
                                          meta={"angle_deg": ang, "min_required": s.min_cross_angle_deg, "strip": s.id})

                crossed.append(s.id)
                extra += float(s.cross_cost)

                # optional: angle soft cost to prefer higher angles (more perpendicular)
                mode = str(self.cfg.get("angle_preference", "prefer_perpendicular"))
                w = float(self.cfg.get("angle_cost_weight", 0.0))
                if w > 0:
                    if mode == "prefer_perpendicular":
                        # smaller penalty for larger angles
                        extra += w * (1.0 / max(ang, 1.0))
                    elif mode == "prefer_parallel":
                        extra += w * (ang / 90.0)

        meta = {"crossed": crossed} if crossed else None
        return EdgeCostResult(extra_cost=extra, hard_blocked=False, meta=meta)

    def post_validate(self, env, path_cells: list[Cell]) -> Dict[str, Any]:
        # Re-check crossings for the final path.
        crossings: Dict[str, int] = {s.id: 0 for s in self.strips}
        min_cross_angle: Dict[str, float] = {s.id: 999.0 for s in self.strips}
        ok = True
        violations = []
        for a, b in zip(path_cells[:-1], path_cells[1:]):
            p0 = env.cell_center_world(a)
            p1 = env.cell_center_world(b)
            vx, vy = p1[0] - p0[0], p1[1] - p0[1]
            for s in self.strips:
                th = math.radians(s.angle_deg)
                if segment_intersects_oriented_rect(p0, p1, s.center, s.length_m, s.width_m, th):
                    if not s.is_crossable:
                        ok = False
                        violations.append({"strip": s.id, "reason": "non_crossable"})
                    else:
                        axis = s.axis_dir()
                        ang = smallest_undirected_angle_deg((vx, vy), axis)
                        crossings[s.id] += 1
                        min_cross_angle[s.id] = min(min_cross_angle[s.id], ang)
                        if ang < s.min_cross_angle_deg:
                            ok = False
                            violations.append({"strip": s.id, "reason": "min_angle", "angle": ang, "min": s.min_cross_angle_deg})
        return {
            "name": self.name,
            "ok": ok,
            "crossings": crossings,
            "min_cross_angle_deg": {k: (None if v == 999.0 else v) for k, v in min_cross_angle.items()},
            "violations": violations,
        }
