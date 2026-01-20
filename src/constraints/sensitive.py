"""Sensitive facility avoidance constraint (airport, meteorological station, military, etc.).

These are typically hard-avoid with a safety buffer.
We model them as circles:
- inside radius_hard_m: hard blocked
- within radius_soft_m: add penalty that decays with distance

This is a simplification of "区域避让" / "相邻设施影响" in the interview notes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .base import Cell, Constraint


@dataclass
class SensitivePoint:
    id: str
    center: Tuple[float, float]
    radius_hard_m: float
    radius_soft_m: float
    soft_penalty: float
    note: str = ""


class SensitiveFacilityConstraint(Constraint):
    name = "sensitive_facility"

    def __init__(self, points: List[SensitivePoint], cfg: Dict[str, Any]):
        self.points = points
        self.cfg = cfg

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        yy, xx = np.indices(costmap.shape)
        wx = (xx + 0.5) * env.resolution_m
        wy = (yy + 0.5) * env.resolution_m

        for p in self.points:
            dx = wx - p.center[0]
            dy = wy - p.center[1]
            dist = np.sqrt(dx * dx + dy * dy)
            hard = dist <= p.radius_hard_m
            hard_mask |= hard
            if landable is not None:
                landable &= ~hard

            # soft ring
            soft_r = max(p.radius_soft_m, p.radius_hard_m)
            if soft_r > p.radius_hard_m and p.soft_penalty > 0:
                ring = (dist > p.radius_hard_m) & (dist <= soft_r)
                sigma = float(self.cfg.get("sigma_m", (soft_r - p.radius_hard_m) / 2.0))
                # 0 at soft boundary, max near hard boundary
                w = np.exp(-((dist - p.radius_hard_m) ** 2) / (2.0 * sigma * sigma))
                costmap[ring] += float(p.soft_penalty) * w[ring]
