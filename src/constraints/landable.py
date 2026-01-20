"""Tower/turning-point landability constraints.

We maintain a `landable` boolean grid. The planner can enforce:
- turning points must be landable
- (post-processing) straight towers are placed only on landable cells

This corresponds to the "落点"约束 in the project docs.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .base import Cell, Constraint


def distance_to_mask_8(mask: np.ndarray) -> np.ndarray:
    """Compute approximate distance (in grid steps) to nearest True cell.

    Uses a multi-source BFS on an 8-connected grid (edge costs are 1 for all steps).
    This is not an exact Euclidean distance transform, but good enough for a demo.
    """
    h, w = mask.shape
    dist = np.full((h, w), np.inf, dtype=np.float32)
    q = collections.deque()

    ys, xs = np.where(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        dist[y, x] = 0.0
        q.append((x, y))

    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while q:
        x, y = q.popleft()
        d0 = dist[y, x]
        nd = d0 + 1.0
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and nd < dist[ny, nx]:
                dist[ny, nx] = nd
                q.append((nx, ny))
    return dist


class LandablePointsConstraint(Constraint):
    name = "landable_points"

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg

    def rasterize(self, env, costmap: np.ndarray, hard_mask: np.ndarray, landable: Optional[np.ndarray] = None) -> None:
        if landable is None:
            return

        # 1) Never land inside hard obstacles
        landable &= ~hard_mask

        # 2) Safety clearance from hard obstacles (buffer)
        min_clearance_m = float(self.cfg.get("min_clearance_m", 0.0))
        if min_clearance_m > 0:
            dist_cells = distance_to_mask_8(hard_mask)
            dist_m = dist_cells * env.resolution_m
            landable &= dist_m >= min_clearance_m

        # 3) Extra forbidden-radius buffer (fast filter) if provided
        forbidden_radius_m = float(self.cfg.get("forbidden_radius_m", 0.0))
        if forbidden_radius_m > 0:
            dist_cells = distance_to_mask_8(hard_mask)
            dist_m = dist_cells * env.resolution_m
            landable &= dist_m >= forbidden_radius_m

        # 4) Random non-landable patches (simulate geology /征地红线)
        rng_seed = int(self.cfg.get("random_seed", 0))
        frac = float(self.cfg.get("random_forbid_fraction", 0.0))
        if frac > 0:
            rng = np.random.default_rng(rng_seed)
            noise = rng.random(size=landable.shape)
            landable &= (noise > frac)

        # 5) Optional: add soft penalty for non-landable cells (planner can still pass through)
        # For this demo we keep it hard for turning points only, so no costmap update here.

    def post_validate(self, env, path_cells: list[Cell]) -> Dict[str, Any]:
        # Check turning points in the produced path.
        # Turning points are cells where direction changes.
        from ..environment import DIRS_8

        dirs = []
        for a, b in zip(path_cells[:-1], path_cells[1:]):
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            try:
                d = DIRS_8.index((int(np.sign(dx)), int(np.sign(dy))))
            except ValueError:
                d = None
            dirs.append(d)

        turning_cells = []
        for i in range(1, len(dirs)):
            if dirs[i] is None or dirs[i - 1] is None:
                continue
            if dirs[i] != dirs[i - 1]:
                turning_cells.append(path_cells[i])

        bad = [c for c in turning_cells if not env.is_landable(c)]
        return {
            "name": self.name,
            "turning_points": len(turning_cells),
            "turning_points_not_landable": len(bad),
            "bad_cells": bad[:10],
        }
