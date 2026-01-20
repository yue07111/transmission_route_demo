"""Grid environment for transmission line routing demo.

Coordinate conventions:
- World coordinates are in meters, x to the right, y upward.
- Grid indexing is (ix, iy) where:
    ix in [0, grid_w-1], iy in [0, grid_h-1]
- Internally arrays are indexed [iy, ix] (row-major).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]
Point = Tuple[float, float]


@dataclass
class Environment:
    width_m: float
    height_m: float
    resolution_m: float

    # Populated after rasterization
    costmap: Optional[np.ndarray] = None
    hard_mask: Optional[np.ndarray] = None
    landable: Optional[np.ndarray] = None

    # For visualization / reporting
    entities: Optional[Dict[str, List[Dict]]] = None

    def __post_init__(self) -> None:
        self.grid_w = int(round(self.width_m / self.resolution_m))
        self.grid_h = int(round(self.height_m / self.resolution_m))

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.grid_w and 0 <= y < self.grid_h

    def world_to_grid(self, p: Point) -> Cell:
        x, y = p
        ix = int(round(x / self.resolution_m))
        iy = int(round(y / self.resolution_m))
        ix = max(0, min(self.grid_w - 1, ix))
        iy = max(0, min(self.grid_h - 1, iy))
        return (ix, iy)

    def grid_to_world(self, cell: Cell) -> Point:
        ix, iy = cell
        x = ix * self.resolution_m
        y = iy * self.resolution_m
        return (x, y)

    def cell_center_world(self, cell: Cell) -> Point:
        """Center point of cell in world coords."""
        ix, iy = cell
        x = (ix + 0.5) * self.resolution_m
        y = (iy + 0.5) * self.resolution_m
        return (x, y)

    def is_blocked(self, cell: Cell) -> bool:
        if self.hard_mask is None:
            return False
        x, y = cell
        return bool(self.hard_mask[y, x])

    def is_landable(self, cell: Cell) -> bool:
        if self.landable is None:
            return True
        x, y = cell
        return bool(self.landable[y, x])

    def cell_cost(self, cell: Cell) -> float:
        if self.costmap is None:
            return 0.0
        x, y = cell
        return float(self.costmap[y, x])


# 8-connected motion model
DIRS_8: List[Tuple[int, int]] = [
    (1, 0),   # 0 E
    (1, 1),   # 1 NE
    (0, 1),   # 2 N
    (-1, 1),  # 3 NW
    (-1, 0),  # 4 W
    (-1, -1), # 5 SW
    (0, -1),  # 6 S
    (1, -1),  # 7 SE
]


def heading_delta_deg(h1: int, h2: int) -> float:
    """Smallest turn angle between two headings (8-connect) in degrees."""
    if h1 < 0 or h2 < 0:
        return 0.0
    d = (h2 - h1) % 8
    d = min(d, 8 - d)
    return 45.0 * d


def step_distance_m(resolution_m: float, dir_idx: int) -> float:
    dx, dy = DIRS_8[dir_idx]
    return resolution_m * (2.0 ** 0.5 if (dx != 0 and dy != 0) else 1.0)
