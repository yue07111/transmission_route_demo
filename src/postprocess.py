"""Post-processing: tower placement on the found route.

This implements (in a simplified way) the "直线塔约束" part:
- maximum span <= max_span_m
- each segment between turning points must have at least min_straight_tower_per_segment
- towers must be placed on landable cells

For the demo we place straight towers by selecting points along the grid path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from .environment import DIRS_8, step_distance_m

Cell = Tuple[int, int]


def extract_turning_indices(path: List[Cell]) -> List[int]:
    """Return indices of turning points including start(0) and end(n-1)."""
    if len(path) < 2:
        return [0] if path else []
    dirs = []
    for a, b in zip(path[:-1], path[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dx = int(np.sign(dx))
        dy = int(np.sign(dy))
        try:
            d = DIRS_8.index((dx, dy))
        except ValueError:
            d = None
        dirs.append(d)

    idx = [0]
    for i in range(1, len(dirs)):
        if dirs[i] is None or dirs[i - 1] is None:
            continue
        if dirs[i] != dirs[i - 1]:
            idx.append(i)  # turning cell is path[i]
    idx.append(len(path) - 1)
    # unique sorted
    idx = sorted(set(idx))
    return idx


def segment_length_m(env, segment_cells: List[Cell]) -> float:
    """Sum step distances along a list of cells."""
    if len(segment_cells) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(segment_cells[:-1], segment_cells[1:]):
        dx = int(np.sign(b[0] - a[0]))
        dy = int(np.sign(b[1] - a[1]))
        d = DIRS_8.index((dx, dy))
        total += step_distance_m(env.resolution_m, d)
    return total


def nearest_landable(env, target: Cell, radius_cells: int) -> Cell | None:
    """Find a nearby landable, non-blocked cell within radius (Chebyshev)."""
    tx, ty = target
    best = None
    best_d2 = 1e18
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            x, y = tx + dx, ty + dy
            c = (x, y)
            if not env.in_bounds(c):
                continue
            if env.is_blocked(c):
                continue
            if not env.is_landable(c):
                continue
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best = c
                best_d2 = d2
    return best


@dataclass
class TowerPlacement:
    turning_towers: List[Cell]
    straight_towers: List[Cell]
    tower_sequence: List[Cell]
    max_span_m: float
    segments: List[Dict]


def place_towers(env, path: List[Cell], cfg: Dict) -> TowerPlacement:
    max_span_m = float(cfg.get("max_span_m", 800.0))
    min_straight = int(cfg.get("min_straight_tower_per_segment", 1))
    search_r = int(cfg.get("straight_tower_search_radius_cells", 3))

    if not path:
        return TowerPlacement([], [], [], 0.0, [])

    turning_idx = extract_turning_indices(path)
    turning_towers = [path[i] for i in turning_idx]

    straight_towers: List[Cell] = []
    tower_sequence: List[Cell] = [turning_towers[0]]
    segments_info: List[Dict] = []

    for i in range(len(turning_idx) - 1):
        a_idx = turning_idx[i]
        b_idx = turning_idx[i + 1]
        seg_cells = path[a_idx:b_idx + 1]
        L = segment_length_m(env, seg_cells)

        # required straight towers
        n_req = max(min_straight, max(0, math.ceil(L / max_span_m) - 1))

        # Evenly pick indices along seg (excluding endpoints)
        chosen: List[Cell] = []
        if len(seg_cells) >= 3 and n_req > 0:
            for k in range(1, n_req + 1):
                t = k / (n_req + 1)
                idx = int(round(t * (len(seg_cells) - 1)))
                idx = max(1, min(len(seg_cells) - 2, idx))
                cand = seg_cells[idx]
                if not env.is_landable(cand) or env.is_blocked(cand):
                    alt = nearest_landable(env, cand, search_r)
                    if alt is not None:
                        cand = alt
                chosen.append(cand)

        # Deduplicate while preserving order
        seen = set()
        chosen2 = []
        for c in chosen:
            if c not in seen:
                chosen2.append(c)
                seen.add(c)
        chosen = chosen2

        # Append to tower sequence
        for c in chosen:
            straight_towers.append(c)
            tower_sequence.append(c)
        tower_sequence.append(path[b_idx])

        segments_info.append({
            "segment_index": i,
            "from": path[a_idx],
            "to": path[b_idx],
            "length_m": L,
            "n_required": n_req,
            "straight_towers": chosen,
        })

    # Recompute max span along the tower sequence
    max_span = 0.0
    for a, b in zip(tower_sequence[:-1], tower_sequence[1:]):
        ax, ay = env.cell_center_world(a)
        bx, by = env.cell_center_world(b)
        max_span = max(max_span, math.hypot(ax - bx, ay - by))

    return TowerPlacement(
        turning_towers=turning_towers,
        straight_towers=straight_towers,
        tower_sequence=tower_sequence,
        max_span_m=max_span,
        segments=segments_info,
    )
