"""A* planner (heading-augmented) for the demo.

We augment state with a discrete heading index so we can support turning constraints
(turn angle limit, turn cost, and "turning point must be landable").

The planner itself is generic: costs come from the constraint system + static maps.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .environment import DIRS_8, step_distance_m
from .constraints.base import ConstraintManager, Cell


@dataclass
class PlanResult:
    path_cells: List[Cell]
    path_states: List[Tuple[int, int, int]]
    total_cost: float
    expansions: int
    visited_order: List[Cell]
    success: bool
    debug: Dict


class AStarPlanner:
    def __init__(self, cfg: Dict):
        self.cfg = cfg

    def heuristic(self, env, a: Cell, b: Cell) -> float:
        ax, ay = env.cell_center_world(a)
        bx, by = env.cell_center_world(b)
        return math.hypot(ax - bx, ay - by) * float(self.cfg.get("heuristic_weight", 1.0))

    def plan(self, env, constraints: ConstraintManager, start: Cell, goal: Cell, 
             progress_callback=None) -> PlanResult:
        # Ensure static maps exist
        if env.costmap is None or env.hard_mask is None:
            raise RuntimeError("env costmap/hard_mask not initialized; call ConstraintManager.rasterize_all first")

        # State is (x, y, heading) where heading in [-1,0..7]
        start_state = (start[0], start[1], -1)

        open_heap: List[Tuple[float, float, Tuple[int, int, int]]] = []
        heapq.heappush(open_heap, (0.0, 0.0, start_state))

        came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
        g_score: Dict[Tuple[int, int, int], float] = {start_state: 0.0}

        visited_order: List[Cell] = []
        expansions = 0
        
        # Dynamic bounding box for infinite map search
        # Calculate search bounds based on start/goal with padding
        search_padding = int(self.cfg.get("search_padding_cells", 200))  # Default 200 cells padding
        min_x = min(start[0], goal[0]) - search_padding
        max_x = max(start[0], goal[0]) + search_padding
        min_y = min(start[1], goal[1]) - search_padding
        max_y = max(start[1], goal[1]) + search_padding
        
        def in_search_bounds(cell: Cell) -> bool:
            """Check if cell is within dynamic search bounds"""
            x, y = cell
            return min_x <= x <= max_x and min_y <= y <= max_y

        base_w = float(self.cfg.get("base_distance_weight", 1.0))
        min_step_cost = float(self.cfg.get("min_step_cost", 1e-6))
        # Preference for straight movement (reduce unnecessary turns)
        straight_preference = float(self.cfg.get("straight_preference_bonus", 0.0))

        best_goal_state: Optional[Tuple[int, int, int]] = None
        best_goal_cost = float('inf')

        while open_heap:
            f, g, state = heapq.heappop(open_heap)
            x, y, heading = state
            cell = (x, y)

            # Skip stale entries
            if g > g_score.get(state, float('inf')) + 1e-9:
                continue

            expansions += 1
            visited_order.append(cell)
            
            # Call progress callback periodically to update visualization
            if progress_callback and expansions % 10 == 0:  # Update every 10 expansions
                progress_callback(visited_order.copy())

            if cell == goal:
                best_goal_state = state
                best_goal_cost = g
                break

            # expand neighbors
            for new_heading, (dx, dy) in enumerate(DIRS_8):
                nx, ny = x + dx, y + dy
                ncell = (nx, ny)
                # Use dynamic search bounds instead of fixed map bounds
                if not in_search_bounds(ncell):
                    continue
                if env.is_blocked(ncell):
                    continue

                # dynamic constraint checks
                ec = constraints.edge_cost(env, cell, ncell, heading, new_heading)
                if ec.hard_blocked:
                    continue

                base_step = step_distance_m(env.resolution_m, new_heading) * base_w
                node_extra = env.cell_cost(ncell)
                step_cost = base_step + node_extra + float(ec.extra_cost)
                
                # Prefer straight movement: give bonus for continuing in same direction
                # This reduces unnecessary turns while still allowing turns when needed
                if heading >= 0 and new_heading == heading:
                    # Straight movement - apply bonus (negative cost = reward)
                    step_cost -= straight_preference
                elif heading >= 0 and new_heading != heading:
                    # Turning movement - add extra penalty beyond turn_cost_weight
                    # This makes algorithm prefer straight paths when possible
                    turn_penalty = float(self.cfg.get("turn_penalty_extra", 0.0))
                    step_cost += turn_penalty

                # Clamp (needed because we allow negative rewards in costmap/edge cost)
                if step_cost < min_step_cost:
                    step_cost = min_step_cost

                ng = g + step_cost
                nstate = (nx, ny, new_heading)
                if ng + 1e-9 < g_score.get(nstate, float('inf')):
                    g_score[nstate] = ng
                    came_from[nstate] = state
                    nf = ng + self.heuristic(env, ncell, goal)
                    heapq.heappush(open_heap, (nf, ng, nstate))

        if best_goal_state is None:
            return PlanResult(
                path_cells=[],
                path_states=[],
                total_cost=float('inf'),
                expansions=expansions,
                visited_order=visited_order,
                success=False,
                debug={"reason": "no_path"},
            )

        # Reconstruct
        path_states: List[Tuple[int, int, int]] = [best_goal_state]
        cur = best_goal_state
        while cur != start_state:
            cur = came_from[cur]
            path_states.append(cur)
        path_states.reverse()
        path_cells = [(s[0], s[1]) for s in path_states]

        return PlanResult(
            path_cells=path_cells,
            path_states=path_states,
            total_cost=best_goal_cost,
            expansions=expansions,
            visited_order=visited_order,
            success=True,
            debug={"best_goal_state": best_goal_state},
        )
