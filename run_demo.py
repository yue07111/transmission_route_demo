#!/usr/bin/env python
"""Run the transmission line routing demo.

Usage:
  python run_demo.py --config configs/demo_config.yaml --map configs/demo_map.yaml

Outputs are written to outputs/<timestamp>/
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from datetime import datetime

import yaml

from src.builder import build_from_spec
from src.astar import AStarPlanner
from src.postprocess import place_towers
from src.visualization import plot_static, save_figure, make_gif


def load_yaml(path: str):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=str, default='configs/demo_config.yaml')
    ap.add_argument('--map', type=str, default='configs/demo_map.yaml')
    ap.add_argument('--out', type=str, default=None, help='output directory (default: outputs/<timestamp>)')
    args = ap.parse_args()

    root = Path(__file__).resolve().parent
    cfg = load_yaml(str(root / args.config))
    map_data = load_yaml(str(root / args.map))

    out_dir = Path(args.out) if args.out else (root / 'outputs' / datetime.now().strftime('%Y%m%d_%H%M%S'))
    out_dir.mkdir(parents=True, exist_ok=True)

    # Build env + constraints
    env, cm = build_from_spec(cfg, map_data)

    start = tuple(map_data['start'])
    goal = tuple(map_data['goal'])
    start_cell = env.world_to_grid(tuple(start))
    goal_cell = env.world_to_grid(tuple(goal))

    # Plan
    planner_cfg = cfg.get('planner', {})
    planner = AStarPlanner(planner_cfg)
    result = planner.plan(env, cm, start_cell, goal_cell)

    report = {
        'success': result.success,
        'total_cost': result.total_cost,
        'expansions': result.expansions,
        'path_cells': len(result.path_cells),
        'start_cell': start_cell,
        'goal_cell': goal_cell,
    }

    if result.success:
        # Postprocess towers
        towers = place_towers(env, result.path_cells, cfg.get('postprocess', {}))
        report['towers'] = {
            'turning_towers': len(towers.turning_towers),
            'straight_towers': len(towers.straight_towers),
            'tower_sequence': len(towers.tower_sequence),
            'max_span_m': towers.max_span_m,
            'segments': towers.segments,
        }

        # Constraint validation
        report['constraint_checks'] = cm.post_validate_all(env, result.path_cells)

        # Save report
        with open(out_dir / 'report.json', 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=2)

        # Static figures
        vis_cfg = cfg.get('visualization', {})
        fig, _ = plot_static(
            env,
            map_data,
            path=result.path_cells,
            visited=result.visited_order,
            towers={'turning': towers.turning_towers, 'straight': towers.straight_towers},
            show_heatmap=bool(vis_cfg.get('show_heatmap', True)),
            heatmap_clip_percentile=float(vis_cfg.get('heatmap_clip_percentile', 95)),
            title='A* route + constraints (demo)',
        )
        save_figure(fig, str(out_dir / 'final.png'), dpi=int(vis_cfg.get('output_dpi', 150)))

        # Penalty heatmap only
        fig2, _ = plot_static(
            env,
            map_data,
            path=None,
            visited=None,
            towers=None,
            show_heatmap=True,
            heatmap_clip_percentile=float(vis_cfg.get('heatmap_clip_percentile', 95)),
            title='Static penalty heatmap (demo)',
        )
        save_figure(fig2, str(out_dir / 'heatmap.png'), dpi=int(vis_cfg.get('output_dpi', 150)))

        # GIF
        gif_cfg = vis_cfg.get('gif', {})
        if bool(gif_cfg.get('enable', True)):
            make_gif(
                env,
                map_data,
                visited_order=result.visited_order,
                final_path=result.path_cells,
                towers={'turning': towers.turning_towers, 'straight': towers.straight_towers},
                out_gif_path=str(out_dir / 'search.gif'),
                cfg=gif_cfg,
            )

    else:
        with open(out_dir / 'report.json', 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=2)

        # Still save heatmap for debugging
        fig, _ = plot_static(env, map_data, show_heatmap=True, title='No path found (heatmap)')
        save_figure(fig, str(out_dir / 'heatmap.png'))

    print(f"[demo] Output directory: {out_dir}")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
