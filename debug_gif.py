import yaml

from src.builder import build_from_spec
from src.astar import AStarPlanner
from src.postprocess import place_towers
from src.visualization import make_gif

cfg = yaml.safe_load(open('configs/demo_config.yaml', 'r', encoding='utf-8'))
map_data = yaml.safe_load(open('configs/demo_map.yaml', 'r', encoding='utf-8'))

env, cm = build_from_spec(cfg, map_data)
start_cell = env.world_to_grid(tuple(map_data['start']))
goal_cell = env.world_to_grid(tuple(map_data['goal']))

planner = AStarPlanner(cfg.get('planner', {}))
res = planner.plan(env, cm, start_cell, goal_cell)
print('success:', res.success, 'visited:', len(res.visited_order), 'path_len:', len(res.path_cells))

if res.success:
    t = place_towers(env, res.path_cells, cfg.get('postprocess', {}))
    out_path = 'outputs/debug.gif'
    make_gif(
        env,
        map_data,
        visited_order=res.visited_order,
        final_path=res.path_cells,
        towers={'turning': t.turning_towers, 'straight': t.straight_towers},
        out_gif_path=out_path,
        cfg=cfg['visualization']['gif'],
    )
    print('saved:', out_path)
