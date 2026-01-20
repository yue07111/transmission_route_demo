"""Build Environment + ConstraintManager from YAML map and config."""

from __future__ import annotations

from typing import Any, Dict, List, Tuple

from .environment import Environment
from .constraints.base import ConstraintManager
from .constraints.areas import CircleArea, AreaDampingConstraint
from .constraints.strips import StripRect, StripCrossingConstraint
from .constraints.landable import LandablePointsConstraint
from .constraints.turns import TurnAngleConstraint
from .constraints.corridor import CorridorLine, CorridorReuseConstraint
from .constraints.roads import RoadLine, RoadRewardConstraint
from .constraints.sensitive import SensitivePoint, SensitiveFacilityConstraint


def build_from_spec(config: Dict[str, Any], map_data: Dict[str, Any]) -> Tuple[Environment, ConstraintManager]:
    env_cfg = map_data['env']
    env = Environment(
        width_m=float(env_cfg['width_m']),
        height_m=float(env_cfg['height_m']),
        resolution_m=float(env_cfg['resolution_m']),
        entities=map_data,
    )

    c_cfg = config.get('constraints', {})

    constraints = []

    # Areas
    areas = []
    for a in map_data.get('areas', []):
        areas.append(CircleArea(
            id=str(a['id']),
            center=tuple(a['center']),
            radius_m=float(a['radius_m']),
            is_enterable=bool(a.get('is_enterable', True)),
            is_crossable=bool(a.get('is_crossable', True)),
            damping=float(a.get('damping', 0.0)),
            entry_penalty=float(a.get('entry_penalty', 0.0)),
            cross_cost=float(a.get('cross_cost', 0.0)),
            note=str(a.get('note', '')),
        ))
    if areas:
        constraints.append(AreaDampingConstraint(areas, c_cfg.get('area_damping', {})))

    # Strips
    strips = []
    for s in map_data.get('strips', []):
        strips.append(StripRect(
            id=str(s['id']),
            center=tuple(s['center']),
            length_m=float(s['length_m']),
            width_m=float(s['width_m']),
            angle_deg=float(s['angle_deg']),
            is_crossable=bool(s.get('is_crossable', True)),
            min_cross_angle_deg=float(s.get('min_cross_angle_deg', 30.0)),
            cross_cost=float(s.get('cross_cost', 0.0)),
            note=str(s.get('note', '')),
        ))
    if strips:
        constraints.append(StripCrossingConstraint(strips, c_cfg.get('strip_crossing', {})))

    # Sensitive facilities
    sens = []
    for p in map_data.get('sensitive_points', []):
        sens.append(SensitivePoint(
            id=str(p['id']),
            center=tuple(p['center']),
            radius_hard_m=float(p['radius_hard_m']),
            radius_soft_m=float(p['radius_soft_m']),
            soft_penalty=float(p.get('soft_penalty', 0.0)),
            note=str(p.get('note', '')),
        ))
    if sens:
        constraints.append(SensitiveFacilityConstraint(sens, c_cfg.get('sensitive_facility', {})))

    # Corridor reuse
    corridor = map_data.get('corridor')
    if corridor and corridor.get('primary'):
        primary = CorridorLine(id=str(corridor['primary']['id']), points=[tuple(p) for p in corridor['primary']['points']])
        others = []
        for o in corridor.get('others', []):
            others.append(CorridorLine(id=str(o['id']), points=[tuple(p) for p in o['points']]))
        constraints.append(CorridorReuseConstraint(primary, others, c_cfg.get('corridor_reuse', {})))

    # Road rewards
    roads = []
    for r in map_data.get('roads', []):
        roads.append(RoadLine(id=str(r['id']), points=[tuple(p) for p in r['points']]))
    if roads:
        constraints.append(RoadRewardConstraint(roads, c_cfg.get('road_reward', {})))

    # Landable points (depends on hard mask, so keep late)
    constraints.append(LandablePointsConstraint(c_cfg.get('landable_points', {})))

    # Turn angle constraint (dynamic)
    constraints.append(TurnAngleConstraint(c_cfg.get('turn_angle', {})))

    manager = ConstraintManager(constraints)

    # Rasterize static layers
    costmap, hard, landable = manager.rasterize_all(env)
    env.costmap = costmap
    env.hard_mask = hard
    env.landable = landable

    return env, manager
