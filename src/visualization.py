"""Visualization utilities: final map + planning process GIF."""

from __future__ import annotations

import io
import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

import imageio.v2 as imageio


Cell = Tuple[int, int]


def _rect_corners(center: Tuple[float, float], length_m: float, width_m: float, angle_deg: float) -> np.ndarray:
    cx, cy = center
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    hx, hy = length_m / 2.0, width_m / 2.0
    # local corners (x along length)
    pts = np.array([
        [hx, hy],
        [hx, -hy],
        [-hx, -hy],
        [-hx, hy],
    ], dtype=float)
    R = np.array([[c, -s], [s, c]], dtype=float)
    pts = pts @ R.T
    pts[:, 0] += cx
    pts[:, 1] += cy
    return pts


def plot_static(
    env,
    map_data: Dict[str, Any],
    path: Optional[List[Cell]] = None,
    visited: Optional[List[Cell]] = None,
    towers: Optional[Dict[str, List[Cell]]] = None,
    show_heatmap: bool = True,
    heatmap_clip_percentile: float = 95.0,
    title: str = "Transmission Line Routing Demo",
) -> Tuple[plt.Figure, plt.Axes]:
    fig, ax = plt.subplots(figsize=(10, 6))

    # Heatmap
    if show_heatmap and env.costmap is not None:
        cm = env.costmap.copy()
        # Show penalties only (clip negatives so rewards do not confuse the "penalty" heatmap)
        cm = np.maximum(cm, 0.0)
        vmax = np.percentile(cm, heatmap_clip_percentile) if cm.size else 1.0
        ax.imshow(
            cm,
            origin='lower',
            extent=(0, env.width_m, 0, env.height_m),
            interpolation='nearest',
            alpha=0.55,
            vmin=0.0,
            vmax=max(vmax, 1e-6),
        )

    # Hard obstacles overlay
    if env.hard_mask is not None and np.any(env.hard_mask):
        # Plot blocked cells as semi-transparent black pixels
        mask = env.hard_mask.astype(float)
        ax.imshow(
            mask,
            origin='lower',
            extent=(0, env.width_m, 0, env.height_m),
            interpolation='nearest',
            alpha=0.25,
        )

    # Draw strips (rectangles)
    for s in map_data.get('strips', []):
        pts = _rect_corners(tuple(s['center']), float(s['length_m']), float(s['width_m']), float(s['angle_deg']))
        poly = Polygon(pts, closed=True, facecolor='0.6', edgecolor='0.25', alpha=0.8)
        ax.add_patch(poly)

    # Draw areas (circles)
    for a in map_data.get('areas', []):
        c = Circle(tuple(a['center']), float(a['radius_m']), facecolor='0.7', edgecolor='0.3', alpha=0.5)
        ax.add_patch(c)

    # Sensitive points
    for p in map_data.get('sensitive_points', []):
        c = Circle(tuple(p['center']), float(p['radius_hard_m']), facecolor='0.35', edgecolor='0.1', alpha=0.9)
        ax.add_patch(c)
        c2 = Circle(tuple(p['center']), float(p['radius_soft_m']), facecolor='none', edgecolor='0.2', linestyle='--', alpha=0.8)
        ax.add_patch(c2)

    # Corridor lines
    corridor = map_data.get('corridor', {})
    if corridor:
        primary = corridor.get('primary')
        if primary:
            pts = np.array(primary['points'], dtype=float)
            # Orange dashed for primary existing corridor
            ax.plot(pts[:, 0], pts[:, 1], linestyle='--', linewidth=2.0, color='orange', alpha=0.9)
        for other in corridor.get('others', []):
            pts = np.array(other['points'], dtype=float)
            ax.plot(pts[:, 0], pts[:, 1], linestyle=':', linewidth=1.5, alpha=0.7, color='orange')

    # Roads
    for r in map_data.get('roads', []):
        pts = np.array(r['points'], dtype=float)
        ax.plot(pts[:, 0], pts[:, 1], linestyle='-.', linewidth=1.5, alpha=0.8, color='tab:blue')

    # Visited nodes scatter
    if visited:
        # Subsample for performance
        step = max(1, len(visited) // 5000)
        v = visited[::step]
        xs = [(c[0] + 0.5) * env.resolution_m for c in v]
        ys = [(c[1] + 0.5) * env.resolution_m for c in v]
        ax.scatter(xs, ys, s=3, alpha=0.25, color='0.2')

    # Final path
    if path:
        xs = [(c[0] + 0.5) * env.resolution_m for c in path]
        ys = [(c[1] + 0.5) * env.resolution_m for c in path]
        ax.plot(xs, ys, linewidth=2.8, color='green')

    # Towers
    if towers:
        if towers.get('turning'):
            xs = [(c[0] + 0.5) * env.resolution_m for c in towers['turning']]
            ys = [(c[1] + 0.5) * env.resolution_m for c in towers['turning']]
            ax.scatter(xs, ys, marker='^', s=50)
        if towers.get('straight'):
            xs = [(c[0] + 0.5) * env.resolution_m for c in towers['straight']]
            ys = [(c[1] + 0.5) * env.resolution_m for c in towers['straight']]
            ax.scatter(xs, ys, marker='s', s=35)

    # Start/goal
    st = map_data.get('start')
    gl = map_data.get('goal')
    if st:
        ax.scatter([st[0]], [st[1]], s=90, color='red')
    if gl:
        ax.scatter([gl[0]], [gl[1]], s=90, color='blue')

    ax.set_xlim(0, env.width_m)
    ax.set_ylim(0, env.height_m)
    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(False)
    fig.tight_layout()
    return fig, ax


def save_figure(fig: plt.Figure, path: str, dpi: int = 150) -> None:
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


def make_gif(
    env,
    map_data: Dict[str, Any],
    visited_order: List[Cell],
    final_path: List[Cell],
    towers: Optional[Dict[str, List[Cell]]],
    out_gif_path: str,
    cfg: Dict[str, Any],
) -> None:
    """Generate a GIF.

    Performance note: Creating a new Matplotlib figure per frame can be very slow.
    Here we render the static background once, then update a scatter/line artist
    for each frame.
    """

    frame_every = int(cfg.get('frame_every_expansions', 50))
    max_frames = int(cfg.get('max_frames', 120))
    heatmap_clip = float(cfg.get('heatmap_clip_percentile', 95.0))
    dpi = int(cfg.get('dpi', 120))

    frames: List[np.ndarray] = []

    # Frame indices
    if visited_order:
        idxs = list(range(0, len(visited_order), frame_every))
        if idxs and idxs[-1] != len(visited_order) - 1:
            idxs.append(len(visited_order) - 1)
    else:
        idxs = [0]

    if len(idxs) > max_frames:
        keep = np.linspace(0, len(idxs) - 1, max_frames).round().astype(int)
        idxs = [idxs[i] for i in keep]

    # Background figure
    fig, ax = plot_static(
        env,
        map_data,
        path=None,
        visited=None,
        towers=None,
        show_heatmap=bool(cfg.get('show_heatmap', True)),
        heatmap_clip_percentile=heatmap_clip,
        title=str(cfg.get('title', 'A* exploration')),
    )

    visited_scatter = ax.scatter([], [], s=3, alpha=0.25, color='0.2')
    path_line, = ax.plot([], [], linewidth=2.8, color='green')

    turning_scatter = None
    straight_scatter = None
    if towers:
        turning_scatter = ax.scatter([], [], marker='^', s=50, color='k')
        straight_scatter = ax.scatter([], [], marker='s', s=35, color='k')

    for k, i in enumerate(idxs):
        if visited_order:
            visited = visited_order[: i + 1]
            step = max(1, len(visited) // 5000)
            v = visited[::step]
            xs = [(c[0] + 0.5) * env.resolution_m for c in v]
            ys = [(c[1] + 0.5) * env.resolution_m for c in v]
            visited_scatter.set_offsets(np.c_[xs, ys] if xs else np.zeros((0, 2)))

        if k == len(idxs) - 1:
            xs_p = [(c[0] + 0.5) * env.resolution_m for c in final_path]
            ys_p = [(c[1] + 0.5) * env.resolution_m for c in final_path]
            path_line.set_data(xs_p, ys_p)

            if towers and turning_scatter is not None and straight_scatter is not None:
                tx = [(c[0] + 0.5) * env.resolution_m for c in towers.get('turning', [])]
                ty = [(c[1] + 0.5) * env.resolution_m for c in towers.get('turning', [])]
                turning_scatter.set_offsets(np.c_[tx, ty] if tx else np.zeros((0, 2)))

                sx = [(c[0] + 0.5) * env.resolution_m for c in towers.get('straight', [])]
                sy = [(c[1] + 0.5) * env.resolution_m for c in towers.get('straight', [])]
                straight_scatter.set_offsets(np.c_[sx, sy] if sx else np.zeros((0, 2)))

        fig.set_dpi(dpi)
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(h, w, 3)
        frames.append(img)

    plt.close(fig)
    imageio.mimsave(out_gif_path, frames, duration=float(cfg.get('duration_s', 0.15)))
