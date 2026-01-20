"""Geometry helpers (pure numpy/math, no shapely dependency).

We keep this file intentionally dependency-light so the demo can run in
most environments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Point = Tuple[float, float]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def rotate_point(p: Point, angle_rad: float) -> Point:
    """Rotate a point around origin by angle_rad."""
    x, y = p
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return (c * x - s * y, s * x + c * y)


def to_local(p: Point, center: Point, angle_rad: float) -> Point:
    """Transform global point p into rectangle-local coordinates.

    The local frame is centered at `center` and rotated by angle_rad.
    We apply translation then rotate by -angle_rad.
    """
    x, y = p
    cx, cy = center
    x -= cx
    y -= cy
    return rotate_point((x, y), -angle_rad)


def segment_intersects_aabb(p0: Point, p1: Point, xmin: float, xmax: float, ymin: float, ymax: float) -> bool:
    """Liang-Barsky line clipping test for segment vs axis-aligned rectangle."""
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0

    p = [-dx, dx, -dy, dy]
    q = [x0 - xmin, xmax - x0, y0 - ymin, ymax - y0]

    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return False
        else:
            t = qi / pi
            if pi < 0:
                if t > u2:
                    return False
                u1 = max(u1, t)
            else:
                if t < u1:
                    return False
                u2 = min(u2, t)
    return True


def segment_intersects_oriented_rect(p0: Point, p1: Point, center: Point, length: float, width: float, angle_rad: float) -> bool:
    """Check whether segment intersects an oriented rectangle.

    Rectangle is defined by center, length (along local x), width (local y), angle.
    """
    # Transform endpoints into rectangle local coordinates where rect becomes axis-aligned.
    lp0 = to_local(p0, center, angle_rad)
    lp1 = to_local(p1, center, angle_rad)
    hx = length / 2.0
    hy = width / 2.0
    return segment_intersects_aabb(lp0, lp1, -hx, hx, -hy, hy)


def angle_between_vectors_deg(v1: Point, v2: Point) -> float:
    """Return the smaller angle between vectors in degrees, in [0, 180]."""
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    dot = (x1 * x2 + y1 * y2) / (n1 * n2)
    dot = clamp(dot, -1.0, 1.0)
    ang = math.degrees(math.acos(dot))
    # Map to [0, 180]
    return ang


def smallest_undirected_angle_deg(v1: Point, v2: Point) -> float:
    """Angle in [0, 90] treating direction as undirected.

    For crossing constraints we often care about the absolute angle (parallel vs perpendicular),
    regardless of sign.
    """
    ang = angle_between_vectors_deg(v1, v2)
    # undirected: map to [0, 90]
    if ang > 90.0:
        ang = 180.0 - ang
    return ang


@dataclass(frozen=True)
class Polyline:
    points: List[Point]

    def segments(self) -> List[Tuple[Point, Point]]:
        return list(zip(self.points[:-1], self.points[1:]))


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Euclidean distance from point p to segment ab."""
    px, py = p
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = (wx * vx + wy * vy) / vv
    t = clamp(t, 0.0, 1.0)
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def point_polyline_distance(p: Point, pts: Sequence[Point]) -> float:
    """Distance from point to polyline."""
    if len(pts) < 2:
        return float('inf')
    best = float('inf')
    for a, b in zip(pts[:-1], pts[1:]):
        best = min(best, point_segment_distance(p, a, b))
    return best


def polyline_tangent_at_nearest(p: Point, pts: Sequence[Point]) -> Point:
    """Approximate tangent direction of a polyline at the nearest segment to point p."""
    if len(pts) < 2:
        return (1.0, 0.0)
    best = float('inf')
    best_seg = (pts[0], pts[1])
    for a, b in zip(pts[:-1], pts[1:]):
        d = point_segment_distance(p, a, b)
        if d < best:
            best = d
            best_seg = (a, b)
    ax, ay = best_seg[0]
    bx, by = best_seg[1]
    vx, vy = bx - ax, by - ay
    n = math.hypot(vx, vy)
    if n < 1e-12:
        return (1.0, 0.0)
    return (vx / n, vy / n)
