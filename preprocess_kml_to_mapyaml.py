#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Preprocess KML to YAML (v4.10.5)

新增：
1) 支持读取“起点和终点.kml”，并写入 YAML（同时保留 xy + lonlat）
2) 若起点/终点也在地物 KML 内（同一个 KML），也可识别
3) 修复关键问题：全量数据统一投影到同一 local UTM 坐标系（避免每个要素各用各的UTM）

输出：
- yaml/map_generated.yaml：现算法字段 + raw_geom保真几何 + start/goal lonlat+xy
- raw/features_raw.jsonl：全量可追溯
- report/classification_report.json：统计
- review/unknown_features.geojson：unknown审阅
- review/preview.kml：简易预览
- review/derived_airport_clearance_polygons_lonlat.kml：机场净空线转面结果（奥维可直接查看）
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from lxml import etree
from shapely.geometry import Point, LineString, Polygon, MultiPolygon, MultiLineString, GeometryCollection, mapping
from shapely.ops import unary_union, linemerge, polygonize, transform, snap
from shapely import wkt as _wkt
from pyproj import CRS, Transformer

KML_NS = {"kml": "http://www.opengis.net/kml/2.2", "gx": "http://www.google.com/kml/ext/2.2"}


# ----------------------------
# Robust geometry helpers (invalid geometries in real KML may crash GEOS unary_union)
# ----------------------------
def _make_valid_fallback(g):
    """
    Try to repair invalid geometries.
    Works with Shapely 1.8+ / 2.x:
    - Prefer shapely.make_valid if available (2.x)
    - Fallback to buffer(0) for polygons (classic fix for self-intersections)
    - As a last resort return original geometry
    """
    if g is None:
        return None
    try:
        if getattr(g, "is_valid", True):
            return g
    except Exception:
        # if g doesn't have is_valid
        return g

    # Shapely 2.x has make_valid
    try:
        from shapely.validation import make_valid  # type: ignore
        gg = make_valid(g)
        if gg is not None and (not gg.is_empty):
            return gg
    except Exception:
        pass

    # buffer(0) often fixes self-intersections for polygons
    try:
        if g.geom_type in ("Polygon", "MultiPolygon"):
            gg = g.buffer(0)
            if gg is not None and (not gg.is_empty):
                return gg
    except Exception:
        pass

    return g

def safe_centroid_lonlat(geoms):
    """
    Compute a robust centroid for a list of lon/lat geometries without unary_union.
    This avoids GEOS TopologyException on invalid polygons.
    Strategy: repair each geom if needed, take centroid of each, average lon/lat.
    """
    xs, ys = [], []
    for g in geoms or []:
        gg = _make_valid_fallback(g)
        if gg is None or getattr(gg, "is_empty", False):
            continue
        try:
            c = gg.centroid
            xs.append(float(c.x))
            ys.append(float(c.y))
        except Exception:
            continue
    if not xs:
        return Point(0.0, 0.0)
    return Point(sum(xs) / len(xs), sum(ys) / len(ys))

def safe_union_bounds_xy(geoms_xy):
    """
    Compute bounds for projected geometries robustly.
    Prefer unary_union for exact bounds; on failure fallback to per-geom bounds aggregation.
    """
    fixed = []
    for g in geoms_xy or []:
        gg = _make_valid_fallback(g)
        if gg is None or getattr(gg, "is_empty", False):
            continue
        fixed.append(gg)

    if not fixed:
        return (0.0, 0.0, 1.0, 1.0)

    try:
        u = unary_union(fixed)
        return u.bounds
    except Exception:
        minx = min(g.bounds[0] for g in fixed)
        miny = min(g.bounds[1] for g in fixed)
        maxx = max(g.bounds[2] for g in fixed)
        maxy = max(g.bounds[3] for g in fixed)
        return (float(minx), float(miny), float(maxx), float(maxy))

def now_ts() -> str:
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")

def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)

def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default

def norm_text(s: Any) -> str:
    if s is None:
        return ""
    s = str(s).strip()
    s = re.sub(r"[\s\u3000]+", " ", s)
    roman_map = {
        "Ⅰ": "I", "Ⅱ": "II", "Ⅲ": "III", "Ⅳ": "IV", "Ⅴ": "V",
        "Ⅵ": "VI", "Ⅶ": "VII", "Ⅷ": "VIII", "Ⅸ": "IX", "Ⅹ": "X",
        "ⅰ": "i", "ⅱ": "ii", "ⅲ": "iii", "ⅳ": "iv", "ⅴ": "v",
        "ⅵ": "vi", "ⅶ": "vii", "ⅷ": "viii", "ⅸ": "ix", "ⅹ": "x",
    }
    for k, v in roman_map.items():
        s = s.replace(k, v)
    s = s.replace("——", "-").replace("—", "-")
    return s

def _findtext(node, xpath: str) -> str:
    el = node.find(xpath, namespaces=KML_NS)
    return "" if el is None or el.text is None else el.text

def parse_coords(text: str) -> List[Tuple[float, float, float]]:
    """
    Parse KML <coordinates> content into list of (lon, lat, alt).
    Primary: "lon,lat,alt lon,lat,alt ..." separated by whitespace/newlines.
    Fallback: handle space-separated tuples like "lon lat alt" (non-standard exports).
    """
    coords: List[Tuple[float, float, float]] = []
    raw = (text or "").strip()
    if not raw:
        return coords

    # Standard comma-separated tuples
    for chunk in re.split(r"\s+", raw):
        if not chunk:
            continue
        if "," in chunk:
            parts = chunk.split(",")
            if len(parts) >= 2:
                lon = safe_float(parts[0])
                lat = safe_float(parts[1])
                alt = safe_float(parts[2], 0.0) if len(parts) >= 3 else 0.0
                if -180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0:
                    coords.append((lon, lat, alt))

    if len(coords) >= 2:
        return coords

    # Fallback: extract floats and group by 3 (preferred) or 2
    nums = [float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", raw)]
    if len(nums) >= 4:
        grouped: List[Tuple[float, float, float]] = []
        if len(nums) % 3 == 0:
            for i in range(0, len(nums), 3):
                lon, lat, alt = nums[i], nums[i+1], nums[i+2]
                if -180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0:
                    grouped.append((lon, lat, alt))
        elif len(nums) % 2 == 0:
            for i in range(0, len(nums), 2):
                lon, lat = nums[i], nums[i+1]
                if -180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0:
                    grouped.append((lon, lat, 0.0))
        if len(grouped) >= 2:
            return grouped

    return coords

def parse_gx_track_coords(pm: etree._Element) -> List[Tuple[float, float, float]]:
    """
    Parse gx:Track / gx:MultiTrack coordinates.
    Some KML exports store tracks as gx:coord entries: "lon lat alt".
    """
    coords: List[Tuple[float, float, float]] = []
    for coord in pm.findall(".//gx:Track/gx:coord", namespaces=KML_NS):
        txt = (coord.text or "").strip()
        if not txt:
            continue
        parts = re.split(r"\s+", txt)
        if len(parts) >= 2:
            lon = safe_float(parts[0])
            lat = safe_float(parts[1])
            alt = safe_float(parts[2], 0.0) if len(parts) >= 3 else 0.0
            coords.append((lon, lat, alt))
    for coord in pm.findall(".//gx:MultiTrack//gx:coord", namespaces=KML_NS):
        txt = (coord.text or "").strip()
        if not txt:
            continue
        parts = re.split(r"\s+", txt)
        if len(parts) >= 2:
            lon = safe_float(parts[0])
            lat = safe_float(parts[1])
            alt = safe_float(parts[2], 0.0) if len(parts) >= 3 else 0.0
            coords.append((lon, lat, alt))
    return coords

def extract_inline_line_width(pm: etree._Element) -> Optional[float]:
    """
    Extract display line width from inline <Style><LineStyle><width> (pixel width in KML viewers).
    Only used for progress visualization KML output, NOT for algorithm width_m (meters).
    """
    w = pm.find(".//kml:Style/kml:LineStyle/kml:width", namespaces=KML_NS)
    if w is not None and (w.text or "").strip():
        return safe_float(w.text)
    w2 = pm.find(".//kml:OvStyle//kml:TrackStyle/kml:width", namespaces=KML_NS)
    if w2 is not None and (w2.text or "").strip():
        return safe_float(w2.text)
    return None

def any_match(patterns: List[str], text: str) -> Optional[str]:
    for p in patterns or []:
        if not p:
            continue
        try:
            if re.search(p, text, flags=re.IGNORECASE):
                return p
        except re.error:
            continue
    return None

def read_yaml(path: Optional[Path]) -> dict:
    if path is None or (not path.exists()):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

def geom_from_placemark(pm: etree._Element):
    """Parse *all* geometries inside a Placemark.

    OvKML/KML from OSM/AutoCAD/DXF exports often stores a logical feature as a MultiGeometry
    (many LineString/Polygon parts). Older logic used `find()` and only kept the first part,
    which caused 'missing' segments in outputs (KML/YAML).
    """
    # gx:Track / gx:MultiTrack (treat as a single LineString)
    track_coords = parse_gx_track_coords(pm)
    if len(track_coords) >= 2:
        return LineString([(x, y) for x, y, _ in track_coords])

    geoms = []

    # Points (can be multiple)
    for pt in pm.findall(".//kml:Point/kml:coordinates", namespaces=KML_NS):
        if pt is not None and (pt.text or "").strip():
            c = parse_coords(pt.text)
            if c:
                geoms.append(Point(c[0][0], c[0][1]))

    # LineStrings (can be multiple; and some exports embed multiple <coordinates> per LineString)
    for ls_el in pm.findall(".//kml:LineString", namespaces=KML_NS):
        coord_els = ls_el.findall(".//kml:coordinates", namespaces=KML_NS)
        for coord_el in coord_els:
            if coord_el is None or not (coord_el.text or "").strip():
                continue
            c = parse_coords(coord_el.text)
            if len(c) >= 2:
                try:
                    geoms.append(LineString([(x, y) for x, y, _ in c]))
                except Exception:
                    # skip invalid segment, but keep others
                    continue

    # Polygons (can be multiple)
    for poly_el in pm.findall(".//kml:Polygon", namespaces=KML_NS):
        outer_el = poly_el.find(".//kml:outerBoundaryIs/kml:LinearRing/kml:coordinates", namespaces=KML_NS)
        if outer_el is None or not (outer_el.text or "").strip():
            continue
        oc = parse_coords(outer_el.text)
        if len(oc) < 3:
            continue
        holes = []
        for inner_el in poly_el.findall(".//kml:innerBoundaryIs/kml:LinearRing/kml:coordinates", namespaces=KML_NS):
            if inner_el is None or not (inner_el.text or "").strip():
                continue
            ic = parse_coords(inner_el.text)
            if len(ic) >= 3:
                holes.append([(x, y) for x, y, _ in ic])
        try:
            geoms.append(Polygon([(x, y) for x, y, _ in oc], holes=holes))
        except Exception:
            continue

    # Standalone LinearRing not wrapped by Polygon (rare, but exists)
    # NOTE: Use XPath to avoid double-counting rings that belong to polygons.
    try:
        ring_coord_els = pm.xpath(".//kml:LinearRing[not(ancestor::kml:Polygon)]/kml:coordinates", namespaces=KML_NS)
    except Exception:
        ring_coord_els = []
    for ring_el in ring_coord_els:
        if ring_el is None or not (ring_el.text or "").strip():
            continue
        rc = parse_coords(ring_el.text)
        if len(rc) >= 3:
            try:
                geoms.append(Polygon([(x, y) for x, y, _ in rc]))
            except Exception:
                continue

    if not geoms:
        return None
    if len(geoms) == 1:
        return geoms[0]

    # Normalize common multi-part cases
    if all(isinstance(g, LineString) for g in geoms):
        return MultiLineString(geoms)
    if all(isinstance(g, Polygon) for g in geoms):
        # keep parts; do not unary_union here (union may dissolve boundaries)
        return MultiPolygon(geoms)

    # Mixed types: keep all parts
    return GeometryCollection(geoms)



def walk_folders(root: etree._Element):
    def rec(node, path: List[str]):
        fname = norm_text(_findtext(node, "kml:name"))
        if fname:
            path = path + [fname]
        for pm in node.findall("kml:Placemark", namespaces=KML_NS):
            yield (path, pm)
        for child in node.findall("kml:Folder", namespaces=KML_NS):
            yield from rec(child, path)

    docs = root.findall(".//kml:Document", namespaces=KML_NS)
    if docs:
        for d in docs:
            yield from rec(d, [])
    else:
        for f in root.findall(".//kml:Folder", namespaces=KML_NS):
            yield from rec(f, [])

def choose_utm_crs(lon: float, lat: float) -> CRS:
    zone = int(math.floor((lon + 180.0) / 6.0) + 1)
    epsg = (32700 + zone) if lat < 0 else (32600 + zone)
    return CRS.from_epsg(epsg)

def build_transformer(lon: float, lat: float) -> Transformer:
    return Transformer.from_crs(CRS.from_epsg(4326), choose_utm_crs(lon, lat), always_xy=True)

def build_inverse_transformer(lon: float, lat: float) -> Transformer:
    # Build inverse transformer from local UTM (meters) back to WGS84 lon/lat.
    return Transformer.from_crs(choose_utm_crs(lon, lat), CRS.from_epsg(4326), always_xy=True)

def geom_xy_to_lonlat(g, inv_tf: Transformer):
    # Reproject shapely geometry from xy (UTM meters) to lon/lat (WGS84).
    if g is None:
        return None
    try:
        return transform(lambda x, y, z=None: inv_tf.transform(x, y), g)
    except Exception:
        return g

def write_derived_polygons_kml_lonlat(path: Path, kept_polys_xy, inv_tf: Transformer, name_prefix: str = "派生面"):
    """
    Export derived polygons (built in xy space) as lon/lat KML for Ovi/Google Earth review.
    kept_polys_xy: List[(Polygon/MultiPolygon, used_lines)]
    """
    if not kept_polys_xy:
        return
    kml = etree.Element("{http://www.opengis.net/kml/2.2}kml")
    doc = etree.SubElement(kml, "Document")
    etree.SubElement(doc, "name").text = "derived_polygons_lonlat"

    # simple style (semi-transparent fill)
    st = etree.SubElement(doc, "Style", id="derived_poly")
    ps = etree.SubElement(st, "PolyStyle")
    etree.SubElement(ps, "color").text = "7d00ffff"  # aabbggrr
    etree.SubElement(ps, "outline").text = "1"
    ls = etree.SubElement(st, "LineStyle")
    etree.SubElement(ls, "color").text = "ff00ffff"
    etree.SubElement(ls, "width").text = "2"

    folder = etree.SubElement(doc, "Folder")
    etree.SubElement(folder, "name").text = name_prefix

    def add_poly(pm_parent, geom_ll):
        if geom_ll is None or getattr(geom_ll, "is_empty", False):
            return
        if geom_ll.geom_type == "Polygon":
            poly_el = etree.SubElement(pm_parent, "Polygon")
            etree.SubElement(poly_el, "extrude").text = "0"
            etree.SubElement(poly_el, "altitudeMode").text = "clampToGround"
            ob = etree.SubElement(poly_el, "outerBoundaryIs")
            lr = etree.SubElement(ob, "LinearRing")
            coords_el = etree.SubElement(lr, "coordinates")
            coords_el.text = " ".join([f"{x},{y},0" for x, y in list(geom_ll.exterior.coords)])
            for hole in geom_ll.interiors:
                ib = etree.SubElement(poly_el, "innerBoundaryIs")
                lr2 = etree.SubElement(ib, "LinearRing")
                ce2 = etree.SubElement(lr2, "coordinates")
                ce2.text = " ".join([f"{x},{y},0" for x, y in list(hole.coords)])
        elif geom_ll.geom_type == "MultiPolygon":
            mg = etree.SubElement(pm_parent, "MultiGeometry")
            for g2 in geom_ll.geoms:
                add_poly(mg, g2)

    for i, (poly_xy, used) in enumerate(kept_polys_xy):
        poly_ll = geom_xy_to_lonlat(poly_xy, inv_tf)
        pm = etree.SubElement(folder, "Placemark")
        etree.SubElement(pm, "name").text = f"{name_prefix}_{i}"
        etree.SubElement(pm, "styleUrl").text = "#derived_poly"
        desc = f"derived_from: line_to_polygon\nidx: {i}\nused_lines: {used}\narea_xy: {float(getattr(poly_xy,'area',0.0))}"
        etree.SubElement(pm, "description").text = desc
        add_poly(pm, poly_ll)

    xml = etree.tostring(kml, xml_declaration=True, encoding="utf-8", pretty_print=True)
    path.write_bytes(xml)

def project_point_lonlat(lon: float, lat: float, tf: Transformer) -> Tuple[float, float]:
    x, y = tf.transform(lon, lat)
    return float(x), float(y)

def project_geom(g, tf: Transformer):
    if g is None:
        return None
    if isinstance(g, Point):
        x, y = project_point_lonlat(g.x, g.y, tf)
        return Point(x, y)
    if isinstance(g, LineString):
        xs, ys = tf.transform([c[0] for c in g.coords], [c[1] for c in g.coords])
        return LineString(list(zip(xs, ys)))
    if isinstance(g, Polygon):
        xs, ys = tf.transform([c[0] for c in g.exterior.coords], [c[1] for c in g.exterior.coords])
        shell = list(zip(xs, ys))
        holes = []
        for ring in g.interiors:
            xs, ys = tf.transform([c[0] for c in ring.coords], [c[1] for c in ring.coords])
            holes.append(list(zip(xs, ys)))
        return Polygon(shell, holes)
    if isinstance(g, MultiPolygon):
        return MultiPolygon([project_geom(p, tf) for p in g.geoms])
    if isinstance(g, MultiLineString):
        return MultiLineString([project_geom(l, tf) for l in g.geoms])
    if isinstance(g, GeometryCollection):
        return GeometryCollection([project_geom(gg, tf) for gg in g.geoms])
    return None

@dataclass
class FeatureLonLat:
    fid: str
    name: str
    folder_path: List[str]
    folder_path_str: str
    style_url: str
    description: str
    extdata: Dict[str, Any]
    geom_type: str
    geom_lonlat_wkt: str
    geom_lonlat: object  # shapely geom

@dataclass
class OverrideRule:
    match_type: str
    pattern: str
    feature_type: str
    constraint_role: str
    params_json: str
    priority: int

def load_overrides(path: Optional[Path]) -> List[OverrideRule]:
    if path is None or not path.exists():
        return []
    rules = []
    with open(path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            mt = (row.get("match_type") or "").strip()
            if not mt or mt.startswith("#"):
                continue
            rules.append(OverrideRule(
                match_type=mt,
                pattern=(row.get("pattern") or "").strip(),
                feature_type=(row.get("feature_type") or "").strip(),
                constraint_role=(row.get("constraint_role") or "").strip(),
                params_json=(row.get("params_json") or "").strip(),
                priority=int(row.get("priority") or 0),
            ))
    rules.sort(key=lambda r: r.priority, reverse=True)
    return rules

class Classifier:
    def __init__(self, rules: dict, overrides: List[OverrideRule]):
        self.rules = rules or {}
        self.overrides = overrides or []
        self.synonyms = self.rules.get("synonyms", {})
        self.feature_rules = self.rules.get("feature_rules", [])
        self.annotation_rules = self.rules.get("annotation_rules", [])
        self.min_score = float(self.rules.get("min_score", 2.0))

    def apply_synonyms(self, s: str) -> str:
        t = s
        for k in sorted(self.synonyms.keys(), key=lambda x: len(str(x)), reverse=True):
            v = self.synonyms[k]
            if not k or not v:
                continue
            try:
                t = re.sub(k, v, t, flags=re.IGNORECASE)
            except re.error:
                t = re.sub(re.escape(k), v, t, flags=re.IGNORECASE)
        return t

    def override_match(self, fid: str, name: str, folder: str) -> Optional[dict]:
        for r in self.overrides:
            try:
                if r.match_type == "folder_regex" and re.search(r.pattern, folder):
                    return self._mk(r, "override(folder)")
                if r.match_type == "name_regex" and re.search(r.pattern, name):
                    return self._mk(r, "override(name)")
                if r.match_type == "id_regex" and re.search(r.pattern, fid):
                    return self._mk(r, "override(id)")
            except re.error:
                continue
        return None

    def _mk(self, r: OverrideRule, src: str) -> dict:
        params = {}
        if r.params_json:
            try:
                params = json.loads(r.params_json)
            except Exception:
                params = {}
        return {"feature_type": r.feature_type, "constraint_role": r.constraint_role,
                "confidence": "high", "reason": [src], "params": params}

    def classify_text(self, fid: str, name: str, folder: str, desc: str, geom_type: str) -> dict:
        ov = self.override_match(fid, name, folder)
        if ov:
            return ov

        folder2 = self.apply_synonyms(folder)
        name2 = self.apply_synonyms(name)
        desc2 = self.apply_synonyms(desc or "")
        text = f"{folder2} | {name2} | {desc2}"

        for ar in self.annotation_rules:
            pat = ar.get("pattern")
            if pat and re.search(pat, text, flags=re.IGNORECASE):
                return {"feature_type": "annotation", "constraint_role": "annotation",
                        "confidence": "high", "reason": [f"annotation:{pat}"], "params": {}}

        best = None
        best_score = -1e9
        best_reason = []

        def any_match_local(ps, t):
            for p in ps or []:
                if not p:
                    continue
                try:
                    if re.search(p, t, flags=re.IGNORECASE):
                        return p
                except re.error:
                    continue
            return None

        for r in self.feature_rules:
            score = 0.0
            hit = False
            rn = r.get("name", "rule")
            mname = any_match_local(r.get("name_patterns", []), name2)
            mfolder = any_match_local(r.get("folder_patterns", []), folder2)
            mdesc = any_match_local(r.get("desc_patterns", []), desc2)

            if mname:
                score += float(r.get("w_name", 3.0)); hit = True
            if mfolder:
                score += float(r.get("w_folder", 2.0)); hit = True
            if mdesc:
                score += float(r.get("w_desc", 1.5)); hit = True

            gtypes = r.get("geom_types")
            if gtypes:
                # Accept Multi* when rule expects base type
                base_gt = geom_type
                if geom_type == "MultiLineString":
                    base_gt = "LineString"
                elif geom_type == "MultiPolygon":
                    base_gt = "Polygon"
                elif geom_type == "MultiPoint":
                    base_gt = "Point"
                if (geom_type in gtypes) or (base_gt in gtypes):
                    score += float(r.get("w_geom", 0.5))
                else:
                    score -= float(r.get("penalty_geom_mismatch", 2.0))

            if hit and score > best_score:
                best_score = score
                best = r
                best_reason = [f"score={score:.2f}", rn]
                if mname: best_reason.append(f"name:{mname}")
                if mfolder: best_reason.append(f"folder:{mfolder}")
                if mdesc: best_reason.append(f"desc:{mdesc}")

        if best is None or best_score < self.min_score:
            return {"feature_type": "unknown", "constraint_role": "unknown",
                    "confidence": "low", "reason": ["no_rule_match"], "params": {}}

        return {"feature_type": best.get("feature_type", "unknown"),
                "constraint_role": best.get("constraint_role", "unknown"),
                "confidence": best.get("confidence", "medium"),
                "reason": best_reason,
                "params": best.get("params", {}) or {}}

def detect_start_goal_from_features(features: List[FeatureLonLat], rules: dict) -> Tuple[Optional[Tuple[float,float]], Optional[Tuple[float,float]], dict]:
    cfg = (rules.get("start_goal") or {})
    start_exact = cfg.get("start_exact_patterns", [r"^(起点|始点|start|source)$"])
    goal_exact  = cfg.get("goal_exact_patterns",  [r"^(终点|止点|end|goal|destination)$"])
    start_contains = cfg.get("start_contains_patterns", [r"起点", r"始点", r"\bstart\b", r"\bsource\b"])
    goal_contains  = cfg.get("goal_contains_patterns",  [r"终点", r"止点", r"\bend\b", r"\bgoal\b", r"\bdestination\b"])

    candidates = []
    for f in features:
        if f.geom_type != "Point":
            continue
        g: Point = f.geom_lonlat
        name = norm_text(f.name)
        folder = norm_text(f.folder_path_str)
        desc = norm_text(f.description)
        text = f"{folder} {name} {desc}"

        score_s = 0
        score_g = 0
        if any_match(start_exact, name):
            score_s += 100
        elif any_match(start_contains, text):
            score_s += 10

        if any_match(goal_exact, name):
            score_g += 100
        elif any_match(goal_contains, text):
            score_g += 10

        if score_s > 0 or score_g > 0:
            candidates.append((score_s, score_g, g.x, g.y, f.fid, f.name, f.folder_path_str))

    start_ll = None
    goal_ll = None
    meta = {"from_main_kml": []}

    if candidates:
        best_s = max(candidates, key=lambda t: t[0])
        if best_s[0] > 0:
            start_ll = (best_s[2], best_s[3])
            meta["from_main_kml"].append({"type": "start", "id": best_s[4], "name": best_s[5], "folder": best_s[6], "score": best_s[0]})

        best_g = max(candidates, key=lambda t: t[1])
        if best_g[1] > 0:
            goal_ll = (best_g[2], best_g[3])
            meta["from_main_kml"].append({"type": "goal", "id": best_g[4], "name": best_g[5], "folder": best_g[6], "score": best_g[1]})

    return start_ll, goal_ll, meta

def read_start_goal_kml(path: Path, rules: dict) -> Tuple[Optional[Tuple[float,float]], Optional[Tuple[float,float]], dict]:
    if not path.exists():
        return None, None, {"from_start_goal_kml": "not_found"}

    parser = etree.XMLParser(recover=True, huge_tree=True)
    tree = etree.parse(str(path), parser)
    root = tree.getroot()

    feats = []
    for folder_path, pm in walk_folders(root):
        name = norm_text(_findtext(pm, "kml:name")) or "unnamed"
        desc = norm_text(_findtext(pm, "kml:description"))
        style = norm_text(_findtext(pm, "kml:styleUrl"))
        ext = {}
        for d in pm.findall(".//kml:ExtendedData//kml:Data", namespaces=KML_NS):
            k = d.get("name")
            v = _findtext(d, "kml:value")
            if k:
                ext[norm_text(k)] = norm_text(v)

        # Extract display width (px) for visualization KML (optional)
        wpx = extract_inline_line_width(pm)
        if wpx is not None:
            ext[norm_text('_kml_line_width_px')] = wpx

        wpx = extract_inline_line_width(pm)
        if wpx is not None:
            ext[norm_text('_kml_line_width_px')] = wpx

        g = geom_from_placemark(pm)
        if g is None or g.geom_type != "Point":
            continue

        feats.append(FeatureLonLat(
            fid=name, name=name, folder_path=folder_path, folder_path_str="/".join(folder_path) if folder_path else "",
            style_url=style, description=desc, extdata=ext,
            geom_type=g.geom_type, geom_lonlat_wkt=g.wkt, geom_lonlat=g
        ))

    s, g, meta = detect_start_goal_from_features(feats, rules)
    meta["from_start_goal_kml"] = str(path)
    return s, g, meta

def polygon_equiv_circle(poly: Polygon) -> Tuple[Tuple[float, float], float]:
    c = poly.centroid
    area = abs(poly.area)
    r = math.sqrt(area / math.pi) if area > 0 else 1.0
    return (c.x, c.y), r

def line_equiv_rect(ls: LineString, width_m: float) -> Dict[str, float]:
    if ls.length <= 0:
        c = ls.centroid
        return {"center_x": float(c.x), "center_y": float(c.y), "length_m": 1.0, "width_m": float(width_m), "angle_deg": 0.0}

    buf = ls.buffer(max(width_m / 2.0, 0.1), cap_style=2, join_style=2)
    rect = buf.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)[:-1]

    edges = []
    for i in range(len(coords)):
        x1, y1 = coords[i]
        x2, y2 = coords[(i + 1) % len(coords)]
        edges.append((math.hypot(x2 - x1, y2 - y1), x1, y1, x2, y2))
    edges.sort(reverse=True, key=lambda t: t[0])

    length = edges[0][0] if edges else ls.length
    width = edges[-1][0] if edges else width_m
    angle = math.degrees(math.atan2(edges[0][4] - edges[0][2], edges[0][3] - edges[0][1])) if edges else 0.0
    center = rect.centroid

    return {"center_x": float(center.x), "center_y": float(center.y), "length_m": float(length), "width_m": float(width), "angle_deg": float(angle)}


def line_to_polygons(
    lines: List[LineString],
    max_use_per_line: int = 2,
    boundary_match_ratio: float = 0.30,
    snap_tol_m: float = 5.0,
    close_tol_m: float = 5.0,
    poly_boundary_support_ratio: float = 0.45,
    max_area_km2: float = 0.0,
    adaptive_snap: bool = True,
):
    """
    Convert a set of boundary lines to polygons using polygonize.

    Why polygons may be missing:
    - Some boundary lines are not exactly closed (first/last point not identical).
    - Endpoints have small gaps (digitizing / export / projection).
    - MultiLineString segments need to be exploded and noded.

    This function improves robustness by:
    - closing near-closed rings (distance <= close_tol_m)
    - snapping endpoints to the global line union (snap_tol_m)
    - using polygonize_full when available to collect dangles/cuts diagnostics
    """
    if not lines:
        return [], []

    def close_near_ring(ln: LineString) -> LineString:
        try:
            coords = list(ln.coords)
            if len(coords) >= 3:
                x0, y0 = coords[0][0], coords[0][1]
                x1, y1 = coords[-1][0], coords[-1][1]
                if ((x0 - x1) ** 2 + (y0 - y1) ** 2) ** 0.5 <= float(close_tol_m):
                    if coords[0] != coords[-1]:
                        coords.append(coords[0])
                    return LineString(coords)
        except Exception:
            pass
        return ln

    # 1) Close near-closed rings
    closed_lines = [close_near_ring(ln) for ln in lines if ln is not None and not ln.is_empty]

    # 2) Snap each line to the union to close small gaps
    try:
        ref = unary_union(closed_lines)
        snapped_lines = [snap(ln, ref, float(snap_tol_m)) for ln in closed_lines]
    except Exception:
        snapped_lines = closed_lines

    # 3) Node intersections and merge
    try:
        noded = unary_union(snapped_lines)
    except Exception:
        noded = unary_union(closed_lines)

    try:
        merged = linemerge(noded)
        merged_lines = (
            [merged]
            if isinstance(merged, LineString)
            else (list(merged.geoms) if isinstance(merged, MultiLineString) else snapped_lines)
        )
    except Exception:
        merged_lines = snapped_lines

    # 4) Polygonize (+ diagnostics if possible)
    polys = []
    dangles = None
    cuts = None
    invalids = None

    # First pass polygonize
    try:
        from shapely.ops import polygonize_full as _polygonize_full
        polys_g, dangles, cuts, invalids = _polygonize_full(merged_lines)
        polys = list(polys_g) if polys_g is not None else []
    except Exception:
        polys = list(polygonize(merged_lines))

    # Optional second pass with larger snap tolerance (only for polygonize) to recover near-closed loops
    polys2 = []
    if adaptive_snap and snap_tol_m and float(snap_tol_m) > 0:
        try:
            bigger = float(snap_tol_m) * 2.0
            ref2 = unary_union(merged_lines)
            snapped2 = []
            for ln in merged_lines:
                try:
                    snapped2.append(snap(ln, ref2, bigger))
                except Exception:
                    snapped2.append(ln)
            polys2 = list(polygonize(snapped2))
        except Exception:
            polys2 = []

    # Merge & rough dedupe (by representative point + area)
    all_polys = (polys or []) + (polys2 or [])
    uniq = []
    seen = set()
    for p in all_polys:
        if p is None or getattr(p, "is_empty", False):
            continue
        try:
            rp = p.representative_point()
            key = (round(float(rp.x), 2), round(float(rp.y), 2), round(float(p.area), 1))
        except Exception:
            key = (0.0, 0.0, round(float(getattr(p, "area", 0.0)), 1))
        if key in seen:
            continue
        seen.add(key)
        uniq.append(p)
    polys = uniq

    # Area cap (km^2) to prevent overflow polygons outside intended region
    max_area_m2 = float(max_area_km2) * 1_000_000.0 if float(max_area_km2 or 0.0) > 0 else 0.0
    if max_area_m2 > 0:
        polys = [p for p in polys if float(getattr(p, "area", 0.0)) <= max_area_m2]

    # Boundary support ratio filter: boundary must be supported by merged linework
    try:
        merged_union = unary_union(merged_lines)
    except Exception:
        merged_union = None

    def _boundary_support(p):
        if merged_union is None:
            return 0.0
        try:
            b = p.boundary
            bl = float(getattr(b, "length", 0.0)) or 0.0
            if bl <= 0:
                return 0.0
            inter = b.intersection(merged_union)
            il = float(getattr(inter, "length", 0.0)) or 0.0
            return il / bl
        except Exception:
            return 0.0

    # 5) Determine which source lines contribute to each polygon boundary + compute support
    contrib = []
    support_list = []
    for p in polys:
        b = p.boundary
        used = []
        for idx, ln in enumerate(lines):
            try:
                inter = b.intersection(ln)
                inter_len = float(getattr(inter, "length", 0.0)) or 0.0
                if ln.length > 0 and (inter_len / ln.length) >= boundary_match_ratio:
                    used.append(idx)
            except Exception:
                continue
        contrib.append(used)
        support_list.append(_boundary_support(p))

    # 6) Keep polygons: require support threshold; then apply line-use constraints
    order = sorted(range(len(polys)), key=lambda i: (-support_list[i], float(getattr(polys[i], "area", 0.0))))

    use = [0] * len(lines)
    kept = []
    review = []

    for i in range(len(polys)):
        review.append({
            "polygon_idx": i,
            "area": float(getattr(polys[i], "area", 0.0)),
            "boundary_support": float(support_list[i]),
            "used_lines": contrib[i],
        })

    for i in order:
        used = contrib[i]
        if not used:
            review.append({"polygon_idx": i, "kept": False, "used_lines": used, "area": float(getattr(polys[i], "area", 0.0)), "reason": "no_contributing_lines"})
            continue

        if float(support_list[i]) < float(poly_boundary_support_ratio):
            review.append({"polygon_idx": i, "kept": False, "used_lines": used, "area": float(getattr(polys[i], "area", 0.0)), "reason": "low_boundary_support", "boundary_support": float(support_list[i])})
            continue

        if all(use[u] < max_use_per_line for u in used):
            kept.append((polys[i], used))
            for u in used:
                use[u] += 1
            review.append({"polygon_idx": i, "kept": True, "used_lines": used, "area": float(getattr(polys[i], "area", 0.0)), "boundary_support": float(support_list[i])})
        else:
            review.append({"polygon_idx": i, "kept": False, "used_lines": used, "area": float(getattr(polys[i], "area", 0.0)), "boundary_support": float(support_list[i]), "reason": "line_use_limit"})
    # 7) Add diagnostics summary (dangles/cuts/invalids)
    try:
        diag = {
            "snap_tol_m": float(snap_tol_m),
            "close_tol_m": float(close_tol_m),
            "n_input_lines": len(lines),
            "n_polygons": len(polys),
            "n_kept": len(kept),
        }
        if dangles is not None:
            diag["dangles_length"] = float(getattr(dangles, "length", 0.0))
        if cuts is not None:
            diag["cuts_length"] = float(getattr(cuts, "length", 0.0))
        if invalids is not None:
            diag["invalids_length"] = float(getattr(invalids, "length", 0.0))
        review.append({"_diagnostics": diag})
    except Exception:
        pass

    return kept, review

def write_preview_kml_xy(path: Path, out_yaml: dict):
    kml = etree.Element("{http://www.opengis.net/kml/2.2}kml")
    doc = etree.SubElement(kml, "Document")
    etree.SubElement(doc, "name").text = "preprocess_preview_xy"

    def add_folder(title):
        f = etree.SubElement(doc, "Folder")
        etree.SubElement(f, "name").text = title
        return f

    def add_point(folder, title, x, y):
        pm = etree.SubElement(folder, "Placemark")
        etree.SubElement(pm, "name").text = title
        pt = etree.SubElement(pm, "Point")
        etree.SubElement(pt, "coordinates").text = f"{x},{y},0"

    def add_line(folder, title, pts):
        pm = etree.SubElement(folder, "Placemark")
        etree.SubElement(pm, "name").text = title
        ls = etree.SubElement(pm, "LineString")
        etree.SubElement(ls, "coordinates").text = " ".join([f"{x},{y},0" for x, y in pts])

    fg = add_folder("start_goal_xy")
    if out_yaml.get("start"):
        add_point(fg, "start", out_yaml["start"]["x"], out_yaml["start"]["y"])
    if out_yaml.get("goal"):
        add_point(fg, "goal", out_yaml["goal"]["x"], out_yaml["goal"]["y"])

    f_pts = add_folder("sensitive_points_xy")
    for p in out_yaml.get("sensitive_points", []):
        add_point(f_pts, p.get("name") or p.get("id"), p["x"], p["y"])

    f_t = add_folder("tower_points_xy")
    for p in out_yaml.get("tower_points", []):
        add_point(f_t, p.get("name") or p.get("id"), p["x"], p["y"])

    f_ro = add_folder("roads_xy")
    for r in out_yaml.get("roads", []):
        add_line(f_ro, r.get("name") or r.get("id"), r.get("points", []))

    f_co = add_folder("corridor_xy")
    corr = out_yaml.get("corridor") or {}
    if corr.get("primary"):
        add_line(f_co, corr["primary"].get("name") or corr["primary"].get("id"), corr["primary"].get("points", []))
    for r in (corr.get("others") or []):
        add_line(f_co, r.get("name") or r.get("id"), r.get("points", []))

    with open(path, "wb") as f:
        f.write(etree.tostring(kml, pretty_print=True, xml_declaration=True, encoding="UTF-8"))


def _kml_coords_from_geom_lonlat(g):
    """Convert shapely lon/lat geom to KML coordinate text (lon,lat,0)."""
    if g is None:
        return None
    gtype = g.geom_type
    if gtype == "Point":
        return f"{g.x},{g.y},0"
    if gtype == "LineString":
        return " ".join([f"{x},{y},0" for x, y in g.coords])
    if gtype == "Polygon":
        outer = " ".join([f"{x},{y},0" for x, y in g.exterior.coords])
        inners = []
        for ring in g.interiors:
            inners.append(" ".join([f"{x},{y},0" for x, y in ring.coords]))
        return {"outer": outer, "inners": inners}
    return None

def write_processed_kml_lonlat(path: Path, classified_recs: list, only_unknown: bool = False):
    """
    输出 WGS84 lon/lat KML，用于奥维/Google Earth 叠加展示。
    KML 坐标必须是经纬度；不可用本地米制 x/y。
    """
    kml = etree.Element("{http://www.opengis.net/kml/2.2}kml")
    doc = etree.SubElement(kml, "Document")
    etree.SubElement(doc, "name").text = "preprocess_processed_lonlat"

    styles = {
        "area": "7f0000ff",     # semi-transparent red (aabbggrr)
        "strip": "7f00ff00",    # semi-transparent green
        "point": "ff0000ff",    # red icon
        "road": "ff00ffff",
        "corridor": "ff888888",
        "tower_point": "ff00a5ff",
        "unknown": "7faaaaaa",
        "annotation": "ffcc00cc",
    }
    for role, color in styles.items():
        st = etree.SubElement(doc, "Style", id=f"st_{role}")
        ls = etree.SubElement(st, "LineStyle")
        etree.SubElement(ls, "color").text = color
        etree.SubElement(ls, "width").text = "2"
        ps = etree.SubElement(st, "PolyStyle")
        etree.SubElement(ps, "color").text = color
        etree.SubElement(ps, "fill").text = "1"
        etree.SubElement(ps, "outline").text = "1"
        ic = etree.SubElement(st, "IconStyle")
        etree.SubElement(ic, "color").text = color

    def add_pm(parent, rec, geom):
        # Optional: preserve original KML display line width (px) for visualization
        wpx = None
        try:
            wpx = (rec.get('extdata') or {}).get('_kml_line_width_px')
        except Exception:
            wpx = None
        role = rec.get("constraint_role") or "unknown"
        if only_unknown and rec.get("feature_type") != "unknown":
            return
        pm = etree.SubElement(parent, "Placemark")
        etree.SubElement(pm, "name").text = (rec.get("name") or rec.get("id") or "unnamed")[:120]
        etree.SubElement(pm, "styleUrl").text = f"#st_{role if role in styles else 'unknown'}"
        if wpx and geom.geom_type == "LineString":
            st = etree.SubElement(pm, "Style")
            ls = etree.SubElement(st, "LineStyle")
            etree.SubElement(ls, "width").text = str(int(wpx) if float(wpx).is_integer() else float(wpx))
        desc = (
            f"id: {rec.get('id')}\n"
            f"role: {role}\n"
            f"feature_type: {rec.get('feature_type')}\n"
            f"confidence: {rec.get('confidence')}\n"
            f"folder: {rec.get('folder_path_str')}\n"
            f"reason: {';'.join(rec.get('reason', []))}"
        )
        # Append params summary
        try:
            params = rec.get('params') or {}
            if 'width_m' in params:
                desc += f"\nwidth_m: {params.get('width_m')}"
            if 'hard' in params:
                desc += f"\nhard: {params.get('hard')}"
            if 'cost' in params:
                desc += f"\ncost: {params.get('cost')}"
        except Exception:
            pass
        etree.SubElement(pm, "description").text = desc

        gtype = geom.geom_type
        if gtype == "Point":
            pt = etree.SubElement(pm, "Point")
            etree.SubElement(pt, "coordinates").text = _kml_coords_from_geom_lonlat(geom)
        elif gtype == "LineString":
            ls = etree.SubElement(pm, "LineString")
            etree.SubElement(ls, "tessellate").text = "1"
            etree.SubElement(ls, "coordinates").text = _kml_coords_from_geom_lonlat(geom)
        elif gtype == "Polygon":
            poly = etree.SubElement(pm, "Polygon")
            etree.SubElement(poly, "tessellate").text = "1"
            cc = _kml_coords_from_geom_lonlat(geom)
            ob = etree.SubElement(poly, "outerBoundaryIs")
            lr = etree.SubElement(ob, "LinearRing")
            etree.SubElement(lr, "coordinates").text = cc["outer"]
            for inner in cc["inners"]:
                ib = etree.SubElement(poly, "innerBoundaryIs")
                lr2 = etree.SubElement(ib, "LinearRing")
                etree.SubElement(lr2, "coordinates").text = inner
        else:
            mg = etree.SubElement(pm, "MultiGeometry")
            for subg in getattr(geom, "geoms", []):
                sg = subg.geom_type
                if sg == "Point":
                    pt = etree.SubElement(mg, "Point")
                    etree.SubElement(pt, "coordinates").text = _kml_coords_from_geom_lonlat(subg)
                elif sg == "LineString":
                    ls = etree.SubElement(mg, "LineString")
                    etree.SubElement(ls, "tessellate").text = "1"
                    etree.SubElement(ls, "coordinates").text = _kml_coords_from_geom_lonlat(subg)
                elif sg == "Polygon":
                    poly = etree.SubElement(mg, "Polygon")
                    etree.SubElement(poly, "tessellate").text = "1"
                    cc = _kml_coords_from_geom_lonlat(subg)
                    ob = etree.SubElement(poly, "outerBoundaryIs")
                    lr = etree.SubElement(ob, "LinearRing")
                    etree.SubElement(lr, "coordinates").text = cc["outer"]
                    for inner in cc["inners"]:
                        ib = etree.SubElement(poly, "innerBoundaryIs")
                        lr2 = etree.SubElement(ib, "LinearRing")
                        etree.SubElement(lr2, "coordinates").text = inner

    folders = {}
    for rec in classified_recs:
        if only_unknown and rec.get("feature_type") != "unknown":
            continue
        top = (rec.get("folder_path") or ["(root)"])[0] if rec.get("folder_path") else "(root)"
        if top not in folders:
            fd = etree.SubElement(doc, "Folder")
            etree.SubElement(fd, "name").text = top
            folders[top] = fd
        try:
            geom = _wkt.loads(rec.get("geom_lonlat_wkt") or "")
        except Exception:
            continue
        add_pm(folders[top], rec, geom)

    with open(path, "wb") as f:
        f.write(etree.tostring(kml, pretty_print=True, xml_declaration=True, encoding="UTF-8"))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", "-i", type=str, default=None)
    ap.add_argument("--start_goal_kml", type=str, default=None, help="起点终点KML；不填则尝试 data/起点和终点.kml")
    ap.add_argument("--outdir", "-o", type=str, default=None)
    ap.add_argument("--rules", type=str, default=None)
    ap.add_argument("--overrides", type=str, default=None)
    ap.add_argument("--start_lonlat", nargs=2, type=float, default=None, help="手工指定起点 lon lat（最高优先级）")
    ap.add_argument("--goal_lonlat", nargs=2, type=float, default=None, help="手工指定终点 lon lat（最高优先级）")
    ap.add_argument("--buffer", type=float, default=1000.0)
    ap.add_argument("--resolution", type=float, default=50.0)
    args = ap.parse_args()

    proj = Path.cwd()

    # input default
    if args.input is None:
        data = proj / "data"
        if data.exists():
            km = sorted(list(data.glob("*.kml")))
            exm = [p for p in km if "exm" in p.name.lower()]
            args.input = str(exm[0]) if exm else (str(km[0]) if km else None)
    if args.input is None:
        raise SystemExit("No input KML found (use --input or put .kml in data/)")

    # rules default
    if args.rules is None:
        for cand in [proj / "preprocess_rules.yaml", proj / "configs" / "preprocess_rules.yaml", Path(__file__).parent / "preprocess_rules.yaml"]:
            if cand.exists():
                args.rules = str(cand)
                break
    rules = read_yaml(Path(args.rules)) if args.rules else {}

    # overrides default
    overrides_path = Path(args.overrides) if args.overrides else (proj / "overrides.csv" if (proj / "overrides.csv").exists() else None)
    overrides = load_overrides(overrides_path) if overrides_path else []
    clf = Classifier(rules, overrides)

    # output dirs
    out = Path(args.outdir) if args.outdir else (proj / "outputs" / f"preprocess_{now_ts()}")
    for d in [out, out / "yaml", out / "raw", out / "report", out / "review", out / "rules"]:
        ensure_dir(d)
    d_yaml, d_raw, d_report, d_review, d_rules = out / "yaml", out / "raw", out / "report", out / "review", out / "rules"

    # parse main KML (lonlat first)
    parser = etree.XMLParser(recover=True, huge_tree=True)
    tree = etree.parse(str(Path(args.input)), parser)
    root = tree.getroot()

    features_ll: List[FeatureLonLat] = []
    for folder_path, pm in walk_folders(root):
        name = norm_text(_findtext(pm, "kml:name")) or "unnamed"
        style = norm_text(_findtext(pm, "kml:styleUrl"))
        desc = norm_text(_findtext(pm, "kml:description"))
        ext = {}
        for d in pm.findall(".//kml:ExtendedData//kml:Data", namespaces=KML_NS):
            k = d.get("name")
            v = _findtext(d, "kml:value")
            if k:
                ext[norm_text(k)] = norm_text(v)

        g = geom_from_placemark(pm)
        if g is None:
            # Record skipped placemark for diagnostics
            try:
                hint = {
                    'has_Point': pm.find('.//kml:Point', namespaces=KML_NS) is not None,
                    'has_LineString': pm.find('.//kml:LineString', namespaces=KML_NS) is not None,
                    'has_Polygon': pm.find('.//kml:Polygon', namespaces=KML_NS) is not None,
                    'has_LinearRing': pm.find('.//kml:LinearRing', namespaces=KML_NS) is not None,
                    'has_MultiGeometry': pm.find('.//kml:MultiGeometry', namespaces=KML_NS) is not None,
                    'has_gxTrack': pm.find('.//gx:Track', namespaces=KML_NS) is not None,
                    'has_gxMultiTrack': pm.find('.//gx:MultiTrack', namespaces=KML_NS) is not None,
                }
                coord_node = pm.find('.//kml:coordinates', namespaces=KML_NS)
                coord_snip = ''
                if coord_node is not None and (coord_node.text or '').strip():
                    coord_snip = re.sub(r'\s+', ' ', (coord_node.text or '').strip())[:180]
                skipped.append({
                    'name': name,
                    'folder_path': folder_path,
                    'style': style,
                    'desc_snip': (re.sub(r'\s+', ' ', desc)[:160] if desc else ''),
                    'ext_keys': list(ext.keys())[:40],
                    'geometry_hint': hint,
                    'coordinates_snip': coord_snip,
                })
            except Exception:
                pass
            continue

        features_ll.append(FeatureLonLat(
            fid=name, name=name, folder_path=folder_path, folder_path_str="/".join(folder_path) if folder_path else "",
            style_url=style, description=desc, extdata=ext,
            geom_type=g.geom_type, geom_lonlat_wkt=g.wkt, geom_lonlat=g
        ))

    # start/goal priority: cli > start_goal_kml > main_kml
    meta_start_goal = {}
    start_ll = tuple(args.start_lonlat) if args.start_lonlat else None
    goal_ll = tuple(args.goal_lonlat) if args.goal_lonlat else None

    if args.start_goal_kml is None:
        cand = proj / "data" / "起点和终点.kml"
        if cand.exists():
            args.start_goal_kml = str(cand)

    if (start_ll is None or goal_ll is None) and args.start_goal_kml:
        s2, g2, m2 = read_start_goal_kml(Path(args.start_goal_kml), rules)
        meta_start_goal.update(m2)
        if start_ll is None:
            start_ll = s2
        if goal_ll is None:
            goal_ll = g2

    if start_ll is None or goal_ll is None:
        s3, g3, m3 = detect_start_goal_from_features(features_ll, rules)
        meta_start_goal.update(m3)
        if start_ll is None:
            start_ll = s3
        if goal_ll is None:
            goal_ll = g3

    # Robust centroid: real-world KML may contain invalid polygons causing unary_union to crash
    all_centroid = safe_centroid_lonlat([f.geom_lonlat for f in features_ll])
    if start_ll is None:
        start_ll = (float(all_centroid.x), float(all_centroid.y))
        meta_start_goal["fallback_start"] = "centroid"
    if goal_ll is None:
        goal_ll = (float(all_centroid.x + 0.01), float(all_centroid.y))
        meta_start_goal["fallback_goal"] = "centroid_offset"

    # single transformer using start
    tf = build_transformer(start_ll[0], start_ll[1])
    inv_tf = build_inverse_transformer(all_centroid.x, all_centroid.y)
    sx, sy = project_point_lonlat(start_ll[0], start_ll[1], tf)
    gx, gy = project_point_lonlat(goal_ll[0], goal_ll[1], tf)

    # project all features
    feats_xy = []
    for f in features_ll:
        gxy = project_geom(f.geom_lonlat, tf)
        if gxy is None:
            continue
        feats_xy.append((f, gxy))

    # de-dup ids
    seen = {}
    for f, _ in feats_xy:
        seen[f.fid] = seen.get(f.fid, 0) + 1
    dup = {k: 0 for k, v in seen.items() if v > 1}
    for f, _ in feats_xy:
        if seen[f.fid] > 1:
            dup[f.fid] += 1
            f.fid = f"{f.fid}__{dup[f.fid]}"

    # classify
    classified, unknown, annotations = [], [], []
    skipped = []  # placemarks whose geometry could not be parsed
    for f, gxy in feats_xy:
        res = clf.classify_text(f.fid, f.name, f.folder_path_str, f.description, f.geom_type)
        rec = {
            "id": f.fid,
            "name": f.name,
            "folder_path": f.folder_path,
            "folder_path_str": f.folder_path_str,
            "style_url": f.style_url,
            "description": f.description,
            "extdata": f.extdata,
            "geom_type": f.geom_type,
            "geom_lonlat_wkt": f.geom_lonlat_wkt,
            "geom_xy_wkt": gxy.wkt,
            "feature_type": res["feature_type"],
            "constraint_role": res["constraint_role"],
            "confidence": res["confidence"],
            "reason": res["reason"],
            "params": res["params"],
        }
        classified.append(rec)
        if rec["feature_type"] == "unknown":
            unknown.append(rec)
        if rec["feature_type"] == "annotation":
            annotations.append(rec)

    # raw jsonl
    with open(d_raw / "features_raw.jsonl", "w", encoding="utf-8") as fw:
        for rec in classified:
            fw.write(json.dumps(rec, ensure_ascii=False) + "\n")

    # report
    counts = {}
    for rec in classified:
        k = (rec["feature_type"], rec["constraint_role"], rec["confidence"])
        counts[k] = counts.get(k, 0) + 1
    rep = {
        "input": args.input,
        "start_goal_kml": args.start_goal_kml,
        "rules_file": args.rules,
        "overrides_file": str(overrides_path) if overrides_path else None,
        "total": len(classified),
        "counts": [{"feature_type": k[0], "constraint_role": k[1], "confidence": k[2], "count": v}
                   for k, v in sorted(counts.items(), key=lambda kv: -kv[1])],
        "unknown_examples": [{"id": u["id"], "name": u["name"], "folder": u["folder_path_str"],
                              "geom_type": u["geom_type"], "reason": u["reason"]}
                             for u in unknown[:80]],
        "start_goal_detect": meta_start_goal,
    }
    rep['skipped_placemarks_count'] = len(skipped)

    with open(d_report / "classification_report.json", "w", encoding="utf-8") as f:
        json.dump(rep, f, ensure_ascii=False, indent=2)

    # env bbox
    gxy_list = []
    for rec in classified:
        try:
            gxy_list.append(_wkt.loads(rec["geom_xy_wkt"]))
        except Exception:
            pass
    minx, miny, maxx, maxy = safe_union_bounds_xy(gxy_list)
    env = {
        "min_x": float(minx - args.buffer), "min_y": float(miny - args.buffer),
        "max_x": float(maxx + args.buffer), "max_y": float(maxy + args.buffer),
        "resolution_m": float(args.resolution), "crs": "local_utm_m"
    }

    start = {"x": sx, "y": sy, "lon": float(start_ll[0]), "lat": float(start_ll[1]), "source": "auto"}
    goal  = {"x": gx, "y": gy, "lon": float(goal_ll[0]),  "lat": float(goal_ll[1]),  "source": "auto"}
    if args.start_lonlat: start["source"] = "cli"
    elif args.start_goal_kml: start["source"] = "start_goal_kml_or_main"
    else: start["source"] = "main_kml_or_fallback"
    if args.goal_lonlat: goal["source"] = "cli"
    elif args.start_goal_kml: goal["source"] = "start_goal_kml_or_main"
    else: goal["source"] = "main_kml_or_fallback"

    defaults = rules.get("default_params", {})
    def_area = defaults.get("area", {"hard": False, "cost": 10.0, "enter_penalty": 0.0})
    def_strip = defaults.get("strip", {"width_m": 50.0, "hard": False, "cost": 5.0, "min_cross_angle_deg": 30.0})
    def_point = defaults.get("point", {"hard": True, "radius_m": 100.0, "cost": 100.0})

    areas, strips, pts, roads = [], [], [], []
    corridor = {"primary": None, "others": []}
    towers = []

    # airport L2P
    l2p = rules.get("line_to_polygon", {}) or {}
    airport_kw = l2p.get("keywords", ["机场净空", "净空", "规划机场"])
    airport_lines = []
    for rec in classified:
        is_airport = (rec.get("feature_type") == "airport_clearance") or any_match(airport_kw, f"{rec['folder_path_str']} {rec['name']}")
        if rec["geom_type"] in ("LineString", "MultiLineString") and is_airport:
            try:
                gxy = _wkt.loads(rec["geom_xy_wkt"])
                if gxy.geom_type == "MultiLineString":
                    airport_lines.extend([ls for ls in gxy.geoms if ls is not None and (not ls.is_empty)])
                elif gxy.geom_type == "LineString":
                    airport_lines.append(gxy)
                else:
                    # ignore non-line
                    pass
            except Exception:
                pass

    kept_polys, poly_review = [], []
    if l2p.get("enable", True) and airport_lines:
        kept_polys, poly_review = line_to_polygons(
            airport_lines,
            max_use_per_line=int(l2p.get("max_use_per_line", 2)),
            boundary_match_ratio=float(l2p.get("boundary_match_ratio", 0.30)),
            snap_tol_m=float(l2p.get("snap_tol_m", 5.0)),
            close_tol_m=float(l2p.get("close_tol_m", 5.0)),
            poly_boundary_support_ratio=float(l2p.get("poly_boundary_support_ratio", 0.45)),
            max_area_km2=float(l2p.get("max_area_km2", 0.0)),
            adaptive_snap=bool(l2p.get("adaptive_snap", True)),
        )
        with open(d_review / "line_to_polygon_review.json", "w", encoding="utf-8") as f:
            json.dump(poly_review, f, ensure_ascii=False, indent=2)
        with open(d_review / "line_to_polygon_review.geojson", "w", encoding="utf-8") as f:
            json.dump({
                "type": "FeatureCollection",
                "features": [{"type": "Feature",
                              "properties": {"idx": i, "used_lines": used, "area": float(poly.area)},
                              "geometry": mapping(poly)}
                             for i, (poly, used) in enumerate(kept_polys)]
            }, f, ensure_ascii=False, indent=2)

    # Export derived polygons for Ovi review (lon/lat)
    try:
        write_derived_polygons_kml_lonlat(d_review / 'derived_airport_clearance_polygons_lonlat.kml', kept_polys, inv_tf, name_prefix='机场净空派生面')
    except Exception:
        pass

    def to_raw(g): return mapping(g)

    for rec in classified:
        try:
            g = _wkt.loads(rec["geom_xy_wkt"])
        except Exception:
            continue

        trace = {
            "id": rec["id"], "name": rec["name"],
            "folder_path": rec["folder_path"], "folder_path_str": rec["folder_path_str"],
            "style_url": rec["style_url"], "description": rec["description"], "extdata": rec["extdata"],
            "feature_type": rec["feature_type"], "constraint_role": rec["constraint_role"],
            "confidence": rec["confidence"], "reason": rec["reason"],
        }

        if rec["feature_type"] == "annotation":
            continue

        role = rec["constraint_role"]

        if role == "area":
            p = {**def_area, **(rec.get("params") or {})}
            if g.geom_type in ("Polygon", "MultiPolygon"):
                poly = g if g.geom_type == "Polygon" else unary_union(list(g.geoms))
            elif g.geom_type == "LineString":
                width = float((rec.get("params") or {}).get("width_m", def_strip.get("width_m", 50.0)))
                poly = g.buffer(width / 2.0)
            else:
                continue
            (cx, cy), r = polygon_equiv_circle(poly)
            areas.append({
                "id": rec["id"], "name": rec["name"],
                "center_x": float(cx), "center_y": float(cy), "radius_m": float(max(r, 1.0)),
                "hard": bool(p.get("hard", False)), "cost": float(p.get("cost", 10.0)),
                "enter_penalty": float(p.get("enter_penalty", 0.0)),
                "raw_geom": to_raw(poly),
                "trace": trace,
            })

        elif role == "strip":
            p = {**def_strip, **(rec.get("params") or {})}
            if g.geom_type in ("LineString", "MultiLineString"):
                # For MultiLineString, keep full raw_geom and use the longest segment as legacy "points"
                if g.geom_type == "MultiLineString":
                    segs = [ls for ls in g.geoms if ls is not None and (not ls.is_empty)]
                    if not segs:
                        continue
                    segs_sorted = sorted(segs, key=lambda _ls: _ls.length, reverse=True)
                    main_ls = segs_sorted[0]
                    segments = [[[float(x), float(y)] for x, y in ls.coords] for ls in segs_sorted]
                else:
                    main_ls = g
                    segments = None
                rect = line_equiv_rect(g, float(p.get("width_m", 50.0)))
                strips.append({
                    "id": rec["id"], "name": rec["name"],
                    **rect,
                    "hard": bool(p.get("hard", False)), "cost": float(p.get("cost", 5.0)),
                    "min_cross_angle_deg": float(p.get("min_cross_angle_deg", 30.0)),
                    "raw_geom": to_raw(g),
                    "trace": trace,
                })

        elif role == "point":
            p = {**def_point, **(rec.get("params") or {})}
            if g.geom_type == "Point":
                pts.append({
                    "id": rec["id"], "name": rec["name"],
                    "x": float(g.x), "y": float(g.y),
                    "radius_m": float(p.get("radius_m", 100.0)),
                    "hard": bool(p.get("hard", True)), "cost": float(p.get("cost", 100.0)),
                    "raw_geom": to_raw(g),
                    "trace": trace,
                })

        elif role == "road":
            if g.geom_type in ("LineString", "MultiLineString"):
                if g.geom_type == "MultiLineString":
                    segs = [ls for ls in g.geoms if ls is not None and (not ls.is_empty)]
                    if not segs:
                        continue
                    segs_sorted = sorted(segs, key=lambda _ls: _ls.length, reverse=True)
                    main_ls = segs_sorted[0]
                    segments = [[[float(x), float(y)] for x, y in ls.coords] for ls in segs_sorted]
                else:
                    main_ls = g
                    segments = None
                roads.append({
                    "id": rec["id"], "name": rec["name"],
                    "points": [[float(x), float(y)] for x, y in main_ls.coords],
                    "segments": segments,
                    "raw_geom": to_raw(g),
                    "trace": trace,
                })

        elif role == "corridor":
            if g.geom_type in ("LineString", "MultiLineString"):
                if g.geom_type == "MultiLineString":
                    segs = [ls for ls in g.geoms if ls is not None and (not ls.is_empty)]
                    if not segs:
                        continue
                    segs_sorted = sorted(segs, key=lambda _ls: _ls.length, reverse=True)
                    main_ls = segs_sorted[0]
                    segments = [[[float(x), float(y)] for x, y in ls.coords] for ls in segs_sorted]
                else:
                    main_ls = g
                    segments = None
                item = {
                    "id": rec["id"], "name": rec["name"],
                    "points": [[float(x), float(y)] for x, y in main_ls.coords],
                    "raw_geom": to_raw(g),
                    "trace": trace,
                }
                if corridor["primary"] is None:
                    corridor["primary"] = item
                else:
                    corridor["others"].append(item)

        elif role == "tower_point":
            if g.geom_type == "Point":
                towers.append({
                    "id": rec["id"], "name": rec["name"],
                    "x": float(g.x), "y": float(g.y),
                    "raw_geom": to_raw(g),
                    "trace": trace,
                })

    if l2p.get("export_as_area", True) and kept_polys:
        p = {**def_area, **(l2p.get("area_params") or {})}
        for i, (poly, used) in enumerate(kept_polys):
            (cx, cy), r = polygon_equiv_circle(poly)
            areas.append({
                "id": f"airport_clearance_poly_{i}",
                "name": f"机场净空面_{i}",
                "center_x": float(cx), "center_y": float(cy), "radius_m": float(max(r, 1.0)),
                "hard": bool(p.get("hard", True)), "cost": float(p.get("cost", 9999.0)),
                "enter_penalty": float(p.get("enter_penalty", 0.0)),
                "raw_geom": to_raw(poly),
                "trace": {"feature_type": "airport_clearance_polygon", "used_lines": used},
            })

    ann = []
    for rec in annotations:
        try:
            g = _wkt.loads(rec["geom_xy_wkt"])
        except Exception:
            continue
        if g.geom_type == "Point":
            ann.append({
                "id": rec["id"],
                "name": rec["name"],
                "text": rec.get("description") or rec.get("name"),
                "x": float(g.x),
                "y": float(g.y),
                "raw_geom": to_raw(g),
                "trace": {"folder_path_str": rec["folder_path_str"], "style_url": rec["style_url"]},
            })

    out_yaml = {
        "env": env,
        "start": start,
        "goal": goal,
        "areas": areas,
        "strips": strips,
        "sensitive_points": pts,
        "roads": roads,
        "corridor": corridor,
        "tower_points": towers,
        "annotations": ann,
        "meta": {
            "input_file": args.input,
            "start_goal_kml": args.start_goal_kml,
            "generated_at": _dt.datetime.now().isoformat(),
            "rules_file": args.rules,
            "overrides_file": str(overrides_path) if overrides_path else None,
            "start_goal_detect": meta_start_goal,
        }
    }

    with open(d_yaml / "map_generated.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(out_yaml, f, allow_unicode=True, sort_keys=False)

    if args.rules:
        try:
            with open(args.rules, "r", encoding="utf-8") as fr, open(d_rules / "rules_effective.yaml", "w", encoding="utf-8") as fw:
                fw.write(fr.read())
        except Exception:
            pass

    features = []
    for rec in unknown:
        try:
            g = _wkt.loads(rec["geom_xy_wkt"])
        except Exception:
            continue
        features.append({
            "type": "Feature",
            "properties": {
                "id": rec["id"], "name": rec["name"], "folder": rec["folder_path_str"],
                "geom_type": rec["geom_type"], "reason": ";".join(rec.get("reason", []))
            },
            "geometry": mapping(g)
        })
    with open(d_review / "unknown_features.geojson", "w", encoding="utf-8") as f:
        json.dump({"type": "FeatureCollection", "features": features}, f, ensure_ascii=False, indent=2)

    # Diagnostics: placemarks whose geometry could not be parsed
    if skipped:
        with open(d_review / 'skipped_placemarks.jsonl', 'w', encoding='utf-8') as f:
            for rec in skipped:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    write_preview_kml_xy(d_review / "preview.kml", out_yaml)

    # WGS84 lon/lat KML for Ovi/Google Earth overlay
    write_processed_kml_lonlat(d_review / "processed_features_lonlat.kml", classified_recs=classified, only_unknown=False)
    write_processed_kml_lonlat(d_review / "unknown_features_lonlat.kml", classified_recs=classified, only_unknown=True)

    print("Done. Output:", out)
    print("YAML:", d_yaml / "map_generated.yaml")

if __name__ == "__main__":
    main()
