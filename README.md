# 输电线路路径优化-数据预处理 (KML → YAML)

本模块将真实工程 KML 地物数据（含多层目录/混放/多段坐标）转换为**算法端可读**的 `map_generated.yaml`，并输出可视化复核与人工修正所需的辅助文件。

- 输入：地物 `*.kml`（可选：起终点 `*.kml`）
- 输出：`outputs/preprocess_<timestamp>/yaml/map_generated.yaml` + 复核/调参文件

---

## 0. 环境（与算法端一致即可）

Python 3.9+，主要依赖：`lxml / shapely / pyproj / pyyaml`

---

## 1. 快速运行

### 1.1 地物（或内含起终点） kml
```bash
python preprocess_kml_to_mapyaml.py --input data/YX工程资料（项目B）.kml
```

### 1.2 地物 + 起终点
```bash
python preprocess_kml_to_mapyaml.py --input data/YX工程资料（项目B）.kml --start_goal_kml data/起点和终点.kml
```

### 1.3 使用自定义规则库
```bash
python preprocess_kml_to_mapyaml.py --input data/YX工程资料（项目B）.kml --rules rules/preprocess_rules.yaml
```
> 否则默认使用根目录下的规则库

### 1.4 第二次运行（应用人工覆盖 overrides）
默认会自动读取上次输出目录下的 `overrides.csv`（或手动指定）：
```bash
python preprocess_kml_to_mapyaml.py --input data/YX工程资料（项目B）.kml --overrides outputs/preprocess_xxx/overrides.csv
```

> 若overrides.csv放根目录：`--overrides overrides.csv`。

---

## 2. 输入说明

### 2.1 地物 KML（必选）
支持：Folder 多层嵌套、Placemark/MultiGeometry、多段 `<coordinates>`、同目录混放不同地物。

### 2.2 起终点 KML（可选）
可独立提供；也支持起终点与地物合并在同一 KML 时自动识别。

---

## 3. 输出目录与文件用途

输出目录：`outputs/preprocess_<timestamp>/`

### 3.1 对齐算法端输入
- `yaml/map_generated.yaml`：算法端主输入（约束要素 + 起终点 + 参数）。
- 包含：
  - `start_point` / `goal_point`（若提供）
  - `areas` / `strips` / `points` 等约束集合
  - `raw_geom`保留真实几何字段（用于未来算法升级）

### 3.2 可视化/质检
- `raw/features_raw.jsonl`：**逐要素明细**（一行一个要素），包含 id/name/folder、分类结果、几何与参数；可用于可视化。
- 一行一个要素，包含：
  - `id`：稳定ID（用于 overrides 精确覆盖）
  - `name`、`folder_path`
  - `feature_type`、`constraint_role`、`confidence`、`reason`
  - `geom_lonlat_wkt`、`geom_xy_wkt` 等几何字段
  
- `review/processed_features_lonlat.kml`：复核 KML（奥维/Google Earth 检查几何完整性与分类）。
- `review/derived_airport_clearance_polygons_lonlat.kml`：机场净空“线转面”派生面复核（若启用线转面）。

### 3.3 人工修正闭环（必用）
- `overrides.csv`：专家修正入口（覆盖分类/角色/关键参数）。二次运行时通过 `--overrides` 应用。
- `rules/rules_effective.yaml`：本次运行的实际生效规则快照（可追溯/可复现）。

### 3.4 调参与诊断
- `review/line_to_polygon_review.json`：线转面诊断（调参用）。
- `classification_report.json`：分类统计汇总（不是逐要素明细）。

---

## 4. 关键字段说明（JSONL / YAML）

### 4.1 分类字段
- `feature_type`：地物语义类型（如 `road / river / dense_housing / airport_clearance ...`）
- `constraint_role`：约束角色
  - `area` 面约束（Polygon/MultiPolygon）
  - `strip` 带状约束（LineString + `width_m`）
  - `point` 点约束（Point + `forbidden_radius_m`）
  - `annotation` 仅展示说明（算法端可忽略）
  - `unknown` 未识别（建议人工修正或扩充规则）

### 4.2 几何字段
- `geom_xy_wkt`：**米制平面坐标 WKT**
- `geom_lonlat_wkt`：经纬度 WKT
- `raw_geom`：真实地物形状的结构化表示（GeoJSON）

> 若算法端从“简化几何”升级为“真实地物形状”，可以改为读取 yaml中的`raw_geom`。

### 4.3 约束参数字段
- `params`：约束参数字典（不同 role 含义不同）
  - `area`：`hard / cost / enter_penalty`
  - `strip`：`width_m`（可选 `hard/cost`）
  - `point`：`forbidden_radius_m` 等

---

## 5. 人工修正流程（overrides）

### 5.1 流程
1) 首次运行生成 `raw/features_raw.jsonl`（检查 unknown/误分）  
2) 编辑 `overrides.csv`（按 `id` 精确覆盖）  
3) 二次运行：`--overrides overrides.csv` 生成最终 `map_generated.yaml`

### 5.2 overrides.csv 示例
```csv
id,feature_type,constraint_role,hard,cost,width_m,comment
feat_000123,dense_housing,area,true,9999,,专家确认密集房屋为硬禁入
feat_000456,road,strip,,,30,道路宽度修正为30m
feat_000789,,annotation,,,,仅展示不参与约束
```

---

## 6. 机场净空线转面

规则库示例（`preprocess_rules.yaml`）：
```yaml
line_to_polygon:
  enable: true
  keywords: ["机场净空", "净空", "规划机场", "airport", "机场"]
  max_use_per_line: 3
  boundary_match_ratio: 0.42  #不高于0.45
  snap_tol_m: 30.0
  close_tol_m: 30.0
  max_area_km2: 80.0                 
  adaptive_snap: true                 
  poly_boundary_support_ratio: 0.30   
  export_as_area: true
  area_params: {hard: true, cost: 9999.0, enter_penalty: 0.0}
```
