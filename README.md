# 输电线路选线-路径优化 Demo (A\*)

本 Demo 用于把“条状物跨越、片状区域阻尼/进入损失、拐点落点与塔位、平行走廊复用、敏感点避让、道路利用”等约束抽象为**可配置**的规则，并在二维栅格上用 A\* 搜索跑通一个可视化流程。

- 输入：`configs/demo_map.yaml` (模拟场景数据) + `configs/demo_config.yaml` (惩罚/奖励外参)
- 输出：`outputs/<timestamp>/final.png` + `heatmap.png` + `search.gif` + `report.json`

## 0. 环境配置（Conda / pip）

本 Demo 为纯 Python 脚本工程，建议使用 conda 创建隔离环境。

### 0.1 环境配置

在项目目录执行：

```bash
conda env create -f environment.yml
conda activate route_opt
```


## 1. 快速运行

在项目目录执行：

```bash
python run_demo.py --config configs/demo_config.yaml --map configs/demo_map.yaml
```

运行成功后会打印输出目录，例如：

```
[demo] Output directory: .../outputs/20260120_123456
```

## 2. 约束类与对应逻辑

| 约束类型                       | 类                            | 作用                                                                                                            |
| ------------------------------ | ----------------------------- | --------------------------------------------------------------------------------------------------------------- |
| 片状区域阻尼/进入损失          | `AreaDampingConstraint`       | 圆形区域：不可进入/不可跨越 -> 硬约束；可进入 -> 进入惩罚 `entry_penalty` + 区域内每格代价 `damping/cross_cost` |
| 条状物跨越+最小交叉角          | `StripCrossingConstraint`     | 矩形走廊：不可跨越 -> 硬约束；可跨越 -> 交叉角 >= `min_cross_angle_deg`，并计入 `cross_cost`                    |
| 敏感设施避让                   | `SensitiveFacilityConstraint` | 圆形硬禁区 + 软缓冲惩罚                                                                                         |
| 走廊复用/平行导向奖励(含密度)  | `CorridorReuseConstraint`     | 静态：走廊密度超过阈值惩罚；动态：靠近走廊且移动方向与走廊切向一致时给奖励(降低代价)                            |
| 交通条件利用(道路奖励)         | `RoadRewardConstraint`        | 静态靠近道路奖励；可选方向性奖励                                                                                |
| 拐点落点/塔位候选              | `LandablePointsConstraint`    | 生成 `landable` 掩膜：距硬障碍太近不可落点；随机模拟征地/地质禁落点                                             |
| 转角约束                       | `TurnAngleConstraint`         | A\* 状态含 heading：转角超过阈值硬阻；转角产生软代价；转角点必须落在 landable                                   |
| 直线塔(<=800m 且每段至少 1 座) | `place_towers()`              | 对转角段后处理布塔：满足 `max_span_m` 与 `min_straight_tower_per_segment`                                       |

> 备注：A\* 目前运行在栅格上；未来换 Dijkstra/ACO/PSO/采样类算法，只需要复用 `ConstraintManager` 提供的 cost/blocked 查询即可。

## 3. 外参配置说明

所有“惩罚/奖励/阈值”均外置在 `configs/demo_config.yaml`，典型可调参数：

- `constraints.strip_crossing.min_cross_angle_deg`：最小交叉角度
- `constraints.area_damping.entry_penalty/damping_cell_cost`：进入一次性损失与区域内代价
- `constraints.corridor_reuse.directional_reward_factor`：走廊导向奖励强度
- `constraints.corridor_reuse.max_density/dense_penalty`：走廊密度限制
- `constraints.turn_angle.max_turn_angle_deg/turn_cost_weight`：转角硬阈值与造价权重
- `postprocess.max_span_m`：塔间最大间距(默认 800m)

## 4. 输出文件

- `final.png`：叠加静态惩罚热力图 + 约束实体 + 搜索访问点 + 最终路径 + 塔位点
- `heatmap.png`：仅静态环境惩罚热力图
- `search.gif`：A\* 探索过程 GIF (按扩展步数采样绘帧)
- `report.json`：结果与约束校验摘要
