# 输电线路路径优化约束的数学建模说明

> 本文档用于将工程实现中的各类路径约束，形式化为统一的数学表达，作为算法设计、参数讨论、模型审查与后续优化的依据。  
> 本模型不绑定具体求解算法（A* / BIT* / RRT* / PSO 等），仅定义**可行性约束（Hard Constraint）**与**代价 / 奖励函数（Soft Constraint）**。

---

## 1. 路径与基本符号定义

### 1.1 路径表示

定义一条输电线路路径为二维平面上的有序点集：

\[
\mathcal{P} = \{ \mathbf{p}_0, \mathbf{p}_1, \dots, \mathbf{p}_N \}, 
\quad \mathbf{p}_i = (x_i, y_i)
\]

其中：
- \(\mathbf{p}_0\)：起点  
- \(\mathbf{p}_N\)：终点  
- \(\mathbf{p}_i\)：拐点或塔位候选点  

相邻两点构成路径线段：

\[
\mathbf{s}_i = (\mathbf{p}_i, \mathbf{p}_{i+1})
\]

---

## 2. 总体优化目标

在满足全部硬约束的前提下，最小化路径总代价函数：

\[
J(\mathcal{P}) =
\sum_{i=0}^{N-1} 
\big(
C_{\text{length}}(\mathbf{s}_i)
+ C_{\text{strip}}(\mathbf{s}_i)
+ C_{\text{area}}(\mathbf{s}_i)
+ C_{\text{turn}}(\mathbf{p}_i)
+ C_{\text{corridor}}(\mathbf{s}_i)
\big)
\]

---

## 3. 条状物交叉跨越约束（Strip Crossing）

### 3.1 几何与交叉判定

条状物 \(k\) 定义为矩形区域 \(\mathcal{R}_k\)。  
当路径线段 \(\mathbf{s}_i\) 与其存在交集：

\[
\mathbf{s}_i \cap \mathcal{R}_k \neq \emptyset
\]

则判定发生跨越。

---

### 3.2 不可跨越硬约束

若条状物设置为不可跨越：

\[
\text{is\_crossable}_k = \text{False}
\Rightarrow
\mathcal{P} \text{ 不可行}
\]

---

### 3.3 最小交叉角约束

定义路径线段方向向量：

\[
\mathbf{v}_i = \mathbf{p}_{i+1} - \mathbf{p}_i
\]

条状物主轴方向为 \(\mathbf{d}_k\)，交叉角定义为：

\[
\theta_{ik} =
\arccos
\left(
\frac{|\mathbf{v}_i \cdot \mathbf{d}_k|}
{\|\mathbf{v}_i\| \|\mathbf{d}_k\|}
\right)
\]

必须满足：

\[
\theta_{ik} \ge \theta_k^{\min}
\]

---

### 3.4 跨越代价

满足角度约束后，引入跨越代价：

\[
C_{\text{strip}} =
\sum_k
\mathbb{I}(\mathbf{s}_i \cap \mathcal{R}_k \neq \emptyset)
\cdot w_k^{\text{cross}}
\]

---

## 4. 片状区域阻尼与进入代价（Area Damping）

### 4.1 区域定义

片状区域 \(j\) 定义为圆形区域：

\[
\mathcal{A}_j = 
\{ \mathbf{x} \mid \|\mathbf{x} - \mathbf{c}_j\| \le r_j \}
\]

---

### 4.2 不可进入 / 不可跨越约束

若：

\[
\text{is\_enterable}_j = \text{False}
\quad \text{或} \quad
\text{is\_crossable}_j = \text{False}
\]

则：

\[
\mathbf{s}_i \cap \mathcal{A}_j \neq \emptyset
\Rightarrow
\mathcal{P} \text{ 不可行}
\]

---

### 4.3 进入损失

路径首次进入区域时，计入一次性代价：

\[
C_{\text{entry}} =
\sum_j
\mathbb{I}(\mathbf{s}_i \text{ 从外进入 } \mathcal{A}_j)
\cdot w_j^{\text{entry}}
\]

---

### 4.4 区域内阻尼代价

路径在区域内的长度为 \(L_{ij}\)，则：

\[
C_{\text{damping}} =
\sum_j
\alpha_j \cdot L_{ij}
\]

---

## 5. 拐点转角约束（Turning Angle）

对连续三点 \(\mathbf{p}_{i-1}, \mathbf{p}_i, \mathbf{p}_{i+1}\)，转角定义为：

\[
\phi_i =
\arccos
\left(
\frac{
(\mathbf{p}_i - \mathbf{p}_{i-1}) \cdot
(\mathbf{p}_{i+1} - \mathbf{p}_i)
}
{
\|\mathbf{p}_i - \mathbf{p}_{i-1}\|
\|\mathbf{p}_{i+1} - \mathbf{p}_i\|
}
\right)
\]

### 5.1 最大转角硬约束

\[
\phi_i \le \phi_{\max}
\]

---

### 5.2 转角代价

\[
C_{\text{turn}} =
w_{\text{turn}} \cdot \phi_i
\]

---

## 6. 落点 / 塔位约束（Landable Constraint）

### 6.1 可落点约束

任意塔位 \(\mathbf{p}_i\) 必须满足：

\[
\mathbf{p}_i \in \mathcal{L}
\]

其中 \(\mathcal{L}\) 为可落点集合。

---

### 6.2 安全距离约束

\[
\min_{\mathbf{o} \in \mathcal{O}}
\|\mathbf{p}_i - \mathbf{o}\|
\ge d_{\min}
\]

---

## 7. 直线塔布设约束（800 米规则）

### 7.1 塔间距约束

\[
\|\mathbf{p}_{i+1} - \mathbf{p}_i\| \le 800
\]

---

### 7.2 最小直线塔数量

设拐点段长度为 \(L\)，则：

\[
n \ge \max
\left(
1,
\left\lceil \frac{L}{800} \right\rceil - 1
\right)
\]

---

## 8. 平行走廊复用与导向奖励（Corridor Constraint）

### 8.1 走廊距离判定

若路径线段与已有线路的最小距离满足：

\[
d \le d_{\text{corridor}}
\]

则视为进入走廊。

---

### 8.2 方向一致性奖励

设已有线路切向方向为 \(\mathbf{t}\)，路径方向为 \(\mathbf{v}_i\)：

\[
\eta_i =
\left|
\frac{\mathbf{v}_i \cdot \mathbf{t}}
{\|\mathbf{v}_i\| \|\mathbf{t}\|}
\right|
\]

导向奖励定义为：

\[
C_{\text{corridor}} =
- w_{\text{align}} \cdot \eta_i
\]

---

## 9. 硬约束与软约束总结

| 类型 | 数学形式 | 约束类型 |
|----|----|----|
| 不可跨越区域 | 几何可行性 | Hard |
| 最小交叉角 | \(\theta \ge \theta_{\min}\) | Hard |
| 最大转角 | \(\phi \le \phi_{\max}\) | Hard |
| 落点合法性 | \(\mathbf{p}_i \in \mathcal{L}\) | Hard |
| 路径长度 | \(\|\mathbf{s}_i\|\) | Soft |
| 跨越代价 | \(w_{\text{cross}}\) | Soft |
| 区域阻尼 | \(\alpha L\) | Soft |
| 走廊奖励 | \(-w_{\text{align}}\eta\) | Soft |

---

## 10. 说明

- 所有参数均通过 YAML / JSON 外部配置；
- 硬约束用于可行性剪枝；
- 软约束统一进入目标函数；
- 算法本身与约束建模完全解耦。


---

## 11. 约束项中的人工可配置惩罚 / 奖励系数汇总

本节用于明确：**每一类约束中，哪些参数需要人工设定（外参）**，以及它们在数学模型与工程实现中的作用。
这些参数通常通过 `YAML / JSON` 配置文件给定，是工程调参与多目标权衡的主要抓手。

---

### 11.1 条状物交叉跨越约束（Strip）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 最小交叉角 | \(\theta_{\min}\) | 限制跨越时的最小交叉角 | 20°–45° |
| 跨越代价 | \(w_{\text{strip}}^{\text{cross}}\) | 每次跨越条状物的附加代价 | 10–100 |
| 角度惩罚权重（可选） | \(w_{\text{angle}}\) | 对接近最小角度的跨越进行额外惩罚 | 0–50 |

---

### 11.2 片状区域阻尼与进入代价（Area）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 进入损失 | \(w_{\text{entry}}\) | 首次进入区域的一次性代价 | 20–200 |
| 阻尼系数 | \(\alpha\) | 区域内单位长度的附加代价 | 1–5 × 基础长度代价 |
| 跨越代价 | \(w_{\text{area}}^{\text{cross}}\) | 穿越区域的额外代价（可选） | 0–100 |
| 缓冲区惩罚（可选） | \(w_{\text{buffer}}\) | 进入区域周边缓冲带的惩罚 | 0–50 |

---

### 11.3 拐点转角约束（Turning）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 最大允许转角 | \(\phi_{\max}\) | 控制线路最大折角 | 30°–60° |
| 转角代价权重 | \(w_{\text{turn}}\) | 转角大小对应的施工代价 | 1–10 / 度 |

---

### 11.4 落点 / 塔位约束（Landable）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 最小安全距离 | \(d_{\min}\) | 塔位与障碍物最小水平距离 | 30–100 m |
| 禁止落点半径 | \(r_{\text{forbid}}\) | 快速判定不可落点的缓冲半径 | 50–150 m |
| 非理想落点惩罚（可选） | \(w_{\text{land}}\) | 勉强可用落点的附加代价 | 0–100 |

---

### 11.5 直线塔布设约束（Span）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 最大塔间距 | \(D_{\max}\) | 单跨最大允许距离 | 固定 800 m |
| 直线塔代价 | \(w_{\text{tower}}\) | 每新增一座直线塔的代价 | 50–300 |

---

### 11.6 平行走廊复用与导向奖励（Corridor）

| 参数名 | 数学符号 | 作用说明 | 典型取值建议 |
|---|---|---|---|
| 走廊距离阈值 | \(d_{\text{corridor}}\) | 判定是否进入走廊的距离 | 20–100 m |
| 方向奖励权重 | \(w_{\text{align}}\) | 与既有线路方向一致的奖励 | 5–50 |
| 密度惩罚权重 | \(w_{\text{density}}\) | 走廊过密时的惩罚 | 10–200 |
| 最大允许密度 | \(\rho_{\max}\) | 单位区域内允许的线路数量 | 按规划等级设定 |

---

### 11.7 参数设计原则（工程建议）

1. **硬约束参数**（角度阈值、距离阈值）  
   → 建议直接来源于规范或工程经验，不参与优化。

2. **软约束权重**  
   → 初期可统一量纲（例如以“等效米”为单位），便于调参。

3. **奖励项权重应小于主要惩罚项**  
   → 避免出现“为了奖励而违反工程直觉”的路径。

4. **不同区域可使用不同参数集**  
   → 同一算法 + 不同 YAML = 不同工程策略。

---

> 注：上述参数均为“策略层参数”，不影响算法正确性，只影响解的工程偏好。
