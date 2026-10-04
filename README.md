# terrain_analyzer

基于**协方差矩阵、PCA 主成分分析与高斯统计**的地形分析 ROS 2 功能包。

节点将输入点云按 XY 平面栅格化，对每个栅格内的点集做 PCA 平面拟合与几何特征提取，得到特征值、几何特征、法向量、坡度、高度统计、平面残差与异常点比例，并据此分类地形类型（平地 / 斜坡 / 起伏 / 杂乱 / 边缘），维护一张带时序融合的局部地形栅格地图，向外发布自定义分析消息与可视化 Marker。

---

## 功能特性

- **PCA 几何特征**：由协方差矩阵特征分解得到线状性、面状性、散乱性。
- **局部平面法向量**：取最小特征值对应的特征向量作为法向量。
- **坡度**：由法向量与竖直方向的夹角计算。
- **高度统计**：z 的均值与标准差。
- **平面残差统计**：点到拟合平面的有符号距离的均值 / 标准差。
- **高斯异常点比例**：残差超过 `mean + 3σ` 的点占比。
- **栅格化地形建图**：按 `cell_size` 划分 XY 栅格，逐格分析。
- **时序融合**：已存在栅格与新观测按指数滑动平均（α = 0.3）融合。
- **地形分类**：依据坡度、面状性、散乱性、线状性判定地形类型。
- **置信度**：由点数、面状性、异常点比例加权得到 `confidence`。
- **可通过性**：由坡度、散乱性与置信度加权得到 `traversability`（坡度超阈值直接置零）。
- **局部地图裁剪**：移除距原点超过 `map_radius` 的栅格。
- **可视化**：发布 `MarkerArray`（CUBE），按可通过性由红（不可通过）到绿（可通过）着色。
- **自定义消息**：发布 `terrain_analyzer/msg/TerrainAnalysis`。

---

## 目录结构

```
terrain_analyzer/
├── CMakeLists.txt                    # 构建脚本
├── package.xml                       # 功能包清单
├── include/
│   └── terrain_analyzer/
│       └── terrain_analyzer.hpp      # 核心类与结果结构体声明
├── src/
│   ├── terrain_analyzer.cpp          # 核心算法实现（库）
│   ├── terrain_analyzer_node.cpp     # ROS 2 节点（可执行）
│   └── test_terrain_analyzer.cpp     # 离线测试 / 演示程序（可执行）
└── msg/
    └── TerrainAnalysis.msg           # 自定义消息定义（已接入构建）
```

构建产物：

| 目标 | 类型 | 说明 |
| --- | --- | --- |
| `terrain_analyzer_core` | 静态库 | 纯算法实现，不依赖 ROS |
| `terrain_analyzer_node` | 可执行 | ROS 2 订阅 / 建图 / 发布节点 |
| `test_terrain_analyzer` | 可执行 | 使用合成数据的离线演示（仅依赖 `core` + Eigen） |
| `terrain_analyzer/msg/TerrainAnalysis` | 消息接口 | 由 `rosidl_generate_interfaces` 生成 |

---

## 算法原理

对输入点集 $\{p_i\}$：

1. **质心**：$\bar{p} = \frac{1}{N}\sum_i p_i$
2. **协方差矩阵**：$C = \frac{1}{N}\sum_i (p_i-\bar{p})(p_i-\bar{p})^T$
3. **特征分解**：对 $C$ 求解对称特征值问题，得到 $\lambda_1 \ge \lambda_2 \ge \lambda_3$ 及对应特征向量。
4. **特征提取**：
   - 法向量 $\mathbf{n}$ = 最小特征值 $\lambda_3$ 对应的特征向量
   - 坡度 $\theta = \arccos(|n_z|)$ （弧度）
   - 线状性 $\text{linearity} = (\lambda_1-\lambda_2)/\lambda_1$
   - 面状性 $\text{planarity} = (\lambda_2-\lambda_3)/\lambda_1$
   - 散乱性 $\text{scattering} = \lambda_3/\lambda_1$
5. **高度统计**：对 z 分量求均值与标准差。
6. **平面残差**：$r_i = \mathbf{n}\cdot(p_i-\bar{p})$，统计均值与标准差。
7. **异常点比例**：残差 $r_i > \text{mean} + 3\sigma$ 的点所占比例（仅当 $\sigma > 10^{-12}$ 时计算）。

> 输入点数少于 3 或特征分解失败时，结果 `valid = false`。

### 地形分类规则

`classifyTerrain()` 按以下顺序判定，命中即返回：

| 类型码 | 名称 | 判定条件 |
| --- | --- | --- |
| `1` | `FLAT` 平地 | `slope < 10°` 且 `planarity > 0.8` |
| `2` | `SLOPE` 斜坡 | `10° ≤ slope < 30°` 且 `planarity > 0.7` |
| `3` | `ROUGH` 起伏 | `scattering > 0.1` |
| `4` | `IRREGULAR` 杂乱 | `planarity < 0.5` |
| `5` | `EDGE` 边缘 | `linearity > 0.6` |
| `0` | `UNKNOWN` 未知 | 以上都不满足，或 `valid = false` |

> Marker 颜色不按地形类型着色，而是按后面的 `traversability` 由红到绿渐变；`terrain_type` 仅出现在消息字段中。

### 置信度与可通过性

每个有效栅格在分类之外，额外计算两个 0~1 的指标（`terrain_analyzer_node.cpp`）：

**置信度 `confidence`**——衡量该栅格分析结果的可靠程度：

$$\text{confidence} = 0.5\cdot\text{point\_conf} + 0.3\cdot\text{planarity\_conf} + 0.2\cdot\text{outlier\_conf}$$

其中

- `point_conf = min(point_count / 200, 1)`：点数越多越可信。
- `planarity_conf = clamp(planarity, 0, 1)`：越接近平面越可信。
- `outlier_conf = 1 - min(outlier_ratio, 1)`：异常点越少越可信。

**可通过性 `traversability`**——衡量该栅格对机器人通行的友好程度：

$$\text{slope\_score} = \max\left(0,\ 1 - \frac{\text{slope}}{30°}\right),\qquad
\text{roughness\_score} = \max\left(0,\ 1 - \min\left(\frac{\text{scattering}}{0.1},\ 1\right)\right)$$

- 若 `slope > 30°`：`traversability = 0`（超过最大可通行坡度直接判为不可通行）。
- 否则：`traversability = 0.5·slope_score + 0.3·roughness_score + 0.2·confidence`。

### 时序融合

栅格已在 `terrain_map_` 中时，用指数滑动平均（硬编码 `α = 0.3`）融合新旧观测：

$$\text{fused} = (1-\alpha)\cdot\text{old} + \alpha\cdot\text{new}$$

位置 `x`、`y`、`point_count` 直接取新观测；法向量融合后重新归一化；其余标量（含 `confidence`、`traversability`）均按上式 EMA 融合；融合完成后用融合值重新分类 `terrain_type`。

---

## 节点处理流程

1. 从 `PointCloud2` 读取 `x`/`y`/`z`（要求 `float32`），跳过非有限值（NaN/Inf）。
2. 若有效点数少于 `min_points`，告警并丢弃该帧。
3. 按 `cell_size` 分组：`grid = floor(x / cell_size)`、`floor(y / cell_size)`，栅格中心为 `(index + 0.5) * cell_size`。
4. 每个点数 ≥ `min_points` 的栅格调用 `TerrainAnalyzer::analyze()`；失败则跳过。
5. 填充 `TerrainCell`，计算 `confidence` 与 `traversability`，分类 `terrain_type`，统计各类型数量。
6. 融合进 `terrain_map_`（已存在则 EMA 融合，否则插入）。
7. 移除距原点超过 `map_radius` 的栅格。
8. 发布 `TerrainAnalysis`（取自融合后的局部地图）与 `MarkerArray`，并打印每格详情与类型汇总日志。

---

## 依赖与环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Eigen3（`/usr/include/eigen3`）
- C++17，基于 colcon 的工作空间

`package.xml` 中声明的依赖：

```
rclcpp
sensor_msgs
std_msgs
visualization_msgs
rosidl_default_generators   # 构建工具
rosidl_default_runtime      # 运行依赖
```

---

## 编译

在工作空间根目录（本仓库中即包含 `terrain_analyzer/`、`build/`、`install/`、`log/` 的上级目录 `/home/daiyuxin/cod_terrain`）执行：

```bash
colcon build --packages-select terrain_analyzer
```

---

## 运行

每次运行前先 source 环境：

```bash
source install/setup.bash
```

### 1. ROS 2 节点

```bash
ros2 run terrain_analyzer terrain_analyzer_node
```

节点参数：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `pointcloud_topic` | string | `/livox/lidar` | 订阅的点云话题 |
| `min_points` | int | `30` | 单个栅格少于该点数则跳过；整帧有效点数少于该值也丢弃 |
| `cell_size` | double | `0.5` | 栅格边长（米） |
| `map_radius` | double | `5.0` | 局部地图保留半径（米，距原点） |

运行时可覆盖参数：

```bash
ros2 run terrain_analyzer terrain_analyzer_node \
  --ros-args -p pointcloud_topic:=/points -p min_points:=50 \
             -p cell_size:=0.4 -p map_radius:=8.0
```

**订阅话题**

- `pointcloud_topic` (`sensor_msgs/msg/PointCloud2`)，采用 `SensorDataQoS`。
- 要求点云包含 `float32` 类型的 `x`、`y`、`z` 字段；非有限值会被自动过滤。

**发布话题**

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/terrain_analysis` | `terrain_analyzer/msg/TerrainAnalysis` | 深度 10 | 当前局部地图全部栅格的分析结果 |
| `/terrain_markers` | `visualization_msgs/msg/MarkerArray` | 深度 10 | CUBE Marker，`frame_id = base_link`，按 `traversability` 着色（`r = 1-t`、`g = t`、`b = 0`），`scale.z = 0.05`，`alpha = 0.8` |


### 2. 离线测试 / 演示程序

用合成点云验证核心算法，仅链接 `terrain_analyzer_core` + Eigen，**不依赖 ROS**，可直接运行：

```bash
ros2 run terrain_analyzer test_terrain_analyzer
```

包含 4 个场景：水平面、30° 坡面、带高斯噪声的平面、平面 + 凸起异常点。

---

## 输出字段说明

### 核心结构体 `TerrainResult`

`include/terrain_analyzer/terrain_analyzer.hpp`：

| 字段 | 含义 |
| --- | --- |
| `lambda1` / `lambda2` / `lambda3` | 从大到小排列的 PCA 特征值 |
| `linearity` | 线状性 |
| `planarity` | 面状性 |
| `scattering` | 散乱性 |
| `normal` | 局部平面法向量（单位向量） |
| `slope` | 坡度（**弧度**，转角度需 `* 180 / M_PI`） |
| `mean_height` / `height_std` | z 的均值 / 标准差 |
| `residual_mean` / `residual_std` | 到拟合平面有符号距离的均值 / 标准差 |
| `outlier_ratio` | 残差大于 `mean + 3σ` 的点的比例 |
| `valid` | 是否计算成功 |

### 自定义消息 `TerrainAnalysis`

`msg/TerrainAnalysis.msg`（数组逐元素对应一个栅格）：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 继承输入点云的时间戳与坐标系 |
| `x` / `y` / `z` | `float64[]` | 栅格中心坐标（z 为融合后高度） |
| `lambda1` / `lambda2` / `lambda3` | `float64[]` | PCA 特征值 |
| `linearity` / `planarity` / `scattering` | `float64[]` | 几何特征 |
| `normal_x` / `normal_y` / `normal_z` | `float64[]` | 法向量分量 |
| `slope` | `float64[]` | 坡度（弧度） |
| `mean_height` / `height_std` | `float64[]` | 高度均值 / 标准差 |
| `residual_mean` / `residual_std` | `float64[]` | 平面残差均值 / 标准差 |
| `outlier_ratio` | `float64[]` | 异常点比例 |
| `terrain_type` | `uint8[]` | 地形类型码（见分类规则表） |
| `point_count` | `uint32[]` | 该栅格点数 |
| `confidence` | `float64[]` | 置信度（0~1，见「置信度与可通过性」） |
| `traversability` | `float64[]` | 可通过性（0~1） |
