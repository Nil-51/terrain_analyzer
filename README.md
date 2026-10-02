# terrain_analyzer

基于**协方差矩阵、PCA 主成分分析与高斯统计**的地形分析 ROS 2 功能包。

对一帧点云（或局部邻域点集）做平面拟合与几何特征提取，输出特征值、几何特征、法向量、坡度、高度统计与异常点比例，用于判断地形类型（平地 / 斜坡 / 起伏 / 杂乱 / 边缘）。

---

## 功能特性

- **PCA 几何特征**：由协方差矩阵特征分解得到线状性、面状性、散乱性。
- **局部平面法向量**：取最小特征值对应的特征向量作为法向量。
- **坡度**：由法向量与竖直方向的夹角计算。
- **高度统计**：均值与标准差。
- **平面残差统计**：点到拟合平面的有符号距离的均值 / 标准差。
- **高斯异常点比例**：残差超过 `mean + 3σ` 的点占比。
- **ROS 2 节点**：订阅 `sensor_msgs/PointCloud2`，实时输出分析结果。

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
    └── TerrainAnalysis.msg           # 自定义消息定义（尚未接入构建）
```

构建产物：

| 目标 | 类型 | 说明 |
| --- | --- | --- |
| `terrain_analyzer_core` | 静态库 | 纯算法实现，不依赖 ROS |
| `terrain_analyzer_node` | 可执行 | ROS 2 订阅节点 |
| `test_terrain_analyzer` | 可执行 | 使用合成数据的离线演示 |

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
5. **高度统计**：对 $z$ 分量求均值与标准差。
6. **平面残差**：$r_i = \mathbf{n}\cdot(p_i-\bar{p})$，统计均值与标准差。
7. **异常点比例**：残差 $r_i > \text{mean} + 3\sigma$ 的点所占比例。

> 输入点数少于 3 或特征分解失败时，结果 `valid = false`。

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
```
---

## 编译

在**工作空间根目录**（本仓库中即包含 `build/`、`install/`、`log/` 的上级目录）执行：

```bash
mkdir -p terrain && cd terrain
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
| `min_points` | int | `30` | 少于该点数则跳过该帧 |

运行时可覆盖参数：

```bash
ros2 run terrain_analyzer terrain_analyzer_node \
  --ros-args -p pointcloud_topic:=/points -p min_points:=50
```

**订阅话题**

- `pointcloud_topic` (`sensor_msgs/msg/PointCloud2`)，采用 `SensorDataQoS`。
- 要求点云包含 `float32` 类型的 `x`、`y`、`z` 字段；非有限值（NaN/Inf）会被自动过滤。

**输出**

节点不发布结果，仅通过日志打印（限流 1s）：

```
points=... | slope=.. deg | planarity=... | scattering=...
```

### 2. 离线测试 / 演示程序

无需 ROS 环境，用合成点云验证算法：

```bash
ros2 run terrain_analyzer test_terrain_analyzer
```

包含 4 个场景：水平面、30° 坡面、带高斯噪声的平面、平面 + 凸起异常点。

---

## 输出字段说明

`TerrainResult`（`include/terrain_analyzer/terrain_analyzer.hpp`）字段含义：

| 字段 | 含义 |
| --- | --- |
| `lambda1` / `lambda2` / `lambda3` | 从大到小排列的 PCA 特征值 |
| `linearity` | 线状性 |
| `planarity` | 面状性 |
| `scattering` | 散乱性 |
| `normal` | 局部平面法向量（单位向量） |
| `slope` | 坡度（**弧度**，转角度需 `* 180 / M_PI`） |
| `mean_height` / `height_std` | $z$ 的均值 / 标准差 |
| `residual_mean` / `residual_std` | 到拟合平面有符号距离的均值 / 标准差 |
| `outlier_ratio` | 残差大于 `mean + 3σ` 的点的比例 |
| `valid` | 是否计算成功 |


---



