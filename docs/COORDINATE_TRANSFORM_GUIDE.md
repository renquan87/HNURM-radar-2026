# 方案二（激光雷达方案）坐标变换完整指南

## 一、官方坐标系定义

根据 RoboMaster 2026 通信协议 V1.2.0（0x020B 表1-20）：

> 场地围挡在**红方补给站**附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向。

- 原点 (0, 0) = 红方补给站角落
- X+ = 向蓝方（场地长边方向，0~28m）
- Y+ = 向红方停机坪（场地短边方向，0~15m）
- Z+ = 垂直地面向上

**无论己方是红方还是蓝方，官方坐标系始终不变。**

0x0305 协议（雷达发送敌方坐标给选手端）的坐标单位是 cm，使用的就是这个官方坐标系。

---

## 二、涉及的四个坐标系

| 编号 | 坐标系名称 | 原点 | 轴向 | 说明 |
|------|-----------|------|------|------|
| 1 | 雷达实时坐标系 (livox) | 雷达当前位置 | X=前方, Y=左方, Z=上方 | 雷达固定不动，所以此坐标系不变 |
| 2 | PCD地图坐标系 (map) | 录制PCD时雷达位置 | 同上 | PCD文件中每个点的xyz就是相对于录制时雷达位置的坐标 |
| 3 | 世界坐标系 | 见下文 | X=右方, Y=上方, Z=垂直向上 | 米制坐标，用于 display_panel 画图和 judge_messager 发送 |
| 4 | 地图片坐标系 | 图片左上角 (0,0) | x=右方, y=下方 | 像素坐标，std_map.png 尺寸 2800×1500 |

### 关于"世界坐标系"

**这个项目中的"世界坐标系"不是固定的，它取决于 PCD 地图原点的设置。**

- 如果 PCD 原点设在红方角落 → 世界坐标 = 官方坐标
- 如果 PCD 原点设在蓝方角落 → 世界坐标 = "蓝方视角"坐标（蓝方角落为原点）

下面分别说明。

---

## 三、雷达物理位置

- **己方红方**：雷达在红方赛场短边（Y轴方向，15m边）边缘的中间位置
- **己方蓝方**：雷达在蓝方赛场短边（Y轴方向，15m边）边缘的中间位置
- 两个位置关于赛场中心对称，分别位于 x=0 和 x=28 的短边中点处

---

## 四、PCD 地图原点设置规则

根据学长的说法和代码逻辑：

> "从己方往前看右下角设为 (0, 0, 0)"

- **己方红方**：从红方往蓝方看，右下角 = 红方补给站角落 = 官方坐标原点 (0, 0)
- **己方蓝方**：从蓝方往红方看，右下角 = 蓝方补给站角落 = 官方坐标 (28, 15)

**赛场结构对称，所以同一份 PCD 地图两方都能用。** ICP 配准会自动处理旋转对齐。

---

## 五、完整坐标变换链路

```
相机检测框 → 点云投影 → 聚类中心 (相机坐标系)
    → [外参矩阵] → lidar_center (雷达坐标系 livox)
    → [ICP TF: livox→map] → field_xyz (PCD地图坐标系)
    → [发布到 /location 话题]
    → [EKF 滤波] → /ekf_location_filtered
    → display_panel (画图) + judge_messager (发送给裁判系统)
```

---

## 六、己方红方 (my_color = "Red") 的完整流程

### 6.1 PCD 原点

PCD 原点设在红方补给站角落 = 官方坐标原点 (0, 0, 0)。

### 6.2 radar.py 输出

```python
# 相机坐标 → 雷达坐标
lidar_center = extrinsic_matrix @ camera_center

# 雷达坐标 → PCD地图坐标（通过 ICP TF）
field_xyz = radar_to_field @ lidar_center  # radar_to_field 是 livox→map 的 TF
```

因为 PCD 原点 = 官方原点，所以 **field_xyz 直接就是官方坐标**。

例如：一个蓝方机器人在赛场中央，官方坐标约 (14, 7.5)，field_xyz ≈ (14, 7.5, 0)。

### 6.3 radar.py 过滤

```python
# my_color = "Red"，过滤掉己方（红方）机器人
if self.global_my_color == "Red" and car_id < 100:  # 红方 car_id < 100
    continue  # 跳过
```

所以只发布**蓝方**（敌方）机器人。`loc.label = "Blue"`。

### 6.4 display_panel.py 画图

```python
x = location.x  # 例如 14.0
y = location.y  # 例如 7.5
xx = int(x * 100)      # 1400
yy = int(1500 - y * 100)  # 750

if location.label == 'Red':  # 不进入此分支，因为 label = "Blue"
    ...
else:
    # 蓝方机器人，直接画
    cv2.circle(show_map, (xx, yy), ...)  # 画在 (1400, 750)
```

**结果**：蓝方机器人画在地图正确位置。

地图图片 std_map.png 的布局：
- 左侧 (px=0) = x=0 = 红方角落
- 右侧 (px=2800) = x=28 = 蓝方角落
- 顶部 (py=0) = y=15
- 底部 (py=1500) = y=0

### 6.5 judge_messager.py 发送坐标

```python
if location.label == 'Red':  # 不进入此分支
    ...
else:  # label = "Blue"，进入此分支
    robot_loc[robot_trans[location.id]] = [x, y]  # 直接使用 field_xyz
```

然后发送：
```python
x_cm = int(x * 100)  # 转为 cm
y_cm = int(y * 100)
# 通过 0x0305 协议发送给裁判系统
```

**结果**：直接发送官方坐标（cm），正确。

### 6.6 红方总结

| 步骤 | 坐标值 | 坐标系 |
|------|--------|--------|
| field_xyz | (14.0, 7.5) | 官方坐标（PCD原点=红方角落） |
| display_panel 像素 | (1400, 750) | 图片像素，直接画 |
| judge_messager 发送 | (1400, 750) cm | 官方坐标 cm，直接发 |

---

## 七、己方蓝方 (my_color = "Blue") 的完整流程

### 7.1 PCD 原点

PCD 原点设在蓝方补给站角落 = 官方坐标 (28, 15)。

但在 PCD 坐标系中，这个点是 (0, 0, 0)。

### 7.2 radar.py 输出

同样的计算：
```python
field_xyz = radar_to_field @ lidar_center
```

因为 PCD 原点 = 蓝方角落，所以 **field_xyz 是"蓝方视角"坐标**：
- field_xyz 的 x=0 对应蓝方角落（官方 x=28）
- field_xyz 的 x=28 对应红方角落（官方 x=0）
- field_xyz 的 y=0 对应蓝方角落（官方 y=15）
- field_xyz 的 y=15 对应红方角落（官方 y=0）

例如：一个红方机器人在赛场中央，官方坐标约 (14, 7.5)。
从蓝方视角看，它在 field_xyz ≈ (14, 7.5)（因为赛场中央到两边距离相等）。

但如果红方机器人在红方角落附近，官方坐标约 (2, 2)：
- 从蓝方视角：field_xyz ≈ (26, 13)（因为 28-2=26, 15-2=13）

### 7.3 radar.py 过滤

```python
# my_color = "Blue"，过滤掉己方（蓝方）机器人
if self.global_my_color == "Blue" and car_id > 100:  # 蓝方 car_id > 100
    continue  # 跳过
```

所以只发布**红方**（敌方）机器人。`loc.label = "Red"`。

### 7.4 display_panel.py 画图

```python
x = location.x  # 例如 26.0（蓝方视角坐标）
y = location.y  # 例如 13.0
xx = int(x * 100)      # 2600
yy = int(1500 - y * 100)  # 200

if location.label == 'Red':  # 进入此分支！
    x = 28 - x        # 28 - 26 = 2（转为官方坐标）← 但被下一行覆盖了
    y = 15 - y        # 15 - 13 = 2 ← 但被下一行覆盖了
    x = round(location.x, 2)  # 覆盖回 26.0（这是个 bug，文字标注显示原始值）
    y = round(location.y, 2)  # 覆盖回 13.0
    xx = 2800 - xx     # 2800 - 2600 = 200
    yy = 1500 - yy     # 1500 - 200 = 1300
    cv2.circle(show_map, (xx, yy), ...)  # 画在 (200, 1300)
```

像素 (200, 1300) 对应地图左下方 = 红方角落附近 ✓

**结果**：红方机器人画在地图正确位置（通过像素镜像实现）。

### 7.5 judge_messager.py 发送坐标

```python
if location.label == 'Red':  # 进入此分支！
    # 蓝方视角坐标 → 官方坐标
    x = 28 - x  # 28 - 26 = 2
    y = 15 - y  # 15 - 13 = 2
    robot_loc[location.id] = [x, y]  # [2, 2] = 官方坐标
```

然后发送：
```python
x_cm = int(2 * 100)  # 200 cm
y_cm = int(2 * 100)  # 200 cm
# 通过 0x0305 协议发送给裁判系统
```

**结果**：转换为官方坐标后发送，正确。

### 7.6 蓝方总结

| 步骤 | 坐标值 | 坐标系 |
|------|--------|--------|
| field_xyz | (26.0, 13.0) | 蓝方视角坐标（PCD原点=蓝方角落） |
| display_panel 像素 | (200, 1300) | 图片像素，经过镜像 |
| judge_messager 发送 | (200, 200) cm | 官方坐标 cm，经过 28-x, 15-y 转换 |

---

## 八、为什么同一份 PCD 两方都能用

赛场结构关于中心点对称。从红方角落看到的点云，和从蓝方角落看到的点云，经过 180° 旋转后完全一致。

实际操作：
1. 用官方赛场 CAD 模型导出一份 PCD 点云地图
2. 把 PCD 原点设在红方角落（官方原点）
3. 不管 my_color 是什么，都用这份 PCD
4. ICP 配准时，如果雷达在蓝方，ICP 的 TF 会自动包含一个约 180° 的旋转，使得实时点云和 PCD 对齐
5. 对齐后 field_xyz 就是相对于 PCD 原点的坐标

但这里有个微妙之处：如果 PCD 原点在红方角落，那么不管雷达在哪一方，field_xyz 都是官方坐标。这种情况下 judge_messager 的 `28-x, 15-y` 转换就是**错误的**。

**所以代码的设计意图是：PCD 原点在己方角落，不是固定在红方角落。**

- 红方：己方角落 = 红方角落 = 官方原点 → field_xyz = 官方坐标 → 直接发
- 蓝方：己方角落 = 蓝方角落 ≠ 官方原点 → field_xyz = 蓝方视角坐标 → 需要 28-x, 15-y

---

## 九、实验室测试场景

实验室没有标准赛场，PCD 原点是你标定时手动设置的。

### 关键问题

实验室测试时，`my_color` 的设置决定了 display_panel 和 judge_messager 是否会做 `28-x, 15-y` 翻转：

- 如果 `my_color = "Red"`：代码认为 field_xyz 已经是官方坐标，**不做翻转**
- 如果 `my_color = "Blue"`：代码认为 field_xyz 是蓝方视角坐标，**会做 28-x, 15-y 翻转**

### 实验室建议

1. 把 `my_color` 设为 `"Red"`
2. PCD 原点设在你标定的原点位置
3. 这样 field_xyz 就直接是相对于你标定原点的坐标，不会被翻转
4. display_panel 和 judge_messager 都直接使用原始坐标

---

## 十、display_panel.py 中的一个 bug

在蓝方分支中：

```python
if location.label == 'Red':
    x = 28 - location.x    # 转换坐标
    y = 15 - location.y
    x = round(location.x, 2)  # ← bug：又覆盖回原始值了
    y = round(location.y, 2)  # ← bug
```

这导致文字标注显示的是蓝方视角坐标而非官方坐标。圆点位置是正确的（用的是 `xx = 2800 - xx`），但文字标注的数值是错的。

---

## 十一、总结：你需要做什么

### 赛场环境

| 己方颜色 | PCD 原点位置 | field_xyz 含义 | judge_messager 处理 | display_panel 处理 |
|---------|-------------|---------------|--------------------|--------------------|
| Red | 红方补给站角落 (官方原点) | 官方坐标 | 直接发送 | 直接画 |
| Blue | 蓝方补给站角落 | 蓝方视角坐标 | 28-x, 15-y 转官方 | 像素镜像 |

### 实验室环境

把 `my_color` 设为 `"Red"`，PCD 原点设在你标定的原点，坐标不翻转，直接对比即可。

---

# 第二部分：基于源代码的坐标变换精确分析与误差溯源

> **以下内容完全基于实际代码分析，不依赖任何说明性文档。**
> 所有公式中使用的符号均在首次出现时给出解释。

## 十二、符号约定

为了清晰表达坐标变换关系，本文采用以下符号约定：

| 符号 | 含义 |
|------|------|
| $\mathbf{p}^{A}$ | 某点在坐标系 $A$ 下的坐标列向量，形如 $[x, y, z, 1]^T$（齐次）或 $[x, y, z]^T$（非齐次） |
| $\mathbf{T}_{A \leftarrow B}$ | 将坐标系 $B$ 下的点变换到坐标系 $A$ 下的 $4 \times 4$ 齐次变换矩阵，即 $\mathbf{p}^{A} = \mathbf{T}_{A \leftarrow B} \cdot \mathbf{p}^{B}$ |
| $\mathbf{R}$ | $3 \times 3$ 旋转矩阵（正交矩阵，$\mathbf{R}^T \mathbf{R} = \mathbf{I}$，$\det(\mathbf{R}) = 1$） |
| $\mathbf{t}$ | $3 \times 1$ 平移向量 |
| $\mathbf{K}$ | $3 \times 3$ 相机内参矩阵 |

**齐次变换矩阵**的一般形式为：

$$
\mathbf{T}_{A \leftarrow B} = \begin{bmatrix} \mathbf{R}_{A \leftarrow B} & \mathbf{t}_{A \leftarrow B} \\ \mathbf{0}^T & 1 \end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

其含义是：如果一个点在坐标系 $B$ 中的齐次坐标为 $\mathbf{p}^{B} = [x_B, y_B, z_B, 1]^T$，那么该点在坐标系 $A$ 中的齐次坐标为：

$$
\mathbf{p}^{A} = \mathbf{T}_{A \leftarrow B} \cdot \mathbf{p}^{B}
$$

**逆变换**：$\mathbf{T}_{B \leftarrow A} = \mathbf{T}_{A \leftarrow B}^{-1}$。

## 十三、本系统涉及的六个坐标系

根据代码分析，完整管线涉及以下坐标系：

| 编号 | 坐标系 | 缩写 | 原点 | 说明 |
|------|--------|------|------|------|
| ① | **图像像素坐标系** | $I$ | 图像左上角 | 单位：像素 (px)；$u$ 向右，$v$ 向下 |
| ② | **相机坐标系** | $C$ | 相机光心 | 单位：米；$Z$ 轴沿光轴向前 |
| ③ | **激光雷达坐标系** | $L$（frame_id: `livox`） | 雷达几何中心 | 实时坐标系；$X$ 前方，$Y$ 左方，$Z$ 上方 |
| ④ | **PCD 地图坐标系** | $M$（frame_id: `map`） | 点云地图原点 | 录制并经 `align_pcd_with_map.py` 变换后的坐标系 |
| ⑤ | **世界坐标系** | $W$ | 地图左下角 $(0,0)$ | 经对齐后与 $M$ 重合；单位：米 |
| ⑥ | **显示像素坐标系** | $D$ | `std_map.png` 图片左上角 | 单位：像素；$x_{px}$ 向右，$y_{px}$ 向下 |

**关键等价关系**：经过 `align_pcd_with_map.py` 对齐后，PCD 地图坐标系 $M$ 与世界坐标系 $W$ 重合（即 $\mathbf{T}_{W \leftarrow M} = \mathbf{I}$）。因此在后续分析中，$M$ 和 $W$ 可视为同一坐标系。

## 十四、完整坐标变换管线（以一个机器人为例）

### 14.1 端到端数据流总览

```
海康相机图像帧
   │
   ▼ ① YOLO 三阶段推理 (detector_node.py)
   │    输出: 2D 检测框 (xyxy_box) + 分类标签 (label) + 追踪 ID (track_id)
   │
   │    上游点云链路补充：`lidar_node.py` 当前订阅 `/livox/lidar` 后会同时发布
   │    `/lidar_pcds`（原始累积点云，供 `registration` 计算 TF）和
   │    `/target_pointcloud`（背景减除后的前景点云，供 `radar.py` 使用）
   │
   ▼ ② 点云投影到相机坐标系 (Converter.lidar_to_camera)
   │    输入: `radar.py` 当前订阅到的实时前景点云 p^L（来源 `/target_pointcloud`，坐标系仍为 `livox`）
   │    操作: p^C = T_{C←L} · p^L
   │    输出: 相机坐标系下的点云 p^C
   │
   ▼ ③ 提取检测框内点云 (Converter.get_points_in_box)
   │    操作: p^C → 投影到像素坐标 (u,v) = K · p^C / z → 筛选落在 bbox 内的点
   │    输出: 检测框内的 3D 点云子集 (仍在相机坐标系 C 下)
   │
   ▼ ④ 滤波 + 聚类 (Converter.filter + Converter.cluster)
   │    操作: 体素降采样 → DBSCAN 聚类 → 取最大簇中心
   │    输出: center^C = [x_c, y_c, z_c] (相机坐标系下的目标中心)
   │
   ▼ ⑤ 相机坐标系 → 雷达坐标系 (radar.py L336-L340)
   │    操作: p^L = T_{C←L}^{-1} · p^C  即 extrinsic_matrix_inv · center^C
   │    输出: lidar_center^L = [x_l, y_l, z_l]
   │
   ▼ ⑥ 雷达坐标系 → 地图坐标系 (radar.py L344-L346)
   │    操作: p^M = T_{M←L} · p^L  即 radar_to_field · lidar_center^L
   │    T_{M←L} 来自 registration 节点的实时 ICP 配准
   │    输出: field_xyz = [x_m, y_m, z_m] (地图/世界坐标系)
   │
   ▼ ⑦ 发布到 /location 话题 (radar.py L393)
   │
   ▼ ⑧ EKF 滤波 (ekf_node.py)
   │    状态向量: [x, v_x, y, v_y]
   │    输出: 平滑后的坐标 → /ekf_location_filtered
   │
   ▼ ⑨ 世界坐标 → 显示像素 (display_panel.py)
        操作: x_px = x_m × 100,  y_px = 1500 − y_m × 100
```

### 14.2 各步骤的精确数学表达

#### 步骤 ②：激光雷达坐标系 → 相机坐标系

**代码位置**：`Converter.lidar_to_camera()` ([Converter.py](../src/hnurm_radar/hnurm_radar/Lidar/Converter.py#L173-L185))

**变换矩阵**：外参矩阵 $\mathbf{T}_{C \leftarrow L}$，由 `converter_config.yaml` 中的 `R` 和 `T` 构成：

$$
\mathbf{T}_{C \leftarrow L} = \begin{bmatrix} \mathbf{R}_{C \leftarrow L} & \mathbf{t}_{C \leftarrow L} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

当前标定值（来自 `converter_config.yaml`）：

$$
\mathbf{R}_{C \leftarrow L} = \begin{bmatrix} -0.00285 & -1.00000 & 0.00113 \\ -0.00663 & -0.00111 & -0.99998 \\ 0.99997 & -0.00285 & -0.00662 \end{bmatrix}, \quad
\mathbf{t}_{C \leftarrow L} = \begin{bmatrix} 0.00765 \\ 0.06018 \\ 0.00000 \end{bmatrix}
$$

> **物理含义**：$\mathbf{R}$ 近似表示 LiDAR 的 $X$(前) 映射到 Camera 的 $Z$(深度)，LiDAR 的 $Y$(左) 映射到 Camera 的 $-X$(右)，LiDAR 的 $Z$(上) 映射到 Camera 的 $-Y$(下)。$\mathbf{t}$ 为雷达光心到相机光心的平移。

**代码实现**（注意：代码使用行向量 × 矩阵转置的形式）：

```python
# Converter.lidar_to_camera()  —— 实际代码
pc = cp.hstack((pc, np.ones((pc.shape[0], 1))))   # N×4 齐次化
pc = cp.dot(pc, self.extrinsic_matrix.T)           # N×4 · 4×4 = N×4
pc = pc[:, :3]                                      # 取前三列
```

**数学等价性**：设点云矩阵 $\mathbf{P}^L$ 为 $N \times 4$ 矩阵（每行一个齐次点），则：

$$
\mathbf{P}^C = \mathbf{P}^L \cdot \mathbf{T}_{C \leftarrow L}^T
$$

这等价于对每个列向量 $\mathbf{p}^L_i$：

$$
(\mathbf{p}^C_i)^T = (\mathbf{p}^L_i)^T \cdot \mathbf{T}_{C \leftarrow L}^T = (\mathbf{T}_{C \leftarrow L} \cdot \mathbf{p}^L_i)^T
$$

即 $\mathbf{p}^C_i = \mathbf{T}_{C \leftarrow L} \cdot \mathbf{p}^L_i$，与标准的列向量左乘变换矩阵完全一致。✓

#### 步骤 ③：相机坐标系 → 图像像素坐标（用于提取框内点云）

**代码位置**：`Converter.camera_to_image()` ([Converter.py](../src/hnurm_radar/hnurm_radar/Lidar/Converter.py#L213-L234)) 和 `Converter.get_points_in_box()` ([Converter.py](../src/hnurm_radar/hnurm_radar/Lidar/Converter.py#L326-L360))

**内参矩阵** $\mathbf{K}$：

$$
\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 3446.07 & 0 & 1505.65 \\ 0 & 3435.47 & 1068.52 \\ 0 & 0 & 1 \end{bmatrix}
$$

其中 $f_x, f_y$ 是焦距（单位：像素），$(c_x, c_y)$ 是主点坐标。

**投影公式**：对于相机坐标系中的点 $\mathbf{p}^C = [X_c, Y_c, Z_c]^T$，其在图像上的像素坐标 $(u, v)$ 为：

$$
\begin{bmatrix} u \cdot Z_c \\ v \cdot Z_c \\ Z_c \end{bmatrix} = \mathbf{K} \cdot \mathbf{p}^C = \begin{bmatrix} f_x X_c + c_x Z_c \\ f_y Y_c + c_y Z_c \\ Z_c \end{bmatrix}
$$

$$
\Rightarrow u = \frac{f_x X_c}{Z_c} + c_x, \quad v = \frac{f_y Y_c}{Z_c} + c_y
$$

**代码实现**：

```python
# Converter.camera_to_image()
xyz = cp.dot(pc, self.intrinsic_matrix.T)  # N×3 · 3×3 = N×3
uvz[:, 0] = xyz[:, 0] / xyz[:, 2]  # u = (fx*X + cx*Z) / Z
uvz[:, 1] = xyz[:, 1] / xyz[:, 2]  # v = (fy*Y + cy*Z) / Z
uvz[:, 2] = xyz[:, 2]               # depth = Z
```

然后 `get_points_in_box()` 筛选 $u \in [u_{min}, u_{max}]$，$v \in [v_{min}, v_{max}]$ 的点，返回的是**相机坐标系下**的 3D 点（不是像素坐标）。

> **⚠️ 重要细节**：投影过程**没有应用畸变校正**（`distortion_matrix` 仅在 `camera_to_field_init` 的 `solvePnP` 中使用）。这意味着越靠近图像边缘的点，投影误差越大。

#### 步骤 ④：滤波 + DBSCAN 聚类

**代码位置**：`Converter.filter()` ([Converter.py](../src/hnurm_radar/hnurm_radar/Lidar/Converter.py#L431-L439)) 和 `Converter.cluster()` ([Converter.py](../src/hnurm_radar/hnurm_radar/Lidar/Converter.py#L399-L427))

> **当前链路前提**：进入这一步的点云已经不是 `/lidar_pcds` 原始累积点云，而是 `lidar_node.py` 发布的 `/target_pointcloud` 前景点云；背景减除在上游 `lidar_node.py` 中完成，但这里仍继续执行 `Converter.filter()` 与 `Converter.cluster()`。

1. **体素降采样**：`voxel_size = 0.035m`，将点云网格化以减少计算量
2. **DBSCAN 聚类**：`eps = 0.40m`，`min_points = 10`
   - 找到所有密度可达的点簇
   - 取**最大簇**的所有点
   - 计算该簇的**质心**（所有点的均值）作为目标中心 $\mathbf{center}^C$

$$
\mathbf{center}^C = \frac{1}{|\mathcal{S}|} \sum_{i \in \mathcal{S}} \mathbf{p}^C_i
$$

其中 $\mathcal{S}$ 是最大簇的点集。

3. **备选方案**：如果聚类结果全为零，退化为取距离中值点 `get_center_mid_distance()`

#### 步骤 ⑤：相机坐标系 → 激光雷达坐标系

**代码位置**：`radar.py` [L333-L340](../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py#L333-L340)

```python
center = np.hstack((center, np.array((1))))  # 齐次化 [x,y,z] → [x,y,z,1]
center = center[:3]
center = np.hstack((center, np.array((1))))  # 再次齐次化 (代码冗余，无害)
lidar_center = np.dot(self.extrinsic_matrix_inv, center)  # 左乘逆外参
lidar_center = lidar_center[:3]
```

**数学表达**：

$$
\mathbf{p}^L = \mathbf{T}_{C \leftarrow L}^{-1} \cdot \mathbf{p}^C = \mathbf{T}_{L \leftarrow C} \cdot \mathbf{p}^C
$$

其中 `extrinsic_matrix_inv` = $\mathbf{T}_{C \leftarrow L}^{-1} = \mathbf{T}_{L \leftarrow C}$。

> **注意**：这里使用的是**列向量左乘**形式 (`np.dot(matrix, vector)`)，与步骤②的行向量右乘转置形式**等价**但**写法不同**。

#### 步骤 ⑥：激光雷达坐标系 → 地图/世界坐标系（ICP 配准 TF）

**代码位置**：`radar.py` [L343-L346](../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py#L343-L346)

```python
lidar_center = np.hstack((lidar_center, np.array((1))))  # 齐次化
field_xyz = np.dot(self.radar_to_field, lidar_center)     # 左乘 TF 矩阵
field_xyz = field_xyz[:3]
```

**数学表达**：

$$
\mathbf{p}^M = \mathbf{T}_{M \leftarrow L} \cdot \mathbf{p}^L
$$

**$\mathbf{T}_{M \leftarrow L}$ 的来源**——这是整个管线中最复杂的变换，它由 `registration` 节点的 ICP 配准实时计算：

1. `registration_node.cpp` 订阅 `/lidar_pcds`（原始累积点云，`livox` 坐标系）
2. 加载预录制的 PCD 地图（已通过 `align_pcd_with_map.py` 变换到世界坐标系 $M$）
3. 使用 **Quatro++**（粗配准）+ **small_gicp**（精配准/GICP）算法，求解实时点云到地图的变换
4. 输出 `T_target_source`，即 $\mathbf{T}_{M \leftarrow L}$
5. 以 TF 广播（`map` → `livox`），频率 ~100Hz

> **链路区分**：`registration` 的输入仍然是 `/lidar_pcds`，而 `radar.py` 的输入已经切换为 `/target_pointcloud`。两者都处于 `livox` 坐标系，因此 `registration` 给出的 `map ← livox` TF 可以直接作用在 `radar.py` 处理得到的 `lidar_center` 上。

**TF 获取代码** (`radar.py` [L132-L148](../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py#L132-L148))：

```python
transform = self.tf_buffer.lookup_transform(
    target_frame='map',       # 目标坐标系
    source_frame='livox',     # 源坐标系
    time=rclpy.time.Time()    # 最新可用
)
# 将 TF 转为 4×4 矩阵
transform_matrix = self.tf_to_matrix(translation, rotation)
self.radar_to_field = transform_matrix  # = T_{map←livox}
```

> **⚠️ TF 方向说明**：ROS2 的 `lookup_transform(target, source)` 返回的变换含义是：将 `source` 坐标系中的点变换到 `target` 坐标系。所以 `lookup_transform('map', 'livox')` 返回 $\mathbf{T}_{map \leftarrow livox}$，即 $\mathbf{T}_{M \leftarrow L}$。✓

#### 步骤 ⑦⑧：EKF 滤波

**代码位置**：`ekf_node.py` ([ekf_node.py](../src/ekf/ekf/ekf_node.py))

EKF 以 50ms 间隔（20Hz）执行，状态向量为 $\mathbf{x} = [x, v_x, y, v_y]^T$，状态转移矩阵：

$$
\mathbf{F} = \begin{bmatrix} 1 & \Delta t & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & \Delta t \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

**EKF 参数**（`ekf_node.py` L141）：`pval=0.001, qval=1e-4, rval=0.0005, interval=0.05`

- $q$ = 过程噪声协方差，值越小表示越信任运动模型
- $r$ = 观测噪声协方差，值越小表示越信任观测值
- 当前 $r = 0.0005$ **非常小**，意味着 EKF 几乎完全信任输入的观测值，滤波效果弱

> **EKF 不改变坐标系**，它只是对世界坐标 $(x_m, y_m)$ 做时序平滑。

#### 步骤 ⑨：世界坐标 → 显示像素坐标

**代码位置**：`display_panel.py` ([display_panel.py](../src/hnurm_radar/hnurm_radar/shared/display_panel.py#L63-L70))

$$
x_{px} = \lfloor x_m \times 100 \rfloor, \quad y_{px} = \lfloor 1500 - y_m \times 100 \rfloor
$$

底图 `std_map.png` 为 $2800 \times 1500$ 像素，对应 $28\text{m} \times 15\text{m}$，即 $100 \text{px/m}$。

### 14.3 完整变换链的矩阵表达

将上述所有步骤串联，一个机器人从相机坐标系中的聚类中心到最终世界坐标的完整变换为：

$$
\boxed{
\mathbf{p}^W = \mathbf{T}_{M \leftarrow L} \cdot \mathbf{T}_{L \leftarrow C} \cdot \mathbf{p}^C
= \mathbf{T}_{M \leftarrow L} \cdot \mathbf{T}_{C \leftarrow L}^{-1} \cdot \mathbf{p}^C
}
$$

展开为三个独立阶段：

| 阶段 | 变换 | 矩阵名称 | 来源 | 代码中的变量 |
|------|------|----------|------|-------------|
| 相机→雷达 | $\mathbf{T}_{L \leftarrow C} = \mathbf{T}_{C \leftarrow L}^{-1}$ | 外参逆矩阵 | 离线标定 (`converter_config.yaml`) | `self.extrinsic_matrix_inv` |
| 雷达→地图 | $\mathbf{T}_{M \leftarrow L}$ | ICP 配准 TF | 实时计算 (registration 节点) | `self.radar_to_field` |
| 地图≡世界 | $\mathbf{I}_{4 \times 4}$ | 恒等 | `align_pcd_with_map.py` 已对齐 | (隐含) |

### 14.4 数值实例：你的 (3,3) 测试点

**已知**：
- 机器人真实世界坐标（手动标定）：$(11.39, 7.67)$
- 系统输出的典型值（`/location` 话题）：约 $(11.97, 6.49)$

**误差**：
- $\Delta x = 11.97 - 11.39 = +0.58\text{m}$
- $\Delta y = 6.49 - 7.67 = -1.18\text{m}$
- 欧氏距离误差：$\sqrt{0.58^2 + 1.18^2} \approx 1.31\text{m}$

**误差特征分析**：
1. **系统性偏移**：$x$ 偏大约 0.6m，$y$ 偏小约 1.2m，这不是随机噪声，而是系统性误差
2. **漂移**（机器人静止时坐标变化）：观测数据中 $x$ 在 $10.39 \sim 13.63$ 之间跳变，$y$ 在 $6.08 \sim 6.92$ 之间跳变，范围约 $3.2\text{m} \times 0.8\text{m}$
3. **偶发跳变**：出现 $(10.42, 6.08)$ 和 $(13.56, 6.89)$ 等远离中心的异常值

### 14.5 PCD 地图对齐流程详解（`align_pcd_with_map.py`）

这是建立 $\mathbf{T}_{M \leftarrow L_0}$（录制时雷达坐标系 $L_0$ → 世界坐标系 $M$）的关键步骤。

**输入**：原始 PCD 点云（$L_0$ 坐标系）+ 地图图片（世界坐标系）

**操作**：用户手动对齐点云俯视图与地图，工具计算：
- 雷达在世界坐标系中的位置 `(radar_x, radar_y)`
- 旋转角 `yaw_deg`
- 雷达离地高度 `radar_h`

**变换公式**（`apply_transform()` 函数）：

$$
\begin{bmatrix} x_M \\ y_M \\ z_M \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_{L_0} \\ y_{L_0} \\ z_{L_0} \end{bmatrix} + \begin{bmatrix} r_x \\ r_y \\ r_h \end{bmatrix}
$$

当前实验室标定结果（`lab_pcd_transform_info.json`）：

$$
r_x = 3.703, \quad r_y = 4.387, \quad r_h = 1.05, \quad \theta = 12.9°
$$

这意味着：**雷达（录制时）在世界坐标系中位于 $(3.703, 4.387, 1.05)$，朝向偏离世界 $X$ 轴 $12.9°$。**

> **⚠️ 关键理解**：这个变换**只对 PCD 地图文件本身做了一次性变换**（把 PCD 从雷达坐标系旋转平移到世界坐标系）。之后 `registration` 节点加载的就是已经在世界坐标系下的 PCD 地图。ICP 配准直接给出 `livox` → `map`(世界) 的 TF。

---

## 十五、坐标不准确的所有可能原因分析

根据上述完整变换链 $\mathbf{p}^W = \mathbf{T}_{M \leftarrow L} \cdot \mathbf{T}_{C \leftarrow L}^{-1} \cdot \mathbf{p}^C$，误差可能来自以下**每一个环节**。下面逐一分析。

### 原因 1：相机内参标定不准确

#### What（原理）

相机内参矩阵 $\mathbf{K}$ 描述了相机的光学特性。它负责在步骤③中将 3D 相机坐标投影到 2D 像素坐标：

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} \sim \mathbf{K} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}
$$

如果 $f_x, f_y, c_x, c_y$ 不准确，投影位置就会偏移。当 YOLO 检测到一个 bbox 后，代码需要用 $\mathbf{K}$ 把相机坐标系的点投影到像素平面来判断哪些点落在 bbox 内。如果 $\mathbf{K}$ 有误，**被选中的点云子集就不是真正对应机器人的那些点**。

此外，**畸变系数** $[k_1, k_2, p_1, p_2, k_3]$ 当前值为 `[-0.107, 0.202, -0.001, 0.000, 0.000]`，代码在 `camera_to_image()` 中**没有应用畸变校正**，这意味着图像边缘区域的投影误差更大。

#### Why（为什么怀疑）

- 内参标定通常用棋盘格方法，精度取决于棋盘格数量、角度覆盖度和图像质量
- 你提到进行了新的标定，新标定值可能与之前不同
- 畸变系数 $k_1 = -0.107$ 不算很小，图像边缘的径向畸变可达数十像素

#### 如何判断（验证方法）

1. **重投影误差检查**：
   - 用标定工具（如 OpenCV `calibrateCamera`）输出的重投影误差（RMS reprojection error）
   - **合格标准**：RMS < 0.5 px（良好），< 1.0 px（可接受），> 2.0 px（需重新标定）
   
2. **直线测试**：
   - 拍摄包含直线的场景（如门框、墙壁边缘）
   - 使用当前内参去畸变：`cv2.undistort(img, K, distortion)`
   - 如果去畸变后直线仍然弯曲，说明内参不准

3. **棋盘格角点验证**：
   - 拍摄棋盘格（不要用标定时的图片，用新拍摄的）
   - 用 `cv2.findChessboardCorners` 检测角点
   - 用 `cv2.projectPoints` 和当前内参反向投影，计算误差

#### How（解决方法）

1. 使用 `/data/projects/radar/lidar_camera_calib_utils` 中的内参标定工具重新标定
2. 确保采集 **≥20 张**不同角度和距离的棋盘格图片
3. 棋盘格应覆盖图像的**四个角落和中央**，而非只在中间
4. 验证标定后的重投影误差
5. 在 `Converter.camera_to_image()` 中增加畸变校正：
   ```python
   # 在投影前对点做反畸变（或对bbox坐标做畸变），减少边缘区域误差
   ```

---

### 原因 2：相机-激光雷达外参标定不准确

#### What（原理）

外参矩阵 $\mathbf{T}_{C \leftarrow L}$ 描述了激光雷达坐标系与相机坐标系之间的刚体变换关系。它出现在两个关键步骤中：

- **步骤②**：$\mathbf{p}^C = \mathbf{T}_{C \leftarrow L} \cdot \mathbf{p}^L$（雷达点云转到相机系，用于投影）
- **步骤⑤**：$\mathbf{p}^L = \mathbf{T}_{C \leftarrow L}^{-1} \cdot \mathbf{p}^C$（聚类中心转回雷达系）

外参有 6 个自由度（3 个旋转 + 3 个平移）。**任何一个参数的微小偏差都会导致两个传感器"看到的世界"产生错位**。

**定量影响估算**：
- 旋转误差 $\delta\theta = 1°$ 时，在距离 $d = 10\text{m}$ 处产生的位置偏差约 $d \cdot \tan(\delta\theta) \approx 0.17\text{m}$
- 平移误差 $\delta t = 0.05\text{m}$ 直接叠加到最终坐标上

#### Why（为什么怀疑）

- 你的误差中 $\Delta y = -1.18\text{m}$ 较大且具有系统性，符合外参旋转角偏差的特征
- 外参标定是离线进行的，如果标定后相机-雷达的相对位置发生了任何变化（如螺丝松动、碰撞），外参就会失效
- 当前外参 $\mathbf{t} = [0.008, 0.060, 0.000]^T$，平移量级为厘米级，如果实际相机-雷达间距更大，说明标定有偏差

#### 如何判断（验证方法）

1. **视觉叠加法**（最直观）：
   - 将点云用外参投影到图像上：对每个雷达点 $\mathbf{p}^L$，计算 $\mathbf{K} \cdot (\mathbf{T}_{C \leftarrow L} \cdot \mathbf{p}^L)[:3]$
   - 将投影点绘制在相机图像上（按深度着色）
   - **如果点云轮廓与图像中物体边缘精确对齐**，则外参正确
   - **如果有系统性偏移**（如全部偏右偏下），则外参不准

2. **特征点对应法**：
   - 在场景中放置容易识别的标志物（如角锥反射器）
   - 在图像中标记像素位置，在点云中标记对应 3D 位置
   - 计算重投影误差

#### How（解决方法）

1. 使用 `/data/projects/radar/direct_visual_lidar_calibration_ws` 中的外参标定工具重新标定
2. 确保标定时**固定好相机和激光雷达**，标定后不要移动
3. 标定环境应包含丰富的**几何特征**（直角墙壁、柱子等），避免在空旷场地标定
4. 标定后用上述视觉叠加法验证

---

### 原因 3：ICP 点云配准（`livox` → `map` TF）不准确或不稳定

#### What（原理）

ICP（Iterative Closest Point）配准的目标是找到最优的刚体变换 $\mathbf{T}_{M \leftarrow L}$，使得实时点云 $\{p^L_i\}$ 变换后与地图点云 $\{q^M_j\}$ 的距离最小：

$$
\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_i \| \mathbf{T} \cdot \mathbf{p}^L_i - \mathbf{q}^M_{nn(i)} \|^2
$$

其中 $nn(i)$ 表示变换后距离最近的地图点。

**本系统使用的配准流程**：
1. **Quatro++**（粗配准）：基于 FPFH 特征匹配 + TEASER++ 鲁棒估计，给出初始位姿
2. **small_gicp**（精配准）：基于 GICP（Generalized ICP），使用协方差加权的点到面距离度量，迭代优化

**配准频率**：每 2 秒运行一次精配准（`timer_callback`），TF 以 ~100Hz 广播（`timer_pub_tf_callback`）。

#### Why（为什么怀疑）

这是**最可能的主要原因**，理由如下：

1. **漂移现象**：你观测到静止机器人的坐标在 $x \in [10.4, 13.6]$ 之间跳变，跳变幅度达 3.2m。这**不可能来自**外参（外参是固定值）或内参，只能来自 **TF 的实时变化**——即 ICP 配准结果在不断变化。

2. **点云稀疏性**：Livox HAP 是非重复扫描激光雷达，单帧点云稀疏。虽然代码累积了 10 帧（`PcdQueue(max_size=10)`），但仍可能不足以保证稳定配准。

3. **环境退化**（Degeneracy）：实验室场景通常是长走廊或空旷房间，缺乏充足的几何约束。在某些方向上缺少特征时，ICP 在该方向上的解是不确定的。

4. **初始位姿依赖**：Quatro++ 给出初始位姿后，GICP 从该位姿迭代。如果 Quatro++ 的初始估计偏差较大（实验室特征少），精配准可能收敛到错误的局部最优。

5. **偶发跳变到 (10.4, 6.08) 和 (13.6, 6.89)**：这些值与主流值偏差 1-2m，是 ICP 配准失败的典型特征。

#### 如何判断（验证方法）

1. **直接监听 TF 变化**：
   ```bash
   ros2 run tf2_ros tf2_echo map livox
   ```
   观察输出的平移和旋转是否在不断变化。如果**机器人没有移动但 TF 在变**，说明 ICP 不稳定。

2. **RViz 可视化**：在 RViz 中同时显示实时点云（`livox` 坐标系）和地图点云（`map` 坐标系），如果两者没有精确重合或在跳动，说明 TF 不准。

3. **查看 registration 节点日志**：
   ```bash
   ros2 topic echo /rosout | grep -i "error\|fail\|converge"
   ```
   如果看到 `"Relocalization failed to converge"` 或 `error` 值很大，说明配准质量差。

4. **固定 TF 对比测试**：
   - 手动设置一个固定的 TF（通过 RViz 的 `2D Pose Estimate`）
   - 对比固定 TF 和动态 ICP TF 下的坐标精度

#### How（解决方法）

1. **提高 PCD 地图质量**：
   - 录制点云时尽量缓慢移动，或多次扫描同一区域
   - 增加点云密度（降低降采样 `voxel_size`）

2. **调整 GICP 参数**（`params/default.yaml`）：
   - 减小 `max_dist_sq`（当前 0.5 → 试试 0.3），拒绝更多错误匹配
   - 增大 `num_neighbors`（当前 20 → 试试 30），更好估计法线

3. **增加实时点云累积帧数**：
   - `PcdQueue(max_size=10)` → 尝试 20 或 30，增加配准点数

4. **使用 ICP 收敛质量过滤**：在 `radar.py` 中，如果 ICP 的 `error` 超过阈值，拒绝使用该 TF，保留上一次的好结果

5. **参考**：`/data/projects/radar/open-source-of-registration` 中的其他开源配准算法，对比效果

---

### 原因 4：PCD 地图与世界坐标系对齐不准确（`align_pcd_with_map.py`）

#### What（原理）

`align_pcd_with_map.py` 通过人工视觉对齐将 PCD 地图从雷达坐标系变换到世界坐标系。这个过程确定了三个参数：

$$
(r_x, r_y, \theta) = (3.703, 4.387, 12.9°)
$$

以及高度 $r_h = 1.05\text{m}$。

PCD 地图中每个点 $\mathbf{p}_{L_0}$ 被变换为：

$$
\mathbf{p}_M = \mathbf{R}_z(\theta) \cdot \mathbf{p}_{L_0} + [r_x, r_y, r_h]^T
$$

如果 $(r_x, r_y, \theta)$ 中任何一个参数不准确，整个 PCD 地图在世界坐标系中的位置就有偏差。由于 ICP 配准是将实时点云对齐到**这张地图**，地图的偏差会直接传递到最终坐标。

**误差放大效应**：旋转角 $\theta$ 的误差在远离旋转中心的位置会被放大。设点到旋转中心（雷达位置）的距离为 $d$，旋转误差为 $\delta\theta$，则位置误差 $\approx d \cdot \delta\theta$（弧度制）。对于 $d = 10\text{m}$，$\delta\theta = 1° = 0.017 \text{rad}$，误差 $\approx 0.17\text{m}$。

#### Why（为什么怀疑）

- 这是一个**人工手动对齐**过程，精度取决于操作者的肉眼判断
- 对齐工具的分辨率约为 $80 \text{px/m}$，1 像素 ≈ 0.0125m，精调步长为 1px
- 旋转步长最小为 0.1°，对应 $d=10\text{m}$ 处约 0.017m 误差——这较小但会累积
- 你说"感觉挺重合的"，但肉眼对齐的精度通常在 ±5px（±0.06m）和 ±0.5°

#### 如何判断（验证方法）

1. **网格线验证**：
   - 打开 `align_pcd_with_map.py`（按 `G` 显示网格）
   - 检查世界坐标网格线（0m, 5m, 10m...）是否与地图上的已知位置一致
   - 特别检查地图的四角坐标是否正确

2. **已知点验证**：
   - 在对齐后的 PCD 地图中找到若干已知世界坐标的特征点（如墙角、柱子）
   - 使用 Open3D 读取这些点的坐标，与已知值对比：
     ```python
     import open3d as o3d
     pcd = o3d.io.read_point_cloud("data/lab_pcds.pcd")
     # 用鼠标或脚本选取特征点坐标
     ```

3. **反向验证**：用你手动标定的那些特征点坐标（如 (7.86, 10.67) 等），看这些位置在 PCD 地图中是否对应正确的物理位置

#### How（解决方法）

1. 重新运行 `align_pcd_with_map.py`，仔细微调
2. 使用**精调模式**（`H/J/K/L` 1px 步进，`W/X` 0.1° 步进）
3. 对齐后，用已知坐标的地面标记点验证
4. 可以考虑用**多个控制点的最小二乘法**替代手动对齐，提高精度

---

### 原因 5：YOLO 检测不准确（无装甲板 / 检测框偏差）

#### What（原理）

YOLO 检测输出的 2D bbox 决定了在步骤③中**哪些 3D 点被选中**。如果 bbox 不准确（偏大、偏小或偏移），选中的点云子集就不能准确表示机器人的实际位置。

**三阶段检测流程**：
1. **Stage 1**（全图检测 + ByteTrack 追踪）：输出 bbox
2. **Stage 2**（装甲板颜色分类）：判断红/蓝/灰
3. **Stage 3**（灰色装甲板细分类）：最终标签

**当机器人没有装甲板时**（你的测试情况，`label = "NULL"`）：
- Stage 1 仍然可以检测到机器人的整体轮廓（因为训练数据中包含机器人整体）
- 但 Stage 2/3 无法识别装甲板，标签为 NULL
- **bbox 可能不稳定**：没有装甲板的视觉锚点，bbox 可能在帧间抖动

#### Why（为什么怀疑）

- 你观测到坐标的帧间漂移（$x$ 波动 3.2m），即使 TF 稳定，bbox 的跳动也会导致选取的点云子集不同，进而影响聚类中心
- 没有装甲板意味着 YOLO 可能检测到的是机器人的不同部位（轮子、底盘、顶部），导致 bbox 中心飘移
- `div_times = 1.01`（`radar.py` L308）表示 bbox 几乎没有缩小，如果原始 bbox 过大，会包含大量背景点

#### 如何判断（验证方法）

1. **查看检测结果图像**：
   ```bash
   ros2 topic echo /detect_view --no-arr  # 查看是否有话题
   # 或者用 rqt_image_view 订阅 /detect_view
   ```
   检查 bbox 是否稳定、是否准确包围机器人

2. **打印 bbox 信息**：在 `radar.py` 的 `radar_callback` 中增加日志：
   ```python
   self.get_logger().info(f"bbox: {xyxy_box}, label: {label}")
   ```
   观察同一机器人的 bbox 是否在帧间大幅变化

3. **统计检测框面积波动**：bbox 面积应基本稳定，如果波动超过 ±30%，说明检测不稳

#### How（解决方法）

1. **测试时加装装甲板**：即使是无 ID 的装甲板，也能提供更稳定的检测锚点
2. **增大 `div_times`**（如 1.5 或 2.0）：缩小 bbox，减少背景点的干扰
3. **降低 Stage 1 置信度阈值**：确保每帧都能检测到机器人
4. **对 bbox 做时序平滑**：使用历史帧的 bbox 均值，减少单帧跳变

---

### 原因 6：DBSCAN 聚类结果不稳定

#### What（原理）

DBSCAN 聚类的结果取决于两个参数：
- `eps = 0.40m`：邻域半径
- `min_points = 10`：最少点数

在**相机坐标系**中对 bbox 内点云进行聚类后，取最大簇的质心作为目标中心。

**问题**：
1. 如果 bbox 内包含背景点（墙壁、地面），背景点可能形成更大的簇，导致中心偏离机器人
2. 如果机器人距离较远，投影到 bbox 内的点很少（< `min_points`），聚类失败，退化为中值取点
3. 聚类在**相机坐标系**中进行，但远处点的深度差异大，簇的形状可能不规则

#### Why（为什么怀疑）

- 你的测试位置 (3,3) 世界坐标 (11.39, 7.67)，距雷达约 $\sqrt{(11.39-3.7)^2 + (7.67-4.4)^2} \approx 8.4\text{m}$，在这个距离下点云密度明显降低
- `eps = 0.40m` 对于远距离可能过小（点间距变大），也可能过大（包含了背景点）

#### 如何判断（验证方法）

1. **打印聚类点数和中心**：在 `radar.py` 中聚类后添加：
   ```python
   self.get_logger().info(f"cluster points: {len(cluster_result[0])}, center: {center}")
   ```
   如果聚类点数经常变化较大，说明聚类不稳定

2. **可视化聚类结果**：将 bbox 内点云和聚类结果用 Open3D 可视化，检查簇是否合理

#### How（解决方法）

1. **自适应 `eps`**：根据目标距离动态调整——远距离增大 eps，近距离减小
2. **增加体素降采样后的密度**：减小 `voxel_size`（当前 0.035），保留更多点
3. **在聚类前先去除地面点**：利用 `radar_to_field` 将点转到世界坐标系，过滤 $z \approx 0$ 的地面点
4. **对聚类中心做时序滤波**：利用历史帧的中心做加权平均

---

### 原因 7：实时点云质量不足（累积帧数、距离滤波）

#### What（原理）

Livox HAP 采用非重复扫描模式，单帧点云覆盖面有限。代码通过 `PcdQueue(max_size=10)` 累积最近 10 帧点云来增加密度。但：

- 10 帧的积累可能不够密集，特别是远距离区域
- 距离滤波 `min_distance=1m, max_distance=40m` 参数是否合适
- 点云发布频率 ~100Hz，但每次发布的是同一个累积队列，实际更新频率取决于雷达帧率

#### Why（为什么怀疑）

- 远距离（~8.4m）处点云密度与近距离差异很大
- 累积的 10 帧可能包含由于微小振动导致的不一致性
- 如果雷达安装有微小振动，多帧累积反而会使点云"模糊化"

#### 如何判断（验证方法）

1. 在 RViz 中同时查看 `/lidar_pcds` 与 `/target_pointcloud`：前者用于判断 `registration` 输入密度，后者才是 `radar.py` 实际使用的前景点云
2. 检查机器人位置处的前景点数——至少需要几十个点才能做有效聚类；如果 `/lidar_pcds` 很密而 `/target_pointcloud` 很稀，优先检查背景地图和背景减除阈值

#### How（解决方法）

1. 增大 `PcdQueue` 的 `max_size`（如 20-30），增加点密度
2. 确保雷达安装稳固，无振动
3. 考虑对累积点云做一次体素降采样，去除重叠点

---

### 原因 8：手动标定的"真实坐标"本身不够准确

#### What（原理）

你的"真实坐标"是通过以下方法获得的：
1. 在实验室地面标记若干点（黑色胶带）
2. 在相机图像中记录这些点的像素位置
3. 通过某种映射计算世界坐标

你给出的数据表中：

| 标记点 | 世界坐标 | 像素坐标 |
|--------|----------|----------|
| (0,0) | (7.86, 10.67) | (786, 433) |
| ... | ... | ... |

可以看到世界坐标和像素坐标之间有一个近似的线性关系：$x_{world} \approx x_{px} / 100$, $y_{world} \approx (1500 - y_{px}) / 100$。这意味着你的"真实坐标"可能就是**从像素坐标直接换算来的**。

如果这个换算关系本身有偏差（地图缩放比例不精确、地图图片与实际尺寸不完全匹配），那么"真实坐标"本身就有误差。

#### Why（为什么怀疑）

- 像素坐标 (786, 433) 对应世界坐标 (7.86, 10.67) 的精确度取决于地图图片的精度
- 实验室环境下，地图是手动制作的，可能不像赛场 CAD 图纸那么精确
- 你的映射关系假设了 $100 \text{px/m}$ 的比例和地图左下角 = 世界原点 (0,0)

#### 如何判断（验证方法）

1. **物理测量验证**：用卷尺实际测量两个标记点之间的距离，与世界坐标计算的距离对比
   - 例如 (0,0) 和 (0,3)：世界距离 $= \sqrt{(11.08-7.86)^2 + (10.95-10.67)^2} = 3.23\text{m}$
   - 实际用尺子测量的距离应该接近这个值

2. **交叉验证**：用不同的方法（如全站仪、激光测距仪）测量部分标记点的坐标

#### How（解决方法）

1. 使用**激光测距仪**或**卷尺**精确测量关键标记点的位置
2. 以某个固定参考点（如房间角落）为原点，建立可靠的物理坐标系
3. 确保地图图片与物理尺寸的比例关系准确

---

### 原因 9：点云投影未考虑镜头畸变

#### What（原理）

在步骤③中，`camera_to_image()` 使用理想针孔模型将 3D 点投影到像素平面：

$$
u = f_x \frac{X_c}{Z_c} + c_x, \quad v = f_y \frac{Y_c}{Z_c} + c_y
$$

但实际相机存在**径向畸变**和**切向畸变**，真实投影应为：

$$
\begin{aligned}
r^2 &= x'^2 + y'^2 \quad (x' = X_c/Z_c, \; y' = Y_c/Z_c) \\
x'' &= x'(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 x'y' + p_2(r^2 + 2x'^2) \\
y'' &= y'(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y'^2) + 2p_2 x'y' \\
u &= f_x x'' + c_x, \quad v = f_y y'' + c_y
\end{aligned}
$$

当前畸变系数 $k_1 = -0.107, k_2 = 0.202$，对于 3072×2048 的图像，**边缘区域**的畸变位移可达 **10-30 像素**。

**影响方式**：投影位置偏移 → 错误的点被选入 bbox → 聚类中心偏离真实位置。

#### Why（为什么怀疑）

- 代码中 `camera_to_image()` **没有应用畸变校正**
- 如果机器人位于图像边缘区域，影响更大
- 但如果机器人在图像中心附近，影响较小（中心畸变接近零）

#### 如何判断（验证方法）

1. 检查机器人在图像中的位置：如果在中心 1/3 区域内，畸变影响较小
2. 用 OpenCV 计算畸变偏移量：
   ```python
   pts = np.array([[u, v]], dtype=np.float32)
   undistorted = cv2.undistortPoints(pts, K, dist_coeffs, P=K)
   print(f"偏移: {pts - undistorted}")
   ```

#### How（解决方法）

1. 在 `camera_to_image()` 投影时加入畸变模型（性能开销小）
2. 或对整张图像先做去畸变，然后在去畸变图像上做检测和投影

---

### 原因 10：外参矩阵求逆中的数值精度问题 + 代码冗余

#### What（原理）

在 `radar.py` L333-L340 中：

```python
center = np.hstack((center, np.array((1))))  # [x,y,z] → [x,y,z,1]
center = center[:3]                            # [x,y,z,1] → [x,y,z]  ← 白做了
center = np.hstack((center, np.array((1))))  # [x,y,z] → [x,y,z,1]  ← 又做了一次
lidar_center = np.dot(self.extrinsic_matrix_inv, center)
```

前三行实际效果等同于只做了一次齐次化，不会产生数学错误，但表明代码可能经过多次修改，需要注意是否存在其他类似的逻辑问题。

另外，`extrinsic_matrix_inv = np.linalg.inv(extrinsic_matrix)` 使用了通用矩阵求逆。对于刚体变换矩阵，更精确的做法是利用正交性：

$$
\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T \mathbf{t} \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

但实际上 `np.linalg.inv` 对 4×4 矩阵的精度足够，这通常不是主要误差来源。

#### Why（为什么怀疑）

- 代码冗余暗示开发过程中可能有其他改动引入了 bug
- 但数学上这段代码是正确的

#### 如何判断

- 打印 `extrinsic_matrix @ extrinsic_matrix_inv`，应非常接近单位矩阵
- 检查条件数：`np.linalg.cond(extrinsic_matrix)` 应接近 1

#### How

- 清理冗余代码，使逻辑更清晰
- 可以使用解析逆公式替代通用 `inv`，但影响极小

---

### 原因 11：LiDAR 与相机的时间不同步

#### What（原理）

`detector_node`（相机检测）和 `lidar_node`（点云接收）是两个独立节点，各自以不同的频率运行。`radar.py` 在收到检测结果时，使用的是**最新缓存的点云**（`self.lidar_points`），而非与检测帧精确同步的点云。

如果相机帧和点云帧之间有时间差 $\Delta t$，而在这段时间内雷达或目标有移动，则点云与检测框之间就会存在空间偏移。

#### Why（为什么怀疑）

- 你的测试中机器人是静止的，所以时间同步对**你的当前测试**影响应该不大
- 但在实际比赛中目标高速移动时，这会是一个显著误差源

#### 如何判断

- 静止测试时该因素影响极小——如果静止时误差已经很大，优先排查其他原因
- 运动测试时可以通过降低速度来判断是否有明显差异

#### How

- 使用 ROS2 的消息时间戳做精确同步（`message_filters::TimeSynchronizer`）
- 或对检测结果做运动补偿

---

### 原因 12：EKF 参数不当导致滤波异常

#### What（原理）

当前 EKF 参数：
- $P_0 = 0.001$（初始协方差）
- $Q = 10^{-4}$（过程噪声）
- $R = 0.0005$（观测噪声）
- 间隔 $\Delta t = 0.05\text{s}$

**问题**：$R = 0.0005$ 非常小，意味着 EKF 几乎完全信任每次观测值。当观测值有跳变（如 ICP 偶发失败导致坐标跳到 13.5m），EKF 不会平滑掉这个跳变，而是直接跟随。

此外，EKF 的速度/加速度估计基于相邻两帧坐标的差值，如果原始坐标跳变，计算出的速度/加速度会异常大，进一步影响预测。

#### Why（为什么怀疑）

- 你观测到的坐标漂移中，如果 EKF 参数合理（$R$ 较大），应该能平滑掉那些偶发的跳变点
- 当前参数设置让 EKF 沦为"透传"，失去了滤波意义

#### 如何判断

- 同时监听 `/location` 和 `/ekf_location_filtered`，对比两者的波动幅度
- 如果两者几乎一样，说明 EKF 没有起到滤波作用

#### How

- 增大 $R$（如 0.1 ~ 1.0），让 EKF 更相信运动模型而非观测值
- 增大 $Q$（如 0.01），允许更大的运动不确定性
- 添加**异常值检测**：如果观测值与预测值偏差超过阈值（如 2m），拒绝该观测

---

### 原因 13：Quatro++ 粗配准给出的初始位姿偏差

#### What（原理）

`registration_node` 使用 `use_fixed: true` 模式——先用 Quatro++ 做一次粗配准，然后用 small_gicp 做持续精配准。如果 Quatro++ 的初始位姿就有偏差，后续 GICP 可能会收敛到**错误的局部最优**。

Quatro++ 依赖 FPFH（Fast Point Feature Histograms）特征匹配。实验室场景如果几何特征不够丰富（如只有平面墙壁），FPFH 特征区分度低，粗配准可能失败。

#### Why（为什么怀疑）

- 实验室通常比赛场几何特征更少
- `m_noise_bound = 0.2` 可能对实验室场景过于严格

#### 如何判断

- 在 registration 节点启动时检查日志输出的初始 TF
- 如果初始 TF 就明显不对（在 RViz 中点云不重合），说明 Quatro++ 失败

#### How

- 使用 RViz 手动设置初始位姿（`2D Pose Estimate`），绕过 Quatro++
- 调整 Quatro++ 参数：增大 `m_fpfh_radius`、减小 `m_noise_bound`
- 或直接设 `use_quatro: false`，完全使用手动初始位姿 + GICP

---

### 原因 14：`remove_ground_points` 中的去地面逻辑无效

#### What（原理）

`radar.py` 的 `remove_ground_points()` 方法原本用于去除地面点，但当前代码中阈值设为 `-11111`：

```python
mask = ((field_pts[:, 2] > -11111))  # 永远为 True
```

这意味着**没有任何点被过滤掉**。地面点全部保留，参与后续的投影和聚类。

**影响**：如果 bbox 覆盖的区域内有大量地面点，这些地面点的 $z \approx 0$（世界坐标系），在相机坐标系中位于机器人下方。它们会拉低聚类中心的 $Y_c$ 值（相机坐标系中 $Y$ 向下），最终导致世界坐标偏移。

#### Why（为什么怀疑）

- $-11111$ 阈值明显是占位符，说明去地面功能被禁用了
- 但如果地面点被聚类为独立簇（与机器人点分开），DBSCAN 应该能把它们区分开
- 然而如果地面点和机器人底部的点太近（$< \text{eps} = 0.40\text{m}$），它们可能合并为同一簇

#### 如何判断

- 打印聚类结果中最大簇的 $z$ 值范围
- 如果包含 $z \approx 0$ 的点，说明地面点被包含了

#### How

- 启用去地面：把 `-11111` 改为合理阈值（如 `0.15` 或 `0.20`，取决于地面高度）
- 或在聚类前，利用 TF 将相机坐标系的点转到世界坐标系，过滤掉 $z < \text{threshold}$ 的点

---

## 十六、原因汇总与优先级排序

根据你的测试现象（静止机器人、系统性偏移 + 大幅跳变），**按可能性从高到低排序**：

| 优先级 | 原因 | 误差类型 | 估计影响 | 验证难度 |
|--------|------|----------|----------|----------|
| ⭐⭐⭐⭐⭐ | **原因 3：ICP 配准不稳定（已排除）** | 跳变 + 漂移 | 0.5 ~ 3m | 低（直接监听 TF） |
| ⭐⭐⭐⭐ | **原因 2：外参标定不准（已排除）** | 系统性偏移 | 0.1 ~ 1m | 中（投影叠加验证） |
| ⭐⭐⭐⭐ | **原因 4：PCD 地图对齐不准** | 系统性偏移 | 0.1 ~ 0.5m | 中（已知点验证） |
| ⭐⭐⭐ | **原因 5：YOLO 检测框抖动** | 随机漂移 | 0.1 ~ 0.5m | 低（查看检测结果） |
| ⭐⭐⭐ | **原因 6：DBSCAN 聚类不稳定** | 随机漂移 | 0.1 ~ 0.5m | 低（打印聚类信息） |
| ⭐⭐⭐ | **原因 12：EKF 参数不当** | 未能平滑跳变 | 间接影响 | 低（对比两话题） |
| ⭐⭐⭐ | **原因 14：去地面无效** | 偏移 | 0.1 ~ 0.3m | 低（检查阈值） |
| ⭐⭐ | **原因 13：Quatro++ 初始位姿偏差** | 系统性偏移 | 0.5 ~ 2m（初始） | 中 |
| ⭐⭐ | **原因 9：投影无畸变校正** | 边缘偏移 | 10-30px | 低（计算畸变量） |
| ⭐⭐ | **原因 8：真实坐标本身不准** | 基准误差 | 0.05 ~ 0.3m | 低（尺子测量） |
| ⭐⭐ | **原因 7：点云密度不足** | 随机漂移 | 0.1 ~ 0.3m | 低（RViz 查看） |
| ⭐ | **原因 1：内参不准** | 投影偏移 | < 0.1m | 中 |
| ⭐ | **原因 10：数值精度** | 极小 | < 0.01m | 低 |
| ⭐ | **原因 11：时间不同步** | （静止测试无影响） | 0m | - |


已找到原因：display小地图位置不准。现已修正。

## 十七、推荐的排查步骤

按照效率最高的顺序：

### 第一步：验证 ICP TF 是否稳定（5 分钟）

```bash
# 终端 A：启动系统
# 终端 B：监听 TF
ros2 run tf2_ros tf2_echo map livox
```

如果平移分量在不断变化（变化幅度 > 0.1m），**ICP 是主要问题**，优先解决。

### 第二步：验证外参（10 分钟）

写一个脚本将点云投影到图像上：
```python
# 伪代码
for point in lidar_points:
    cam_point = T_CL @ point
    u, v = project(cam_point, K)
    cv2.circle(image, (u, v), 1, color_by_depth)
cv2.imshow("projection", image)
```

如果点云轮廓与图像物体对齐良好，外参没问题。

### 第三步：固定 TF 测试（15 分钟）

手动计算或用 RViz 设定一个固定的 `map→livox` TF，跳过 ICP。如果此时坐标准确且稳定，确认问题在 ICP。

### 第四步：逐一排查其余因素

按优先级表逐一验证和修复。

---

*文档结束。最后更新：2026-03-11，基于 `master` 分支代码分析。*
