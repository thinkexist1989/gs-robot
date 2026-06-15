# GS-Robot 六维力导纳柔顺控制技术文档

## 1. 项目概述

本项目为 7-DOF 机械臂 EtherCAT 实时控制系统，基于 Qt + SOEM + KDL + Ruckig 实现笛卡尔直线运动（`makeMoveL`）与导纳力位混合控制。

**核心能力：**
- EtherCAT 实时通信（100Hz / 10ms 周期）
- M4313 六维力/力矩传感器数据采集
- KDL 正/逆运动学
- Ruckig 时间最优 S 曲线轨迹规划
- 导纳柔顺控制（XY 平面 + 绕 Z 轴旋转）

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Qt 主线程 (MainWindow)                       │
│                                                                     │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐  │
│  │  UI 控件      │    │  makeMoveL()     │    │  computeAdm()    │  │
│  │  导纳参数输入  │───▶│  规划层采样循环    │───▶│  导纳增量计算     │  │
│  │  启用开关     │    │  (10ms步长)       │    │  (显式积分)      │  │
│  └──────────────┘    └────────┬─────────┘    └──────────────────┘  │
│                               │                                     │
│                          IK 求解 (KDL)                              │
│                               │                                     │
│                     ┌─────────▼──────────┐                         │
│                     │  关节脉冲序列        │                         │
│                     │  lines_arr[7][]     │                         │
│                     └─────────┬──────────┘                         │
│                               │ setInputsCMD()                     │
├───────────────────────────────┼─────────────────────────────────────┤
│                  EtherCAT 实时线程 (EtherCatThread)                  │
│                               │                                     │
│  ┌────────────────────────────▼──────────────────────────────────┐  │
│  │  slave_execute_command()  —  10ms 周期循环                     │  │
│  │                                                                │  │
│  │  • 读取 7 个关节反馈（位置/速度/力矩/状态字）                     │  │
│  │  • 读取 M4313 六维力传感器 → latestFtData（mutex 保护）          │  │
│  │  • 下发 CSP 目标位置脉冲                                       │  │
│  │  • emit sendEngDatas() → 主线程更新 UI                         │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  共享数据：                                                          │
│  • latestFtData (M4313_TxPDO) — 六维力最新值，mutex 保护             │
│  • engDatas (EngDatas) — 7 轴反馈状态                                │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. 六维力传感器（M4313）

### 3.1 硬件信息

| 项目 | 值 |
|------|-----|
| 型号 | M4313 |
| EtherCAT 从站号 | Slave 8 |
| 接口 | EtherCAT CoE (CAN over EtherCAT) |
| PDO | TxPDO: `M4313_TxPDO` (28 bytes) |
| 数据内容 | `fx, fy, fz` (N), `mx, my, mz` (N·m) |

### 3.2 PDO 数据结构

```cpp
struct M4313_TxPDO {      // 六维力/力矩
    uint16_t data_no;      // 0x6030:01  数据序号
    float fx;              // 0x6030:02  X 轴力 (N)
    float fy;              // 0x6030:03  Y 轴力 (N)
    float fz;              // 0x6030:04  Z 轴力 (N)
    float mx;              // 0x6030:05  X 轴力矩 (N·m)
    float my;              // 0x6030:06  Y 轴力矩 (N·m)
    float mz;              // 0x6030:07  Z 轴力矩 (N·m)
};
```

### 3.3 采集流程

```
EtherCAT 周期 (10ms)
  │
  ▼
readM4313Data(8, ft)          // 从 ec_slave[8].inputs memcpy
  │
  ▼
ftMutex.lock()
latestFtData = ft             // 更新共享数据（mutex 保护）
ftMutex.unlock()
```

**关键点：**
- `readM4313Data()` 通过 `memcpy` 从 SOEM 的 `ec_slave[slave].inputs` 直接读取二进制 PDO 数据
- 每个 EtherCAT 周期（10ms）都更新 `latestFtData`，确保主线程读取的是最新值
- 打印频率：每 100 个周期（1秒）打印一次，避免刷屏

### 3.4 坐标系

**六维力传感器坐标系 ≠ 法兰盘坐标系。** 传感器安装时绕 Z 轴旋转了 135°。

```
法兰盘坐标系 ──绕Z轴旋转135°──▶ 六维力传感器坐标系
```

因此读取到的 `fx, fy` 需要先进行旋转变换到法兰盘坐标系，再参与导纳计算。

变换公式：

```
┌ fx_flange ┐       ┌ fx_sensor ┐
│           │ = R_z │           │    R_z = RotZ(-135°)
└ fy_flange ┘(-135°)└ fy_sensor ┘

mz 不受 XY 平面旋转影响，直接使用。
```

---

## 4. 导纳柔顺控制

### 4.1 控制原理

导纳控制的核心思想：**把机械臂末端当作一个虚拟的质量-弹簧-阻尼系统**。传感器测到力 → 导纳模型计算出期望的位移/旋转偏差 → 叠加到目标位姿上 → IK 求解 → 发送给伺服。

```
         ┌─────────────────────────────────────────────┐
力传感器 │  fx, fy, mz                                 │
         │    │                                        │
         ▼    ▼                                        │
    ┌──────────────┐                                   │
    │  坐标系变换    │  传感器系 → 法兰盘系 (RotZ(-135°))  │
    └──────┬───────┘                                   │
           │                                           │
           ▼                                           │
    ┌──────────────┐                                   │
    │  零漂补偿     │  F_corrected = F_raw - F_zero      │
    └──────┬───────┘                                   │
           │                                           │
           ▼                                           │
    ┌──────────────┐                                   │
    │  死区滤波     │  |F| < threshold → F = 0           │
    └──────┬───────┘                                   │
           │                                           │
           ▼                                           │
    ┌──────────────────────────────────────┐           │
    │  导纳模型（显式 Euler 积分）           │           │
    │                                      │           │
    │  a = (F - B·v) / M                   │           │
    │  v += a · dt                          │           │
    │  x += v · dt                          │           │
    │  x = clamp(x, -x_max, x_max)         │           │
    │                                      │           │
    │  输出：delta_x, delta_y (位置偏移)    │           │
    │        delta_rz (旋转偏移)            │           │
    └──────┬───────────────────────────────┘           │
           │                                           │
           ▼                                           │
    ┌──────────────┐                                   │
    │  坐标系叠加    │  delta_base = R · delta_tool      │
    │  （IK 之前）   │  R_corrected = R · RotZ(delta_rz) │
    └──────┬───────┘                                   │
           │                                           │
           ▼                                           │
    ┌──────────────┐                                   │
    │  IK 求解      │  R, P → joint angles              │
    └──────────────┘───────────────────────────────────┘
```

### 4.2 导纳公式

经典导纳模型：

```
M · a + B · v + K · x = F(t)
```

离散化（显式 Euler）：

```cpp
a = (F - B * v) / M;    // 加速度
v += a * dt;             // 速度积分
x += v * dt;             // 位置积分
```

**为什么用大阻尼？**
- 大 B 值（200~500 N·s/m）使系统过阻尼
- 力消失后位移迅速收敛，不会产生碰撞反弹振荡
- K = 0（纯阻尼），不引入弹簧恢复力，位置偏差完全由限幅约束

### 4.3 两路导纳

系统同时运行两路独立导纳：

| 导纳 | 输入力 | 物理量 | 参数 | 输出 |
|------|--------|--------|------|------|
| XY 平面位置 | fx, fy | 力 (N) | M_xy=1.0 kg, B_xy=300 N·s/m | delta_x, delta_y (m) |
| 绕 Z 轴旋转 | mz × gain | 力矩 (N·m) | M_rz=0.01 kg·m², B_rz=300 N·m·s/rad | delta_rz (rad) |

**Mz 放大：** `mz_effective = mz × 5.0`（可配置 `mz_gain`），补偿旋转自由度力矩灵敏度不足。

### 4.4 零漂补偿

每次执行 `makeMoveL()` 时，程序启动瞬间读取一次六维力作为零漂基准：

```cpp
ftZeroDrift = latestFtData;  // 运行开始时的读数 = 零漂
```

后续每个采样点都减去零漂：

```cpp
fx_corrected = rawFt.fx - ftZeroDrift.fx;
```

**注意：** 不做重力补偿。仅做零漂补偿，传感器安装后的重力分量被视为恒定偏置一并消除。

### 4.5 安全限幅

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `delta_xy_max` | 0.01 m (10mm) | XY 位置偏移上限 |
| `delta_rz_max` | 5° (0.087 rad) | Z 轴旋转偏移上限 |
| `force_threshold` | 0.5 N | 力死区，小于此值视为噪声 |
| `torque_threshold` | 0.05 N·m | 力矩死区 |

### 4.6 参数配置

**UI 配置（运行时可调）：**

| UI 控件 | 参数 | 默认值 |
|---------|------|--------|
| `cb_admEnable` | 导纳启用开关 | unchecked |
| `adm_Mxy` | XY 质量 (kg) | 1.0 |
| `adm_Bxy` | XY 阻尼 (N·s/m) | 300 |
| `adm_Kxy` | XY 刚度 (N/m) | 0 |
| `adm_Brz` | 旋转阻尼 (N·m·s/rad) | 300 |
| `adm_mzGain` | Mz 放大倍数 | 5.0 |

**防误操作：** `on_moveLGo_clicked()` 在调用 `makeMoveL()` 前自动调用 `on_admApply_clicked()` 从 UI 加载参数，即使未点"应用"也能正常运行。

---

## 5. makeMoveL() 完整流程

```
makeMoveL()
  │
  ├─ 1. 采集六维力零漂 (ftZeroDrift)
  │
  ├─ 2. 获取当前末端位姿 (lastFeedbackPositionPose)
  │     → R_start (四元数→旋转矩阵), P_start
  │
  ├─ 3. 读取目标位姿 (UI 输入)
  │     → R_target (RPY→旋转矩阵), P_target
  │
  ├─ 4. 计算归一化偏差
  │     → dist = ||P_target - P_start||
  │     → theta = rotation angle (axis-angle)
  │     → delta_max = max(dist, theta × 0.1)
  │     → v_norm = v_max / delta_max
  │     → a_norm = a_max / delta_max
  │
  ├─ 5. Ruckig 1D 轨迹规划 s(t) ∈ [0, 1]
  │     → otg(0.01)  // 10ms 步长
  │     → trajectory.get_duration()
  │
  ├─ 6. 采样循环 for (t = 0; t <= duration; t += 0.01)
  │     │
  │     ├─ trajectory.at_time(t, s_arr)  → s ∈ [0, 1]
  │     │
  │     ├─ 位置插值: P = P_start + s × dP
  │     │
  │     ├─ 姿态插值: SCLERP (球面线性插值)
  │     │   R = R_start × Rot(axis, s × theta)
  │     │
  │     ├─ [导纳补偿] (if admEnable)
  │     │   ├─ 读力 → 去零漂 → 坐标系变换 (传感器→法兰盘)
  │     │   ├─ computeAdmittanceDelta() → dx, dy, drz
  │     │   ├─ P += R × [dx, dy, 0]     (位置补偿)
  │     │   └─ R = R × RotZ(drz)        (旋转补偿)
  │     │
  │     ├─ IK 求解: R, P → joint angles
  │     │
  │     └─ 关节角 → 脉冲, 存入 lines_arr[]
  │
  └─ 7. 下发: setInputsCMD(cmd)
        同时通过 TCP 发送关节角给上位机可视化
```

---

## 6. 坐标系约定

### 6.1 坐标系层次

```
基坐标系 (Base Frame)
  └─▶ 法兰盘坐标系 (Flange Frame)  = R_base_tool · [0,0,0]^T
       └─▶ 工具坐标系 (Tool Frame)   (本项目中与法兰盘重合)
            └─▶ 传感器坐标系 (Sensor Frame) = RotZ(135°) · 法兰盘系
```

### 6.2 KDL 中的旋转矩阵约定

KDL::Rotation R 表示 **工具系到基系的旋转**：
```
p_base = R · p_tool
```

- **右乘旋转** = 在工具坐标系下旋转：`R_new = R · RotZ(δ)` → 工具绕自身 Z 轴转 δ
- **左乘旋转** = 在基坐标系下旋转：`R_new = RotZ(δ) · R` → 绕基坐标系 Z 轴转 δ

导纳输出的 delta 在工具坐标系下，因此：
- 位置补偿：`P_base += R · delta_tool`（R 将工具系向量映射到基系）
- 旋转补偿：`R = R · RotZ(delta_rz)`（工具系下绕 Z 轴旋转）

### 6.3 力的坐标系变换链

```
传感器原始读数 (Sensor Frame)
  │  减去零漂
  ▼
传感器坐标系 (已去零漂)
  │  乘以 RotZ(-135°)
  ▼
法兰盘坐标系 (= 工具坐标系)
  │  导纳模型处理
  ▼
delta_x, delta_y, delta_rz (工具坐标系下)
  │  位置: 乘以 R (工具→基系)
  │  旋转: 右乘 RotZ
  ▼
基坐标系下的位姿修正
```

---

## 7. EtherCAT 实时通信

### 7.1 周期与时序

| 参数 | 值 |
|------|-----|
| 周期 | 10ms (100Hz) |
| EC_CYCLETIME | 10,000,000 ns |
| EC_CYCLETIME_US | 10,000 μs |
| Ruckig OTG 步长 | 0.01s (与 EC 周期一致) |

### 7.2 从站分配

| Slave # | 设备 | 说明 |
|---------|------|------|
| 1~7 | 伺服驱动器 | 关节 1~7 (CSP 模式) |
| 8 | M4313 六维力传感器 | fx/fy/fz/mx/my/mz |

### 7.3 关节电机参数

| 关节 | 类型 | DACC | 脉冲转换 |
|------|------|------|---------|
| 1~4 | BIG_JOINT | 161 | `angle × 65536 × 161 / (2π)` |
| 5~7 | SMALL_JOINT | 121 | `angle × 65536 × 121 / (2π)` |

---

## 8. 关键代码文件

| 文件 | 说明 |
|------|------|
| `mainwindow.h` | `AdmittanceParams`, `AdmittanceState` 结构体定义；`sensorRot`, `ftZeroDrift` 成员 |
| `mainwindow.cpp:854` | `computeAdmittanceDelta()` — 导纳核心算法 |
| `mainwindow.cpp:889` | `makeMoveL()` — 笛卡尔直线运动 + 导纳补偿 |
| `mainwindow.cpp:1100` | `on_moveLGo_clicked()` — 入口，自动加载 UI 参数 |
| `mainwindow.cpp:1106` | `on_admApply_clicked()` — 从 UI 读取导纳参数 |
| `ethercatthread.h:60` | `M4313_TxPDO` 结构体定义 |
| `ethercatthread.h:163` | `latestFtData` + `ftMutex` 共享数据声明 |
| `ethercatthread.cpp:119` | `readM4313Data()` — 从 SOEM 读取 PDO |
| `ethercatthread.cpp:529` | 力数据每周期更新逻辑 |
| `mainwindow.ui:2364` | 导纳 UI 控件（checkbox, line edits, apply button） |

---

## 9. 扩展开发指南

### 9.1 新增导纳自由度

当前仅补偿 XY 平面和绕 Z 轴旋转。如需扩展到 6-DOF 完整导纳：

1. 在 `AdmittanceParams` 中增加 Z 方向和绕 X/Y 轴旋转的参数
2. 在 `AdmittanceState` 中增加对应积分状态
3. 在 `computeAdmittanceDelta()` 中增加计算逻辑
4. 在 `makeMoveL()` 循环中将 `delta_z` 加到位置，将 `delta_rx`, `delta_ry` 通过 `R = R * Rot(axis, angle)` 应用到姿态

### 9.2 加入重力补偿

当前不做重力补偿（零漂包含重力分量）。如需重力补偿：

1. 需要已知末端工具质量和质心位置
2. 根据当前姿态 R 计算重力在传感器坐标系下的分量
3. 在零漂去除后额外减去重力分量

### 9.3 调参建议

| 现象 | 调整方向 |
|------|---------|
| 响应太慢 | 减小 B_xy 或 B_rz |
| 有振荡/反弹 | 增大 B_xy 或 B_rz |
| 位移太大 | 减小 delta_xy_max |
| Mz 响应不足 | 增大 mz_gain |
| 力噪声导致抖动 | 增大 force_threshold / torque_threshold |
