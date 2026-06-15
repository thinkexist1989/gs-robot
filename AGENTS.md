# gs-robot — AI Agent Instructions

7 自由度机械臂实时控制系统。Qt GUI + EtherCAT 硬件通信 + 轨迹规划。

## 构建

```bash
mkdir build && cmake -S . -B build
cmake --build build -j$(nproc)
```

**运行**（EtherCAT 需要 raw socket 权限）：
```bash
sudo ./build/SpaceBuilding
```

没有单元测试框架，测试方式是构建后实机运行。

## 项目结构

| 文件 | 职责 |
|------|------|
| `ethercatthread.cpp/.h` | EtherCAT 主站，10ms 周期控制 7 个伺服驱动器 + 1 个力矩传感器（从站 8）。若初始化时发现从站数量不为 7（或 8），应记录错误并安全退出，不得进入控制循环。|
| `mainwindow.cpp/.h` + `mainwindow.ui` | Qt 主窗口，显示关节状态，发送控制命令 |
| `robotdescription.cpp/.h` | KDL 运动学：从 `robotdh.txt` 加载 DH 参数，正/逆运动学求解。若 `robotdh.txt` 不存在或列数不符，应抛出带有明确路径和期望格式说明的异常，禁止以默认零值静默继续运行。|
| `straightlinetrajectorygenerator.cpp/.h` | Ruckig 7 自由度笛卡尔直线轨迹规划 |
| `socketthread.cpp/.h` | TCP 线程，100ms 周期向远程客户端发送 7 轴角度 |
| `vision.cpp/.h` | 深度相机 36 字节二进制协议解析，转换为 6D 位姿 |
| `common.cpp/.h` | 字节序转换、校验和、数据类型转换工具函数 |

## 第三方库（全部在 `third_party/` 源码编译）

| 库 | 目录 | CMake 目标 |
|----|------|-----------|
| SOEM 1.4.0 | `third_party/SOEM-1.4.0` | `soem`（静态库）|
| ruckig 0.17.3 | `third_party/ruckig-0.17.3` | `ruckig` |
| orocos-kdl 1.5.3 | `third_party/orocos_kinematics_dynamics-1.5.3/orocos_kdl` | `orocos-kdl` |
| Eigen3 | 系统安装（`/usr/include/eigen3`）| — |
| Qt 5/6 | 系统安装 | `Qt5::Widgets` 等 |

### CMakeLists.txt 已知陷阱

- **SOEM 使用旧式 `include_directories()`**，头文件不会自动传播。`set_target_properties(soem PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ...)` 必须保留。
- **SOEM 自带 `-Werror`**，GCC 11 上会编译失败，已通过 `target_compile_options(soem PRIVATE -Wno-error)` 解决。
- **ruckig 传播 C++20 要求**：其 CMakeLists 中 `target_compile_features(ruckig PUBLIC cxx_std_20)` 会通过 CMake 依赖模型传播给所有链接它的目标。因此 `SpaceBuilding` 实际上以 C++20 编译（CMakeLists.txt 中 `set(CMAKE_CXX_STANDARD 17)` 的效果被覆盖）。如需明确，可将主目标改为 `set(CMAKE_CXX_STANDARD 20)`。

## 关键约定

### 单位
- 关节位置：电机计数（counts），转度数：大关节（关节 1–4）÷161，小关节（关节 5–7）÷121（`CNT_PER_CYCLE = 65536`）
- 笛卡尔位置：**米**（KDL 内部）
- 角度：内部使用**弧度**，与外部 I/O 交换使用**度**
- 力矩传感器：力（N）、力矩（Nm）

### PDO 结构体
`ethercatthread.h` 中的 `outputs`/`inputs`/`M4313_TxPDO` 必须使用 `#pragma pack(push,1)` 保证字节对齐与硬件匹配，不要修改填充方式。

### 网络接口（硬编码）
EtherCAT 网口在 `ethercatthread.cpp:30` 硬编码为 `"enp172s0"`，换机器需修改：
```cpp
char *argv_ether[] = {"ether_main", "enp172s0"};
```
如需使其可配置，应将接口名提取为构造函数参数或配置文件条目，而非继续硬编码；不要在未告知用户此变更影响实时性的情况下静默修改。

### 线程通信
- GUI → EtherCAT：通过 `QMutex` 保护的命令队列（`servo_CMD`）
- EtherCAT → GUI：`sendEngDatas()` Qt 信号（每 10ms 周期发送）
- 关节角度发送：`std::queue<std::array<double,7>>` + QMutex

### robotdh.txt 格式
DH 参数文件（Craig 1989 约定），列顺序：`d a alpha theta q_max q_min`（度为单位）。最后一行为末端执行器工具帧，4 列含义为：`d a alpha theta`（与关节行相同含义，无关节限位列），例如：`0.257 0 0 0`。

## 运行时要求
- Linux，**root 权限**或 `CAP_NET_RAW`（EtherCAT raw socket）
- 网口 `enp172s0` 连接 7 个 Synapticon 伺服驱动器
- 从站 8（可选）：M4313 六维力矩传感器
- 实时性：10ms EtherCAT 周期，建议关闭 CPU 频率调节（`/dev/cpu_dma_latency` 已在代码中设置）
