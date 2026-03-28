<div align="center">

# Renesas-DM-Arm 激光除草 (RA6M5)

一个面向 **Renesas RA6M5** 的六轴机械臂嵌入式工程，集成了 **FSP/RASC 工程生成**、**CMake 交叉编译**、**DM 电机 CAN 控制**、**WiFi/UDP 通信**、**运动学求解** 与 **任务级分层状态机**，可用于从视觉识别输入到机械臂执行激光除草任务的完整联调。

![MCU](https://img.shields.io/badge/MCU-RA6M5-blue)
![Toolchain](https://img.shields.io/badge/Build-CMake%20%2B%20Ninja-orange)
![Framework](https://img.shields.io/badge/FSP-Renesas-green)
![Language](https://img.shields.io/badge/Language-C%20%2F%20Python%20%2F%20MATLAB-lightgrey)

</div>

---

## 1. 项目简介

本仓库是一个 **基于 RA6M5 的六轴机械臂控制工程**，已经按职责分层的完整嵌入式应用：

- **platform 层**：UART / CAN / SysTick / LED / retarget 等硬件抽象
- **infra 层**：环形缓冲区、帧解析器、非阻塞延时、矩阵库、HFSM 引擎
- **domain 层**：机械臂运动学建模、正逆解、姿态转换
- **device 层**：DM 电机驱动、WiFi/BT 模块驱动
- **app 层**：通信数据解析、任务状态机、业务流程编排

当前主流程可概括为：

1. RA6M5 上电初始化外设与网络；
2. 通过 WiFi 透传链路接收上位机 / K230 发送的识别结果；
3. 解析 `WEED:id,x,y,z,confidence` 数据；
4. 在 MCU 上完成目标位姿规划与逆解搜索；
5. 驱动 6 个 DM 电机运动到瞄准位姿；
6. 发送 `Ready to Laser` 通知外部执行激光；
7. 接收 `Laser OK` 或超时后回到待机；
8. 接收完成信号后执行整机复位。

---

## 2. 工作区特性

### 2.1 核心能力

- 基于 **Renesas RA6M5 (R7FA6M5BF2CBG / Cortex-M33)**
- 使用 **Renesas FSP + Smart Configurator (RASC)** 生成底层工程
- 使用 **CMake + ARM GNU Toolchain + Ninja** 构建
- 支持 **DM 电机 CAN 总线控制**
- 支持 **W800 类 WiFi/BT 模块 AT 指令 + 透传通信**
- 内置 **六轴机械臂正逆运动学与姿态工具**
- 内置 **轻量级分层状态机 HFSM**
- 配套 **Python 联调脚本 (`k230/`)** 与 **MATLAB 验证脚本 (`matlab/`)**

### 2.2 当前业务状态机

任务状态机定义位于 `src/app/fsm/`，当前状态包括：

- `init`
- `idle`
- `plan_aim`
- `moving`
- `lasering`
- `finish`
- `fault`

事件包括：

- `INIT_COMPLETE`
- `START_SEARCH`
- `SEARCH_COMPLETE`
- `MOVE_COMPLETE`
- `LASER_COMPLETE`
- `FINISH`
- `FINISH_COMPLETE`
- `DETECT_ERROR`
- `ERROR_COMPLETE`

---

## 3. 仓库结构

```text
.
├─ CMakeLists.txt              # 顶层构建入口
├─ Config.cmake                # 用户自定义构建配置（如 RASC_EXE_PATH）
├─ configuration.xml           # FSP / Smart Configurator 配置源
├─ buildinfo.json              # 目标器件、链接脚本、包含路径等元信息
├─ cmake/                      # 工具链与 RASC 生成的 CMake 配置（应纳入版本管理）
├─ ra/                         # Renesas FSP 源码 / CMSIS / 驱动
├─ ra_cfg/                     # FSP 配置头文件
├─ ra_gen/                     # FSP / RASC 生成代码（main、hal_data 等）
├─ script/                     # 链接脚本等构建脚本
├─ src/
│  ├─ platform/                # 平台抽象：UART、CAN、LED、SysTick...
│  ├─ infra/                   # 基础设施：delay、matrix、protocol_parser、hfsm...
│  ├─ domain/                  # 机械臂运动学与数学模型
│  ├─ device/                  # 设备驱动：DM 电机、WiFi/BT
│  ├─ app/                     # 业务逻辑：packet parser、mission FSM
│  ├─ hal_entry.c              # 应用主入口
│  └─ hal_warmstart.c          # 启动相关代码
├─ k230/                       # 上位机 / K230 侧 Python 联调脚本
├─ matlab/                     # 正逆运动学建模与验证脚本
├─ build/                      # 构建输出目录
└─ .vscode/                    # VS Code 调试 / 烧录配置
```

### 分层说明

| 层级 | 路径 | 作用 |
|---|---|---|
| 平台层 | `src/platform/` | 最贴近硬件，封装 UART/CAN/GPIO/SysTick 等底层能力 |
| 基础设施层 | `src/infra/` | 与业务无关的通用能力，如环形缓冲、帧解析、矩阵、HFSM |
| 领域层 | `src/domain/` | 六轴机械臂模型、姿态表示、IK/FK 等核心算法 |
| 设备层 | `src/device/` | 将具体硬件设备抽象成接口，如 DM 电机与 WiFi 模块 |
| 应用层 | `src/app/` | 任务流程编排、通信数据转业务事件、状态机驱动 |

---

## 4. 软硬件组成

### 4.1 硬件

| 模块 | 说明 |
|---|---|
| MCU | Renesas RA6M5 (`R7FA6M5BF2CBG`) |
| 执行机构 | 6 个 DM 电机 |
| 通信总线 | CAN |
| 无线模块 | UART 接入的 WiFi/BT 模块 |
| 上位机 / 感知端 | K230 / PC / 其他能发送识别结果的设备 |

### 4.2 软件

| 模块 | 说明 |
|---|---|
| FSP / RASC | 管理外设配置与生成底层初始化代码 |
| CMake | 跨平台构建入口 |
| ARM GNU Toolchain | 交叉编译器 |
| pyOCD | 烧录与调试 |
| Python | 联调、模拟视觉输入 |
| MATLAB | 运动学推导、验证与符号建模 |

---

## 5. 快速开始

### 5.1 依赖准备

你至少需要准备：

- **Renesas FSP / Smart Configurator (RASC)**
- **ARM GNU Toolchain**（含 `arm-none-eabi-gcc`）
- **CMake >= 3.16**
- **Ninja**
- **pyOCD**（用于烧录 / 调试）
- **K230 IDE**（用于 `k230/` 联调脚本）

#### Python 安装建议

```bash
pip install pyocd
```

---

### 5.2 配置工具链路径

本工程的 `cmake/gcc.cmake` 会强制要求你提供 ARM 工具链的 `bin` 目录路径。

可通过以下两种方式之一设置：

#### 方式一：环境变量

```bash
export ARM_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain/bin
```

Windows PowerShell：

```powershell
$env:ARM_TOOLCHAIN_PATH = "C:/path/to/arm-gnu-toolchain/bin"
```

#### 方式二：CMake 命令行传参

```bash
-DARM_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain/bin
```

---

### 5.3 配置 RASC 路径

由于当前工程的构建流程包含 **RASC 预生成 / 后生成步骤**，因此请确保 `RASC_EXE_PATH` 可用。

你可以：

- 直接修改 `Config.cmake`
- 或在命令行传入 `-DRASC_EXE_PATH=...`
- 或设置同名环境变量

例如：

```bash
-DRASC_EXE_PATH=/path/to/rasc
```

> 注意：当前 `Config.cmake` 中的默认值是一个 **Windows 本机路径**。如果你换电脑或换系统，不覆盖它通常会直接导致构建失败。

---

### 5.4 命令行构建

#### Windows PowerShell（推荐与当前目录结构保持一致）

```powershell
$env:ARM_TOOLCHAIN_PATH = "C:/path/to/arm-gnu-toolchain/bin"
$env:RASC_EXE_PATH = "C:/path/to/rasc.exe"

cmake -S . -B build/Debug `
  -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake `
  -DARM_TOOLCHAIN_PATH="$env:ARM_TOOLCHAIN_PATH" `
  -DRASC_EXE_PATH="$env:RASC_EXE_PATH"

cmake --build build/Debug
```

#### Linux / WSL

```bash
export ARM_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain/bin
export RASC_EXE_PATH=/path/to/rasc

cmake -S . -B build/Debug \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake \
  -DARM_TOOLCHAIN_PATH="$ARM_TOOLCHAIN_PATH" \
  -DRASC_EXE_PATH="$RASC_EXE_PATH"

cmake --build build/Debug
```

#### 构建产物

默认重要输出包括：

- `build/Debug/0_Debug.elf`
- `build/Debug/0_Debug.srec`
- `build/Debug/0_Debug.map`
- `build/Debug/0_Debug.sbd`

> 说明：本工程在 **单配置生成器**（如普通 `Ninja`）下，必须显式指定 `-DCMAKE_BUILD_TYPE=Debug` 或其他配置，否则顶层 `CMakeLists.txt` 会直接报错。

---

### 5.5 烧录与调试

#### pyOCD 烧录

```bash
pyocd flash -t r7fa6m5bf build/Debug/0_Debug.elf
```

#### VS Code

仓库内已经提供：

- `.vscode/tasks.json`：烧录任务
- `.vscode/launch.json`：`cortex-debug + pyOCD` 调试配置

如需直接使用，请根据你自己的环境修改：

- `gdbPath`
- 工具链安装路径
- 调试器连接方式

---

## 6. 运行逻辑

### 6.1 程序入口

程序入口链路如下：

```text
ra_gen/main.c
  └─ hal_entry()
       └─ src/hal_entry.c
```

其中 `hal_entry()` 是整个应用的主入口，负责：

- 创建 UART 环形缓冲区
- 初始化 SysTick / delay / UART / WiFi / CAN / LED
- 使能 6 个 DM 电机
- 设置 WiFi 网络参数
- 初始化机械臂 MDH 参数
- 初始化任务状态机
- 在主循环中处理：
  - mission 状态机
  - 网络心跳
  - 数据帧接收与解析
  - 电机反馈轮询
  - 周期性日志输出

---

### 6.2 默认网络配置

`src/hal_entry.c` 中当前默认配置如下：

| 参数 | 值 |
|---|---|
| SSID | `K230` |
| Password | `12345678` |
| Protocol | `UDP` |
| Role | `Client` |
| Remote IP | `192.168.169.1` |
| Remote Port | `8080` |
| Local Port | `5000` |
| 帧头 | `RENE:` |

如果你更换了上位机、热点、端口或工作模式，请同步修改 `WifiBtConnectInfo`。

---

### 6.3 上下位机通信协议

#### 帧格式

WiFi 模块透传的业务帧基于自定义头部封装，当前初始化时使用：

```text
RENE:
```

底层帧解析逻辑位于：

- `src/infra/protocol_parser.h`
- `src/infra/protocol_parser.c`

该解析器支持带帧头、长度字段与可选 CRC 的流式解析。

#### 当前业务负载

应用层当前主要接收：

```text
WEED:id,x,y,z,confidence
```

例如：

```text
WEED:3,0.12,0.25,0.18,0.91
```

字段定义：

| 字段 | 含义 |
|---|---|
| `id` | 杂草编号 |
| `x,y,z` | 目标坐标 |
| `confidence` | 识别置信度，范围应在 `[0, 1]` |

业务解析位于：

- `src/app/packet_parser.h`
- `src/app/packet_parser.c`

#### 激光控制握手

机械臂进入 `lasering` 状态后会发送：

```text
Ready to Laser
```

外部设备处理完成后可回传：

```text
Laser OK
```

若未收到，也会在超时后自动回到空闲态。

---

## 7. 任务流程

当前任务状态机执行逻辑大致如下：

```text
init
  -> idle
  -> plan_aim
  -> moving
  -> lasering
  -> idle
```

异常情况下：

```text
任意 active 子状态
  -> fault
  -> init
  -> idle
```

### 业务说明

#### 1) `idle`

等待识别结果或完成信号。

#### 2) `plan_aim`

根据 `WEED` 数据生成目标瞄准位姿，并在 MCU 侧完成可达位姿搜索与 IK 求解。

#### 3) `moving`

向 6 个 DM 电机下发目标关节角，并结合反馈判断是否到位；若 5 秒内未完成则进入故障态。

#### 4) `lasering`

通知外部设备执行激光；收到 `Laser OK` 或超时后回到 `idle`。

#### 5) `finish`

执行六轴回零；若回零失败或超时则进入 `fault`。

#### 6) `fault`

记录错误信息并闪灯，等待外部恢复事件。

---

## 8. DM 电机控制

DM 电机接口位于：

- `src/device/motor.h`
- `src/device/motor.c`

支持的能力包括：

- 电机使能 / 失能
- MIT 模式控制
- 位置速度模式控制
- 纯速度模式控制
- 位置速度电流模式控制
- 主动请求反馈
- 位置 / 速度 / 扭矩 / 错误码读取

当前任务流程中最常用的是：

```c
 dm.set_pos_spd(id, target_pos, speed);
```

并通过：

```c
 dm.request_feedback(id);
 dm.update(&feedback);
```

完成状态反馈闭环。

---

## 9. 运动学与数学模块

### 9.1 MCU 侧

机械臂运动学接口位于：

- `src/domain/arm_kine.h`
- `src/domain/arm_kine.c`

基础数学能力位于：

- `src/infra/matrix.h`
- `src/infra/matrix.c`

内容包括：

- 矩阵创建、乘法、转置、求逆
- 向量运算
- 四元数与旋转矩阵互转
- 六轴机械臂正逆运动学
- 多解求解与解选择

### 9.2 MATLAB 侧

`matlab/` 目录可用于：

- 建模验证
- FK/IK 推导
- 公式核对
- 离线调参与分析

代表脚本包括：

- `forward_kinematics.m`
- `inverse_kinematics.m`
- `get_all_ik.m`
- `get_piper_equation.m`
- `model.m`

---

## 10. K230 / 上位机联调

`k230/` 目录中的 Python 脚本可用于快速模拟视觉输入，需要 K230 的 IDE。

### 示例

当前示例脚本会：

- 建立 WiFi 通信对象
- 周期性处理 MCU 回包
- 在 MCU 就绪后每 20 ms 发送一帧随机的 `WEED` 数据

适合作为以下场景的测试入口：

- 网络链路是否通畅
- 帧解析是否正确
- 状态机是否能被正确触发
- IK 与电机联动流程是否正常

## 11. 许可证

本项目采用 **MIT License** 开源，详见 [LICENSE](LICENSE) 文件。

## 12. 贡献

欢迎提交 Issues 和 Pull Requests！

## 13. 联系方式

如有问题或建议，请通过以下方式联系：
- GitHub Issues
- 提交讨论 (GitHub Discussions)
