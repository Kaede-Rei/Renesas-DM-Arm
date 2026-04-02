<div align="center">

# Renesas-DM-Arm 激光除草 (RA6M5)

一个面向 **Renesas RA6M5** 的六轴机械臂嵌入式控制工程，覆盖了从**通信接入**、**运动学求解**、**电机控制**到**任务状态机编排**的完整主链路。

![MCU](https://img.shields.io/badge/MCU-RA6M5-blue)
![Toolchain](https://img.shields.io/badge/Build-CMake%20%2B%20Ninja-orange)
![Framework](https://img.shields.io/badge/FSP-Renesas-green)
![Language](https://img.shields.io/badge/Language-C%20%2F%20Python%20%2F%20MATLAB-lightgrey)

</div>

该项目适合用于以下场景：

* 六轴机械臂嵌入式控制工程实践
* RA6M5 / FSP / RASC 工程组织参考
* DM 电机 CAN 控制与联调
* WiFi/UDP 感知输入到机械臂执行的闭环样例
* 机械臂运动学、任务状态机、通信协议的分层实现参考

> 当前仓库聚焦于 **RA6M5 侧主控工程**。视觉识别、上位机联调、运动学验证分别通过 `k230/` 与 `matlab/` 提供辅助脚本与验证材料。

---

## 项目概览

本项目实现了一套基于 **Renesas RA6M5** 的六轴机械臂任务执行系统。主控通过 UART 接入 WiFi 模块，与外部视觉端或上位机建立透传链路；接收识别结果后，在 MCU 侧完成目标解析、任务状态切换、运动学求解与电机控制，并在任务完成后回到待机或执行复位流程。

当前主流程如下：

1. 系统上电，初始化 SysTick、UART、CAN、LED、WiFi、电机与机械臂模型
2. WiFi 模块连接热点，并建立 UDP 透传通信
3. 外部设备通过自定义帧协议发送业务数据
4. MCU 解析 `WEED:id,x,y,z,confidence` 数据
5. 状态机根据识别结果进入搜索 / 规划 / 运动流程
6. MCU 执行逆运动学求解，得到目标关节角
7. 通过 CAN 驱动 6 个 DM 电机运动到目标位姿
8. 到位后发送 `Ready to Laser`
9. 外部设备返回 `Laser OK`，或等待超时后自动结束当前激光流程
10. 回到待机，继续处理下一目标；当收到“全部完成”标记后，进入整机收尾流程

---

## 功能特性

### 核心能力

* 基于 **Renesas RA6M5** 的裸机嵌入式控制工程
* 使用 **Renesas FSP + Smart Configurator (RASC)** 管理底层工程
* 使用 **CMake + Ninja + ARM GNU Toolchain** 构建
* 支持 **DM 电机 CAN 总线控制**
* 支持 **UART 接入 WiFi 模块 + UDP 透传通信**
* 支持 **六轴机械臂正逆运动学**
* 支持 **任务级分层状态机**
* 提供 **K230 / Python 联调脚本**
* 提供 **MATLAB 运动学验证脚本**

### 工程特点

* 分层清晰，适合后续扩展更多设备和任务
* 业务流程和底层驱动解耦，便于维护
* 通信协议、运动学、电机反馈、状态机职责边界明确
* 既可作为比赛 / 项目原型，也适合作为教学与学习样例

---

## 仓库结构

```text
./
├─ src/
│  ├─ app/                    # 应用层：任务状态机、业务包解析
│  │  ├─ fsm/
│  │  ├─ packet_parser.c
│  │  └─ packet_parser.h
│  ├─ device/                 # 设备层：DM 电机、WiFi/BT 模块
│  │  ├─ motor.c
│  │  ├─ motor.h
│  │  ├─ wifi_bt.c
│  │  └─ wifi_bt.h
│  ├─ domain/                 # 领域层：机械臂运动学
│  │  ├─ arm_kine.c
│  │  └─ arm_kine.h
│  ├─ infra/                  # 基础设施层：矩阵、延时、协议解析、HFSM
│  │  ├─ delay.c
│  │  ├─ delay.h
│  │  ├─ matrix.c
│  │  ├─ matrix.h
│  │  ├─ protocol_parser.c
│  │  ├─ protocol_parser.h
│  │  └─ hfsm/
│  ├─ platform/               # 平台层：UART / CAN / LED / SysTick / retarget
│  │  ├─ can.c
│  │  ├─ can.h
│  │  ├─ led.c
│  │  ├─ led.h
│  │  ├─ retarget.c
│  │  ├─ simple_api.h
│  │  ├─ systick.c
│  │  ├─ systick.h
│  │  ├─ uart.c
│  │  └─ uart.h
│  ├─ hal_entry.c             # 应用主入口
│  └─ hal_warmstart.c
├─ k230/                      # 上位机 / K230 侧联调脚本（如仓库中包含）
├─ matlab/                    # FK / IK / 建模验证脚本（如仓库中包含）
├─ ra/                        # Renesas FSP / CMSIS / 底层驱动
├─ ra_cfg/                    # FSP 配置头文件
├─ ra_gen/                    # RASC / FSP 生成代码
├─ cmake/                     # 构建相关脚本
├─ script/                    # 链接脚本等辅助构建资源
├─ configuration.xml          # FSP / RASC 配置源
├─ buildinfo.json             # 工程元信息
├─ Config.cmake               # 用户自定义构建配置
└─ CMakeLists.txt             # 顶层构建入口
```

---

## 分层设计

| 层级       | 路径              | 职责                                    |
| -------- | --------------- | ------------------------------------- |
| Platform | `src/platform/` | 最底层硬件抽象，封装 UART、CAN、LED、SysTick 等平台能力 |
| Infra    | `src/infra/`    | 与业务无关的通用能力，如协议解析、矩阵、延时、HFSM           |
| Domain   | `src/domain/`   | 机械臂运动学与位姿计算等核心算法                      |
| Device   | `src/device/`   | 设备级驱动与适配，如 DM 电机、WiFi 模块              |
| App      | `src/app/`      | 业务流程编排、状态机、业务数据解析                     |

这种组织方式的目标是：

* 平台相关代码与业务逻辑隔离
* 算法层与具体外设解耦
* 任务流程可重构，底层驱动可替换
* 便于后续引入更多通信协议、更多执行机构或新的任务状态

---

## 硬件与软件组成

### 硬件

| 模块    | 说明                       |
| ----- | ------------------------ |
| MCU   | Renesas RA6M5            |
| 执行机构  | 6 个 DM 电机                |
| 通信总线  | CAN                      |
| 无线模块  | UART 接入 WiFi/BT 模块       |
| 外部感知端 | K230 / PC / 其他能发送识别结果的设备 |

### 软件

| 模块                | 说明               |
| ----------------- | ---------------- |
| FSP / RASC        | 配置外设并生成底层工程      |
| CMake             | 顶层构建系统           |
| Ninja             | 构建执行器            |
| ARM GNU Toolchain | 交叉编译工具链          |
| pyOCD             | 烧录 / 调试          |
| Python            | 联调脚本与模拟通信        |
| MATLAB            | 机械臂建模、FK / IK 验证 |

---

## 通信协议

### 链路说明

RA6M5 侧通过 UART 驱动 WiFi 模块，并在上层使用一套自定义帧协议进行业务通信。

当前联调链路通常为：

```text
K230 / PC / 上位机
    ↓
WiFi / UDP
    ↓
WiFi 模块
    ↓ UART
RA6M5
```

### 帧格式

当前辅助联调脚本采用如下封包格式：

```text
[header] + [payload_len(2 bytes, big-endian)] + [payload]
```

默认帧头为：

```text
RENE:
```

### 心跳握手

为了确认 MCU 与上位机链路已建立，辅助脚本中使用了一个简单的就绪握手机制：

* 客户端发送：`HEART`
* 服务器返回：`ALIVE`

在联调脚本中，通常会在收到 `ALIVE` 之后再开始发送业务数据。

> 说明：`HEART / ALIVE` 属于当前联调脚本采用的握手机制，目的是帮助脚本确认链路可用；业务主数据仍以 `WEED` 与任务控制消息为主。

### 业务数据格式

当前主要业务包为：

```text
WEED:id,x,y,z,confidence
```

示例：

```text
WEED:3,0.12,0.25,0.18,0.91
```

字段含义如下：

| 字段           | 含义                |
| ------------ | ----------------- |
| `id`         | 杂草编号              |
| `x`          | 目标坐标 X            |
| `y`          | 目标坐标 Y            |
| `z`          | 目标坐标 Z            |
| `confidence` | 识别置信度，范围 `[0, 1]` |

### 激光流程握手

机械臂到位后，MCU 会发送：

```text
Ready to Laser
```

外部设备完成激光动作后，可返回：

```text
Laser OK
```

若未收到 `Laser OK`，当前流程也支持通过超时结束当前激光阶段。

### 完成标记

当前实现中，任务“全部完成”的触发使用了一个业务约定：

* 当 `WEED.id == 18` 时，主控将其视为“杂草已全部处理完毕”，并触发收尾流程

这属于当前仓库中的**协议约定**，后续你也可以根据自己的项目需求改为更明确的独立控制消息。

---

## 任务状态机

状态机定义位于 `src/app/fsm/`。

### 当前主状态

* `init`
* `idle`
* `plan_aim`
* `moving`
* `lasering`
* `finish`
* `fault`

### 典型流程

```text
init
  -> idle
  -> plan_aim
  -> moving
  -> lasering
  -> idle
```

全部完成后：

```text
idle
  -> finish
  -> idle
```

异常情况下：

```text
任意活动状态
  -> fault
  -> init / idle
```

### 各状态说明

#### `init`

完成任务上下文初始化与必要资源准备。

#### `idle`

等待外部输入，主要包括：

* 新的 `WEED` 识别结果
* 激光完成信号
* 全部完成标记

#### `plan_aim`

根据目标点生成瞄准位姿，执行可达性搜索与逆运动学求解。

#### `moving`

下发关节角到 DM 电机，并依据反馈判断是否到位。

#### `lasering`

通知外部设备执行激光动作，等待 `Laser OK` 或超时。

#### `finish`

执行整机收尾 / 回零流程。

#### `fault`

进入故障状态，记录错误信息并等待恢复。

> 当前实现中，故障恢复与任务完成的触发方式都带有一定项目约定色彩，适合作为样例，也建议在实际产品化时进一步规范为显式协议。

---

## 电机控制

DM 电机相关接口位于：

* `src/device/motor.h`
* `src/device/motor.c`

当前支持的典型能力包括：

* 电机使能 / 失能
* 位置速度控制
* 速度控制
* MIT 模式控制
* 请求反馈
* 位置 / 速度 / 扭矩 / 错误码读取

主流程中会周期性请求反馈，并将结果写入任务上下文，供状态机判断“是否到位”与“是否完成回零”。

---

## 运动学模块

机械臂运动学位于：

* `src/domain/arm_kine.h`
* `src/domain/arm_kine.c`

基础数学能力位于：

* `src/infra/matrix.h`
* `src/infra/matrix.c`

当前主要包括：

* 六轴机械臂 MDH 参数建模
* 正运动学（FK）
* 逆运动学（IK）
* 位姿表示与变换
* 与任务流程耦合的目标求解逻辑

这部分代码既可直接服务于嵌入式任务，也可作为后续迁移到 ROS / 上位机验证环境的算法基础。

---

## 快速开始

本仓库当前更推荐的使用方式是：

* **Windows**：使用 **RA Smart Configurator (RASC)** 配置并生成 HAL / FSP 相关代码，再通过 **VS Code + CMake** 完成编译，通过 **pyOCD** 完成烧录与调试。
* **Linux / WSL**：可使用 **CMake + ARM GNU Toolchain + Ninja** 完成构建；如需重新生成 RASC/FSP 相关代码，需要本机具备 **RA Smart Configurator (RASC)**。Renesas 提供了 RA Smart Configurator 以及对应的 Linux 安装入口，因此 Linux 端并非不可用，但是否作为你的主开发环境，取决于你是否真的需要在 Linux 上改动并重新生成 FSP/RASC 配置。

> 对于本仓库而言，仅仅在 Windows 端完成开发和调试，Linux 端不保证能够正常工作。

### 1. 准备依赖

你至少需要以下工具：

* **RA Smart Configurator (RASC)**：用于配置并生成 RA/FSP 相关代码
* **ARM GNU Toolchain**：用于交叉编译
* **CMake >= 3.16**
* **Ninja**
* **pyOCD**：用于烧录与调试

可选依赖：

* **Python**：用于运行联调脚本
* **MATLAB**：用于机械臂运动学验证
* **VS Code**：推荐作为当前仓库的主要编辑、构建与调试环境

### 2. 推荐工作流

#### Windows（推荐）

这是当前仓库最推荐、能够正常工作的流程：

1. 使用 **RA Smart Configurator (RASC)** 配置工程并生成 `ra/`、`ra_cfg/`、`ra_gen/` 等相关代码
2. 在 **VS Code** 中使用 **CMake Tools** 调用 `cmake/gcc.cmake` 进行配置与编译
3. 通过 **pyOCD** 烧录 `build/Debug/0_Debug.elf`
4. 如需在线调试，可结合 `cortex-debug` 与 pyOCD 使用

#### Linux / WSL（可选）

Linux / WSL 可以用于：

* 纯编译构建
* 阅读与修改非 RASC 生成区代码
* 静态分析 / clangd / CI
* 辅助脚本运行

如果你只是基于仓库现有的 `ra/`、`ra_cfg/`、`ra_gen/` 进行编译，那么 Linux 侧重点是准备好 ARM 工具链与构建环境即可。
如果你需要**重新生成** FSP / HAL 配置代码，则需要额外安装 **RA Smart Configurator (RASC)**。

### 3. 配置工具链路径

本仓库的 VS Code 配置与 CMake Kit 均依赖环境变量来定位 ARM GNU Toolchain。

#### 推荐环境变量

从你当前仓库配置来看，更建议统一使用：

```bash
ARM_GCC_TOOLCHAIN_PATH
```

因为 `.vscode/cmake-kits.json`、`settings.json` 中都在使用这个名字。

Linux / WSL：

```bash
export ARM_GCC_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain/bin
```

Windows PowerShell：

```powershell
$env:ARM_GCC_TOOLCHAIN_PATH = "C:/path/to/arm-gnu-toolchain/bin"
```

> 如果你的 `cmake/gcc.cmake` 仍然读取的是 `ARM_TOOLCHAIN_PATH`，建议在仓库内统一变量名；否则 README 里至少要说明两者的关系，避免使用者混淆。

### 4. 配置 RASC 路径

如果你的构建流程中包含 RASC 预生成 / 后生成步骤，请确保 `RASC_EXE_PATH` 可用。

Linux：

```bash
export RASC_EXE_PATH=/path/to/rasc
```

Windows PowerShell：

```powershell
$env:RASC_EXE_PATH = "C:/path/to/rasc.exe"
```

也可以在 CMake 配置时直接传入：

```bash
-DRASC_EXE_PATH=/path/to/rasc
```

> 若仓库中的默认路径是开发者本机路径，请务必按你的环境覆盖。

### 5. VS Code 配置说明

当前仓库已包含一套与实际工作流匹配的 VS Code 配置：

* `cmake-kits.json`：提供 ARM GCC + Ninja / MinGW Makefiles 的 CMake Kit
* `settings.json`：配置 CMake 构建目录、生成器偏好、`compile_commands.json` 导出、clangd 参数等
* `tasks.json`：提供 pyOCD 一键烧录任务
* `launch.json`：提供基于 `cortex-debug + pyOCD` 的调试配置

如果你直接使用 VS Code，推荐流程通常是：

1. 先设置好 `ARM_GCC_TOOLCHAIN_PATH`
2. 打开仓库，等待 CMake Tools 自动配置
3. 选择 `ARM GCC - Ninja` Kit
4. 编译生成 `build/Debug/0_Debug.elf`
5. 用任务或命令行执行 pyOCD 烧录
6. 需要时再使用 `launch.json` 中的调试配置启动调试

### 6. 命令行构建

#### Linux / WSL

```bash
cmake -S . -B build/Debug \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake \
  -DARM_TOOLCHAIN_PATH="$ARM_GCC_TOOLCHAIN_PATH" \
  -DRASC_EXE_PATH="$RASC_EXE_PATH"

cmake --build build/Debug
```

#### Windows PowerShell

```powershell
cmake -S . -B build/Debug `
  -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake `
  -DARM_TOOLCHAIN_PATH="$env:ARM_GCC_TOOLCHAIN_PATH" `
  -DRASC_EXE_PATH="$env:RASC_EXE_PATH"

cmake --build build/Debug
```

> 从你当前仓库配置看，README 最好显式说明：**环境变量名是 `ARM_GCC_TOOLCHAIN_PATH`，但传给 CMake 的变量是 `ARM_TOOLCHAIN_PATH`**。这是因为你的 VS Code Kit 已经把前者映射到了后者。

### 7. 烧录

命令行方式：

```bash
pyocd flash -t r7fa6m5bf build/Debug/0_Debug.elf
```

如果你使用 VS Code，也可以直接执行仓库自带任务：

* **烧录程序 (RA6M5)**

该任务本质上也是调用：

```bash
pyocd flash -t r7fa6m5bf ${workspaceFolder}/build/Debug/0_Debug.elf
```

### 8. 调试

仓库中已提供 `launch.json`，默认使用：

* `cortex-debug`
* `pyOCD`
* `arm-none-eabi-gdb`

当前示例配置中的 `gdbPath` 是一个**开发者本机的 Windows 路径**，开源前建议改成更通用的写法，或在 README 中注明需要用户自行修改。

例如，你可以在 README 中提示：

* 修改 `launch.json` 中的 `gdbPath`
* 确保其指向本机 ARM GNU Toolchain 自带的 `arm-none-eabi-gdb`

---

## 联调说明

如果你提供了 `wifi.py` / `main.py` 这类辅助脚本，可以用来快速验证：

* UDP 通信链路是否正常
* 帧协议解析是否正常
* `WEED` 数据是否能正确触发状态机
* `Ready to Laser / Laser OK` 流程是否连通

### 联调建议

当前示例脚本更适合做：

* 链路打通
* 协议联调
* 状态切换验证

如果要稳定测试“可达位姿 + IK 成功 + 电机正确到位”，建议不要直接使用过大范围的随机坐标，而应发送**位于机械臂有效工作空间内的已知目标点**。

---

## 适合谁使用

这个仓库尤其适合以下读者：

* 想学习 RA6M5 嵌入式机械臂控制工程的人
* 想参考“运动学 + 电机控制 + 状态机 + 通信协议”完整链路的人
* 想做比赛样机、实验室项目或课程设计的人
* 想从零整理一套可扩展的裸机控制架构的人

---

## 后续可扩展方向

你可以在此基础上继续扩展：

* 更规范的任务控制协议
* 更完善的异常恢复机制
* 更稳定的通信重连策略
* 视觉识别结果的时间戳 / 序列号机制
* 更严格的轨迹规划与速度约束
* ROS / 上位机可视化调试接口
* 多任务调度或 RTOS 化改造

---

## 许可证

本项目采用 **MIT License**，详见 `LICENSE` 文件。

---

## 致谢

感谢 Renesas FSP、ARM GNU Toolchain 以及相关开源工具链提供的支持。

如果这个项目对你有帮助，欢迎 Star、Issue 或 Pull Request。
