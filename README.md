# Renesas-DM-Arm (RA6M5)

本仓库包含基于 Renesas RA6M5 (R7FA6M5BF2CBG) 的六轴机械臂源码，采用 Renesas FSP + CMake 构建系统。工程包括底层 HAL（由 RASC/FSP 生成）、电机驱动（DM）、WiFi/UDP 通信、CAN 接口、臂运动学实现（MATLAB 工具）以及用于主机/仿真的 Python 工具。

主要特点
- 目标设备：R7FA6M5BF2CBG (RA6M5, Cortex-M33)
- 构建系统：CMake + Ninja（使用 `cmake/gcc.cmake` 工具链文件）
- 生成代码：`ra_gen/`、`ra/`（FSP）由 RASC/FSP 生成，请勿直接修改
- 应用入口：`ra_gen/main.c` -> `hal_entry()` 在 `src/hal_entry.c`

快速开始
1) 前提（示例）
	- 安装 CMake (>= 3.16)、Ninja
	- 安装 ARM GNU 工具链（包含 `arm-none-eabi-gcc`），并记下 bin 目录路径
	- Python 3（用于工具脚本）和 `pyocd`（用于通过 SWD 烧录）
	- 可选：MATLAB（用于 `matlab/` 下的运动学脚本）

2) 命令行构建（示例，Windows PowerShell）
```powershell
set ARM_TOOLCHAIN_PATH=C:/path/to/your/toolchain/bin
cmake -DARM_TOOLCHAIN_PATH="$env:ARM_TOOLCHAIN_PATH" -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake -G Ninja -B build/Debug
cmake --build build/Debug
```

3) 烧录到设备
	- 使用 pyOCD（项目任务也提供了一个 VS Code task）
```powershell
pyocd flash -t r7fa6m5bf build/Debug/0_Debug.elf
```
	- 或在 IDE 中使用调试/烧录插件，确保选择目标 `R7FA6M5BF`。

运行与调试说明
- 可执行文件：`build/Debug/0_Debug.elf`
- 程序入口位于 `ra_gen/main.c`，其会调用 `hal_entry()`（在 `src/hal_entry.c` 中实现）。
- 默认设备网络配置（见 `src/hal_entry.c`）：
	- WiFi SSID: `K230`
	- 密码: `12345678`
	- UDP 目标 IP: `192.168.169.1`
	- 远端端口: `8080`
	- 本地端口: `5000`

开发目录说明（简要）
- `ra/`：Renesas FSP 源与第三方库（CMSIS、驱动）
- `ra_gen/`：由 RASC/FSP 生成的 HAL/启动代码（`hal_data.h`、`main.c` 等）——请勿手动修改
- `src/`：应用层代码（`hal_entry.c`、`app/`、`device/`、`infra/`、`domain/`、`platform/` 等）
- `script/`：链接脚本模板（`fsp.ld` 等）
- `cmake/`：CMake 工具链与生成配置（`gcc.cmake`、`Generated*` 等）
- `build/`：CMake 输出目录（二进制、编译中间文件）
- `k230/`：主机/测试脚本（Python，用于与设备通过 UDP 通信）
- `matlab/`：用于运动学求解与验证的 MATLAB 脚本

主机/仿真工具
- `k230/main.py` 与 `k230/wifi.py` ：主机侧 UDP 通信示例，使用帧格式 `RENE:` + 2 字节长度 + payload。该脚本可作为与设备联调的参考实现（详见 `k230/wifi.py` 注释）。

生成代码与注意事项
- `ra_gen/`、`ra/` 下的文件由工具生成，修改应通过 RASC/FSP 配置而不是直接编辑生成文件。
- 在 CMake 工具链中请设置 `ARM_TOOLCHAIN_PATH` 到工具链 `bin` 目录（参考 `cmake/gcc.cmake`）。
- 避免在项目路径或工具链路径中使用空格（可能会导致 CMake/工具链解析问题）。

常见命令速查
- 配置（Debug）：
```bash
cmake -DARM_TOOLCHAIN_PATH="/your/toolchain/path" -DCMAKE_TOOLCHAIN_FILE=cmake/gcc.cmake -G Ninja -B build/Debug
```
- 构建：
```bash
cmake --build build/Debug
```
- 烧录：
```bash
pyocd flash -t r7fa6m5bf build/Debug/0_Debug.elf
```

后续事项 / 贡献
- 如果要更改外设配置，请在 RASC/FSP 中修改配置并重新生成 `ra_gen/`。
- 欢迎提交 issue 或 pull request，描述所做更改与测试步骤。

许可证
- MIT License（详见 LICENSE 文件）

更多信息
- 设备目标与链接脚本：见 `buildinfo.json`（`targetDeviceName`、`linkerScript` 等字段）
- 入口与初始化：见 [src/hal_entry.c](src/hal_entry.c#L1)
