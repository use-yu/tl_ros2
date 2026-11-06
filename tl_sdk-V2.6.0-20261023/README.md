# TL Robot Arm SDK

通过 **C++** 或 **Python** 远程控制机器人控制器的 SDK。基于 TCP/IP 通信，提供运动控制、IO 控制、作业管理、Modbus 等完整接口。

对外交付为**单一合并库** `libtl_host.so`（Windows: `tl_host.dll`）+ `tl_host.lib` + 自包含头文件（`include/` 下 `tl_api.h` 等）：所有接口位于 `namespace tl`，提供连接/运动/伺服插值/恒力/IO/作业/Modbus/轨迹等常用接口。Python 绑定为 nanobind 自产单模块（`_tl_host`），入口 `tl_interface.py`。

---

## SDK 能做什么

- **关节运动**（MoveJ）和 **直线运动**（MoveL）—— 发送目标位置，非阻塞执行
- **伺服实时跟踪**（servoJ）—— 10ms 周期高频透传，实时控制每个关节角度
- **周期同步模式**（CSV / CST）—— 按固定周期下发关节速度或扭矩（7000 端口）
- **扩展伺服运动**（`servo_movej` / `servo_movel`）—— 基于 servoJ 的 250Hz 后台插值封装，支持关节坐标（`coord=0`）和笛卡尔坐标（`coord=1`）
- **队列运动** —— 预编排多条轨迹，一次性下发执行
- **IO 控制** —— 读写数字/模拟输入输出
- **作业管理** —— 创建、编辑、运行机器人作业文件
- **Modbus 通信** —— 主站/从站、TCP/RTU，读写外部设备寄存器
- **信息查询** —— 当前位置、伺服状态、运行状态、DH 参数、关节温度等
- **六维力传感器** —— 通讯参数设置与数据读取
- **轨迹记录与回放** —— 拖拽示教记录轨迹，回放复现
- **拖动轨迹文件管理** —— 控制器内拖动轨迹批量导出/导入本地目录，按文件名回放或删除
- **状态回调** —— 注册错误/警告/机器人状态回调，异步接收控制器事件
- **夹爪 / 灵巧手控制** —— 通过 Modbus（TCP/RTU）控制末端执行器
- **恒力控制** —— 恒力驻留/扫动（后台线程恒力闭环，运行中可实时 `set_constforce_target_pose` 更新目标位姿）
- **Python 调用** —— nanobind 自产绑定（`tl_interface.py` + `_tl_host`），函数名与 C++ 一致

---

## 示例程序

- C++ 示例 → [`examples/cpp/README.md`](examples/cpp/README.md)
- Python 示例 → [`examples/py/README.md`](examples/py/README.md)

所有示例覆盖连接控制、运动控制、队列运动、servoJ 实时跟踪、扩展伺服运动、IO、Modbus、作业管理、坐标变换、错误处理、力传感器、手柄遥操作、轨迹记录回放、状态回调、夹爪控制、恒力控制等功能。

---

## 在自己的项目中使用 SDK

### 目录结构

SDK 交付的核心文件：

```
your_project/
├── include/                  # 对外头文件（唯一接口，namespace tl，自包含）
│   ├── tl_api.h              #   汇总入口（一站式包含）
│   ├── tl_interface.h        #   连接/伺服/运动/坐标/传感器
│   ├── tl_types.h            #   常用类型定义
│   └── ...                   #   tl_force / tl_servo_ext / tl_admittance / tl_constant_force / tl_io / tl_job_operate / tl_modbus / tl_queue_operate / tl_track 等
├── lib/                      # 平台动态库 + Python 绑定
│   ├── linux/
│   │   ├── x86/              # Linux x86_64
│   │   │   ├── libtl_host.so           # 合并库（全部 C++ API + 扩展伺服）
│   │   │   └── python/                 # nanobind 绑定
│   │   │       ├── tl_interface.py     #   统一入口
│   │   │       └── _tl_host.cpython-310-*.so  # nanobind 扩展模块
│   │   └── arm64/            # Linux ARM64
│   │       ├── libtl_host.so           # 合并库（交叉编译）
│   │       └── python/                 # nanobind 绑定（aarch64 平台标签）
│   │           ├── tl_interface.py     #   统一入口
│   │           └── _tl_host.cpython-310-*.so  # nanobind 扩展模块
│   └── windows/              # Windows x86_64
│       ├── tl_host.dll                 # 动态库（Release）
│       ├── tl_host.lib                 # 导入库（Release）
│       ├── debug/                      # 调试版库（Windows 专有）
│       │   ├── tl_host.dll             #   动态库（Debug）
│       │   ├── tl_host.lib             #   导入库（Debug）
│       │   └── tl_host.pdb             #   调试符号（供调试器定位 SDK 内部调用）
│       └── python/                     # nanobind 绑定（Release）
│           ├── tl_interface.py         # 统一入口
│           └── _tl_host.cp310-win_amd64.pyd  # nanobind 扩展模块
├── docs/                     # 产品说明文档（PDF，打包时由 .md 生成）
├── examples/                 # 示例程序
│   ├── cpp/                  # C++ 示例源码 + CMakeLists.txt
│   └── py/                   # Python 示例
├── README.md                 # 本说明（安装 / 用法 / 核心概念）
└── CHANGELOG.md              # 版本变更记录
```

### C++ 项目

`#include` 使用 SDK 接口只需一行：

```cpp
#include "tl_api.h"  // 封装层汇总头（namespace tl，常用接口）

using namespace tl;  // 或显式 tl:: 前缀
```

#### 推荐构建方式：CMake Presets

项目提供 `CMakePresets.json`，一键配置和编译：

**Linux x86_64：**

```bash
cmake --preset linux-native
cmake --build --preset linux-native
```

**Linux ARM64（交叉编译）：**

```bash
cmake --preset linux-arm64
cmake --build --preset linux-arm64
```

**Windows（Visual Studio x64 Release）：**

```powershell
cmake --preset windows-x86
cmake --build --preset windows-x86
```

> 所有 preset 均使用 Ninja 生成器，需预装 `ninja-build`（Ubuntu: `sudo apt install ninja-build`；Windows: VS 自带）。

运行时 CMake 已自动嵌入 RPATH（Linux）或拷贝 DLL 到输出目录（Windows），无需额外设置。

#### 传统编译方式

**Linux：**

```bash
g++ -std=c++17 my_program.cpp -o my_program -I./include ./lib/linux/x86/libtl_host.so -lpthread
export LD_LIBRARY_PATH=lib/linux/x86:$LD_LIBRARY_PATH
./my_program
```

**Windows（MSVC）：**

```powershell
cl /EHsc /std:c++17 my_program.cpp /I.\include .\lib\windows\tl_host.lib
```

编译后需将 `tl_host.dll` 放在与可执行文件同一目录或 `PATH` 中。

**Windows（MSVC，调试版）：** 需要在调试器中单步跟踪 SDK 内部调用时，改用 `lib/windows/debug/` 下的库（自建的调试版程序须与调试版库配套使用）：

```powershell
cl /EHsc /MDd /Od /Zi /std:c++17 my_program.cpp /I.\include .\lib\windows\debug\tl_host.lib
```

调试版依赖 Visual Studio 的调试运行库（仅装了 Visual Studio 的机器可用），并把同目录下的 `tl_host.pdb` 放在 `tl_host.dll` 旁，调试器才能定位到库内代码。正式发布请使用 `lib/windows/` 下的 Release 版库。Linux 平台只提供 Release 版库。

**Windows（MinGW）：**

```bash
g++ -std=c++17 my_program.cpp -o my_program.exe -I./include ./lib/windows/tl_host.dll
```

### Python 项目

Python 绑定为 nanobind 自产单模块 `_tl_host`，统一入口 `tl_interface.py`（函数名与 C++ 一致，含伺服插值/恒力）。linux-x86 / linux-arm64 / Windows 三平台均为 nanobind 模块（ARM64 交叉编译产物带 `aarch64` 平台标签），API 签名一致：

```python
import sys, os, time, platform

_script_dir = os.path.dirname(os.path.abspath(__file__))
# 自动检测平台，添加对应 SDK Python 绑定路径
if platform.system() == "Windows":
    sys.path.append(os.path.join(_script_dir, "lib/windows/python"))
    # 绑定模块 _tl_host 已自包含（静态并入 SDK 实现），无需 tl_host.dll / add_dll_directory
elif platform.machine() in ("aarch64", "arm64"):
    sys.path.append(os.path.join(_script_dir, "lib/linux/arm64/python"))
else:
    sys.path.append(os.path.join(_script_dir, "lib/linux/x86/python"))

from tl_interface import *  # 全部封装层 API（含伺服插值/恒力）

# 连接控制器（与 SDK 原生 API 一致）
sock_tcp = connect_robot("192.168.1.13", "6001")
sock_udp = connect_robot("192.168.1.13", "7000")

# 伺服就绪 → 上电
set_servo_state(sock_tcp, 1)
time.sleep(0.5)
set_servo_poweron(sock_tcp)

# 扩展伺服运动：后台 250Hz 线程插值，非阻塞入队
open_servo_movej(sock_udp, vmax_pct=30)
servo_movej(sock_tcp, sock_udp, [0, 0, 90, 0, 90, 0], coord=0)  # 立即返回
wait_servo()                                                      # 阻塞等待完成
close_servo_movej(sock_udp)

# 下电 → 断开
set_servo_poweroff(sock_tcp)
disconnect_robot(sock_tcp)
disconnect_robot(sock_udp)
```

扩展接口（`tl::`，并入合并库 libtl_host.so）：

| 函数 | 说明 |
|---|---|
| `open_servo_movej` / `close_servo_movej` | 开启/关闭关节空间伺服跟踪模式（传入 7000 端口 socket） |
| `open_servo_movel` / `close_servo_movel` | 开启/关闭笛卡尔空间伺服跟踪模式（与 movej 共享后台线程） |
| `servo_movej(socket_tcp, socket_udp, target, coord)` | 关节空间伺服运动，支持关节坐标（coord=0）和笛卡尔坐标（coord=1） |
| `servo_movel(socket_tcp, socket_udp, target_pose, coord)` | 笛卡尔空间直线伺服运动，支持关节坐标（coord=0）和笛卡尔坐标（coord=1） |
| `wait_servo` | 阻塞等待所有排队运动执行完毕 |
| `servo_busy` | 查询是否有运动正在执行或排队 |
| `cancel_servo` | 取消当前运动并清空排队任务（不会关闭伺服） |
| `open_servo_csv_mode` / `close_servo_csv_mode` / `set_servo_csv_velocity` | 周期同步速度模式：按周期下发 7 元素关节速度（°/s） |
| `open_servo_cst_mode` / `close_servo_cst_mode` / `set_servo_cst_torque` | 周期同步扭矩模式：按周期下发 7 元素关节扭矩（额定扭矩的 0.1%） |

> **⚠️ Python 版本要求**
> `.pyd`/`.so` 扩展模块编译目标为 **Python 3.10**。
> - Linux: `python3.10 examples/py/ex_connect.py`
> - Windows: `py -3.10 examples\py\ex_connect.py`

---

## 核心概念

### 双端口连接

SDK 使用两个 TCP 端口分工：

- **6001 端口** —— 控制与状态。所有运动指令、IO、作业、Modbus 等常规接口
- **7000 端口** —— 伺服高频模式。`open_servoJ` / `set_servoJ_pos` / `close_servoJ`（10ms 周期透传）、周期同步模式（`open_servo_csv_mode` / `open_servo_cst_mode` 系列）、伺服插值（`open_servo_movej` / `open_servo_movel` 等）、恒力驻留（`open_constforce`）均使用该端口

如果只做常规运动控制，只需要连接 6001 端口。servoJ 实时跟踪、CSV/CST 周期同步模式、伺服插值（servo_movej/movel）和恒力驻留控制才需要额外连接 7000 端口。

> **⚠️ 7000 端口 servoJ 会话互斥**：`open_servoJ`（servoJ 透传）、`open_servo_movej` / `open_servo_movel`（伺服插值）、`open_constforce`（恒力驻留）三组接口共用 7000 端口的同一个 servoJ 会话，**任意时刻只能开启其中一组**，同时下发会互相冲突、破坏运动。互斥检测仅恒力侧进行（`open_constforce` 会检查异步伺服模式并返回 `OPERATION_NOT_ALLOWED`），反向不受保护，切换时必须先关闭当前组（`close_servoJ` / `close_servo_movej(movel)` / `close_constforce`）再开启另一组。

### 上电 / 下电

上电必须分两步走：

```cpp
set_servo_state(sock, 1);          // ① 伺服就绪
std::this_thread::sleep_for(std::chrono::milliseconds(500));
set_servo_poweron(sock);           // ② 上电使能
```

下电后网络会自动断开，此时 `disconnect_robot` 返回 -1 是正常现象，无需处理。

### 运动控制特点

- **非阻塞**：`robot_movej` / `robot_movel` 发出指令后立即返回，需通过 `sleep_for` 等待机械臂到位
- **坐标系**：0=关节 1=直角 2=工具 3=用户
- **队列模式**：先 `queue_motion_set_status(true)` 开启，多次 `queue_motion_push_back_moveJ/moveL` 添加轨迹点，最后 `queue_motion_send_to_controller(sock, size)` 一次性下发
- **扩展伺服**：`servo_movej` / `servo_movel` 在后台线程按 250Hz 插值执行，支持 `wait_servo()` 阻塞等待完成

### 清错后恢复

清除错误后不能直接上电，需按顺序操作：

```
clear_error → set_servo_poweroff → set_servo_state(1) → set_servo_poweron
```

### servoJ vector 长度约定

SDK 底层协议统一使用 **7 元素 vector** 兼容 6 轴和 7 轴机械臂：

- **6 轴机器人**：第 7 元素（索引 6）补 0（外部轴）
- **7 轴机器人**：第 7 元素是真实 J7 关节角

涉及接口：`open_servoJ` 的 `vmax/amax/jmax`、`set_servoJ_pos` 的 `q`。

---

## 典型调用流程

### 基础运动

```
连接控制器（6001 端口）
  ↓
清错
  ↓
伺服就绪 → 上电使能
  ↓
运动控制 / IO / 作业 / 查询 ...
  ↓
下电
  ↓
断开连接
```

### 扩展伺服运动（伺服插值，tl::servo_movej / servo_movel）

```
连接 6001 端口 + 连接 7000 端口
  ↓
伺服就绪 → 上电使能
  ↓
open_servo_movej / open_servo_movel（开启后台线程）
  ↓
servo_movej / servo_movel（入队，立即返回）
  ↓
wait_servo()（阻塞等待完成）
  ↓
close_servo_movej / close_servo_movel
  ↓
下电 → 断开连接
```

### 恒力驻留控制（tl::open_constforce 等）

```
连接 6001 端口 + 连接 7000 端口
  ↓
清错 → 伺服就绪 → 上电使能 → 切换运行模式(2)
  ↓
运动到接触位（robot_movej / robot_movel 或伺服插值）
  ↓
open_constforce(sock_6001, sock_7000, base_pose, admittance, sensor_callback, user_data, params)（立即返回，后台闭环）
  ↓
运行中可实时 set_constforce_target_pose 更新目标位姿
  ↓
close_constforce()（必须调用，join 后台线程）
  ↓
下电 → 断开连接
```

> 恒力控制需要六维力传感器正常连接且机器人已到接触位；参数与注意事项见 `docs/天链机械臂SDK使用说明文档_CPP版.md` / `docs/天链机械臂SDK使用说明文档_PY版.md`。

---

## 注意事项

- **默认 IP**：`192.168.1.13`，请确保控制器网络可达
- Python 绑定为 nanobind 自产单模块 `_tl_host`（`tl_interface.py` 为入口），函数名与 C++ 封装层一致
- 部分接口仅在特定机器人模式下可用（示教/运行/远程），详见头文件注释
- 动态库位于 `lib/linux/x86/`（Linux x86_64）、`lib/linux/arm64/`（Linux ARM64）、`lib/windows/`（Windows）；`lib/` 为构建产物目录，不纳入版本控制，构建后自动同步
- Linux CMake 构建已嵌入 RPATH 无需额外设置；Windows 需将 `tl_host.dll` 放在可执行文件同目录或 `PATH` 中
- Python 绑定需 Python 3.10 环境加载，示例已内置平台自动检测逻辑
- **Python 绑定自包含**：`_tl_host` 静态并入封装层源码与控制器协议静态库，运行时不需要 `libtl_host.so` / `tl_host.dll`，也无需 RUNPATH / `LD_LIBRARY_PATH` / `os.add_dll_directory()`，拷贝 `python/` 目录即用
- ARM64 Python 绑定为 nanobind 交叉编译产物（`_tl_host.cpython-310-aarch64-linux-gnu.so`），可直接 `import _tl_host`，API 与 x86/Windows 一致

---

## 版本历史

详见 [CHANGELOG.md](CHANGELOG.md)。
