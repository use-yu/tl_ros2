# C++ 示例程序

通过 CMake 一键编译，运行时 CMake 已自动嵌入 RPATH，无需额外设置 `LD_LIBRARY_PATH`。

---

## 编译（CMake Presets，推荐）

项目提供了 `CMakePresets.json`，支持预设一键配置和编译：

### Linux x86_64

```bash
cd examples/cpp

# 配置 + 编译
cmake --preset linux-native
cmake --build --preset linux-native

# 运行
build/linux-native/ex_connect
```

### Linux ARM64（交叉编译）

```bash
# 安装交叉编译工具链（仅首次）
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

cd examples/cpp

# 配置 + 编译
cmake --preset linux-arm64
cmake --build --preset linux-arm64

# 运行（在 ARM64 目标上）
build/linux-arm64/ex_connect
```

已安装 ARM64 SDL2（`sudo apt install libsdl2-dev:arm64`）时 `ex_teleop_f710_servoj` 会自动参与编译；否则自动跳过。

### Windows 编译（Visual Studio）

```powershell
cd examples\cpp

# 配置 + 编译（Visual Studio，x64 Release）
cmake --preset windows-x86
cmake --build --preset windows-x86

# 运行
build\windows-x86\ex_connect.exe
```

> **注意**：运行时需要确保 `lib/windows/tl_host.dll` 在可执行文件所在目录或 `PATH` 环境变量中。
> CMake 已自动将 `tl_host.dll` 拷贝到构建输出目录，无需手动处理。

### 同时编译所有平台

```bash
cd examples/cpp
cmake --preset linux-native && cmake --build --preset linux-native
cmake --preset linux-arm64 && cmake --build --preset linux-arm64
```

## 编译（传统方式）

```bash
cd examples/cpp/build
cmake ..
make
```

编译产物为可执行文件，文件名对应源文件名（如 `ex_connect.cpp` → `ex_connect`）。

## 运行

```bash
# 在 build 目录下直接运行
./ex_connect

# 或在项目根目录运行
examples/cpp/build/ex_connect
```

所有示例支持通过命令行参数指定控制器 IP（默认 `192.168.1.13`）：

```bash
./ex_connect -ip 192.168.1.13          # 等价于默认值
./ex_connect -ip 10.0.0.5              # 指定其他控制器
./ex_connect --ip 10.0.0.5             # --ip 与 -ip 等价
```

> **参数说明**: 参数名 `-ip` / `--ip`，后跟 IP 地址字符串；不传或传错参数位置时使用默认 `192.168.1.13`。
> 部分示例还支持自身额外的位置参数（如 `ex_tool_coord_ik [-c base|user] [X Y Z Rx Ry Rz]`），`-ip` 可与它们任意组合。

> **SDL2 说明**：如果未安装 SDL2，`ex_teleop_f710_servoj` 会自动跳过。安装方式：
> - Linux 本机：`sudo apt install libsdl2-dev`
> - ARM64 交叉编译：`sudo apt install libsdl2-dev:arm64`
> - Windows：从 [SDL2 官网](https://github.com/libsdl-org/SDL/releases) 下载开发库，设置 `SDL2_DIR` 为 CMake 配置路径
> - 自定义路径：`cmake -DSDL2_DIR=/path/to/sdl2/lib/cmake/SDL2 ...`

## 示例一览

| 示例 | 核心功能 | 适合谁看 |
|---|---|---|
| `ex_connect` | 连接控制器 → 获取 SDK 版本 → 断开 | 初次接触，验证通信是否正常 |
| `ex_info_query` | 查询位置、运行状态、关节角度、DH 参数 | 需要读取机器人信息时参考 |
| `ex_servo` | 伺服状态管理：就绪→上电→运行→下电→停止，速度查询/设置 | 了解伺服状态机与速度控制 |
| `ex_move_control` | MoveJ（关节运动）、MoveL（直线运动） | 基础运动控制 |
| `ex_queue_motion` | 队列运动完整流程：开启模式 → 编排 MoveJ/MoveL → 下发执行 | 需要预编排多条轨迹时参考 |
| `ex_servoj` | servoJ 关节实时跟踪（7000 端口高频透传，10ms 周期） | 需要实时关节控制时参考 |
| `ex_servo_movej` | 扩展伺服 MoveJ：250Hz 插值关节空间伺服运动，支持关节/笛卡尔坐标 | 需要平滑伺服运动时参考 |
| `ex_servo_movel` | 扩展伺服 MoveL：250Hz 插值笛卡尔空间直线伺服运动 | 需要平滑直线伺服时参考 |
| `ex_digital_io` | 数字 IO 输出/输入读写，模式切换 | IO 控制 |
| `ex_inspire_claw` | 因时电爪（Inspire EG2-4C2）Modbus RTU 控制：开度/速度/抓力设置，0=闭合完 1000=张开完 | 需要通过 Modbus 控制夹爪时参考 |
| `ex_omnipicker_claw` | OmniPicker 夹爪 Modbus RTU 控制：目标位置/速度/力矩/加减速度 + 运动触发，读状态与错误码 | 需要通过 Modbus 控制夹爪时参考 |
| `ex_job_control` | 创建作业 → 插入 MoveJ/L/C → 获取作业列表 → 运行作业 | 作业文件管理 |
| `ex_coord_tool` | 工具手参数、用户坐标系、坐标编号、当前坐标系设置 | 坐标系与工具管理 |
| `ex_coord_transform` | RPY ↔ 四元数 ↔ 旋转矩阵，TR ↔ R，正逆解 + servoJ 运动 | 坐标变换算法 |
| `ex_user_coord_compare` | 同一坐标值在两个用户坐标系（与基重合 / 绕 Z 顺时针转 90°）下各运动一次，对比实际到达位置 | 理解用户坐标系语义 |
| `ex_user_coord_ik` | 工具手 + 用户坐标系下逆解关节角，并用 MoveJ 下发运动 | 用户坐标逆解参考 |
| `ex_error_callback` | 注册错误/警告回调 → 通过逆解不可达点触发回调 | 异常处理与消息监听 |
| `ex_collision_detect` | 碰撞检测参数获取/设置（`get_collision`/`set_collision`），碰撞后清错自动恢复，碰撞阈值在示教器上设置 | 碰撞防护与自动恢复 |
| `ex_robot_state_callback` | 7000 端口状态回调测试（TCP + UDP 双模式） | 需要实时状态订阅时参考 |
| `ex_error_zero` | 清除错误、设置指定关节零位 | 错误恢复与零位标定 |
| `ex_global_position` | 设置/查询全局路点 | 全局路点管理 |
| `ex_tool_hand_number` | 设置/获取工具手编号 | 多工具手切换时参考 |
| `ex_track_queue_drag` | 拖拽示教 + 轨迹记录/回放 | 需要示教复现时参考 |
| `ex_log_download` | 从控制器下载日志文件到本地 | 需要诊断/调试时参考 |
| `ex_teach_control` | 切换示教模式 → 关节点动 | 手动操作与示教 |
| `ex_teleop_f710_servoj` | F710 手柄遥操作（SDL2），通过 set_servoJ_pos 实现 250Hz 笛卡尔速度积分控制 | 需要手柄实时遥操作时参考 |
| `ex_tool_coord_move` | 工具坐标系下增量点动，模拟示教器工具坐标系点动效果（servoJ + IK） | 需要工具坐标定向运动时参考 |
| `ex_tool_coord_ik` | 末端系逆解：接收末端坐标系下目标位姿（如手眼变换结果），复合当前基坐标（基坐标系）后逆解关节角并用 MoveJ 下发运动，内置末端系逆解接口 `solve_endframe_ik` | 手眼/末端坐标逆解参考 |
| `ex_admittance_control` | 导纳（柔顺）控制：力传感器数据 → 导纳补偿 → 伺服运动 | 需要柔顺/力控交互时参考 |
| `ex_constant_force` | 恒力驻留控制（无扫动） | 需要恒力接触场景时参考 |
| `ex_massage` | 恒力按摩控制（直接力控） | 需要力控移动场景时参考 |
| `ex_six_axis_force` | 六维力传感器通讯参数设置与数据读取 | 需要力传感器数据时参考 |
| `ex_six_axis_force_drag` | 六维力传感器拖动示教：设置通讯参数并启用拖拽，进入力矩拖拽模式后实时读取六维力数据，可按指定时长或检测拖拽结束自动退出 | 力控拖动示教参考 |

---

## 安全注意事项

以下接口涉及机器人标定、网络配置，误用可能导致断连、定位偏差、机械碰撞或设备损坏：

| 接口 | 风险 | 示例 | 说明 |
|---|---|---|---|
| `set_axis_zero_position` / `set_sync_axis_zero_position` | **极高** — 修改关节零位标定，所有运动基准偏移 | `ex_error_zero` | 默认已注释，仅出厂校准或更换关节后使用 |
| `set_zero_pos_deviation` | **极高** — 修改零位偏差值，直接改变关节零点 | — | 与零位标定同等危险 |
| `set_robot_dh_param` | **极高** — 修改 DH 参数，运动学解算完全错误，可能导致不可预测运动 | — | 仅在更换连杆或重新标定后使用 |
| `set_robot_joint_param` | **极高** — 修改关节参数（限位、减速比等），可能导致超限或碰撞 | — | 确认参数来源，错误值会导致物理损坏 |
| `set_controller_ip` / `set_controller_network_config` | **极高** — 修改控制器 IP 或网络配置，立即断连且可能无法重新连接 | — | 需通过其他方式恢复网络时极为麻烦 |
| `set_result_for_DH` | **极高** — 应用 DH 标定结果，错误标定导致运动学完全混乱 | — | 仅在完整的 DH 标定流程后调用 |
| `set_collision_para` | **高** — 碰撞检测参数设得过大则碰撞不检测，设得过小则频繁误触发 | — | 在安全前提下通过实验调整 |
| `set_collision` | **高** — 碰撞检测系数/伺服执行响应时间设得过大则碰撞不检测，过小则频繁误触发 | `ex_collision_detect` | 示例将各轴系数设为 98，响应时间不建议设置（沿用原值） |
| `set_tool_hand_param` | **中** — 工具参数影响 TCP 位姿计算 | `ex_coord_tool` | 确认工具实际尺寸后再设置 |
| `set_tool_coordinate_range` | **中** — 工具坐标范围限制，错误值限制关节运动 | — | 确认工具可达范围后再设置 |
| `set_user_coordinate_data` | **中** — 用户坐标系影响路径规划 | `ex_coord_tool` | 确认坐标系定义后再设置 |
| `set_position_dragParams` / `set_darg_mode` | **中** — 拖动示教参数，错误值导致拖动过重或过轻 | — | 根据实际负载和工具配置 |
| `set_four_point_mark` | **中** — 四点标定标记，错误标记影响工具/用户坐标标定 | — | 在完整标定流程中按步骤使用 |
| `set_global_position` / `set_global_sync_position` | **低** — 修改全局路点数据，错误路点导致运动到意外位置 | `ex_global_position` | 确认路点值在安全工作范围内 |
| `set_reconnect` | **低** — 重连机制，网络异常时行为可能不符合预期 | — | 确认网络策略后再启用 |

> **通用原则**: 在真实机械臂上运行任何示例前，请先确认工作区域安全、速度设置在合理范围内，并随时准备按下急停按钮。
