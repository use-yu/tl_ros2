# Python 示例程序

nanobind 生成的 `tl_interface.py`（底层 `_tl_host` 模块）提供了与 C++ 一致的 API。示例无需编译，直接运行即可。

---

## 环境要求

- **Python 3.10**（扩展模块编译目标为此版本）
- **Windows**: 绑定模块自包含（静态并入 SDK 实现），无需 `tl_host.dll` 或 DLL 搜索路径设置

## 运行

示例已内置平台自动检测逻辑，会自动选择对应平台的 SDK 绑定路径。

### Linux (x86_64)

```bash
cd examples/py
python3.10 ex_connect.py
```

也可在项目根目录直接运行：

```bash
python3.10 examples/py/ex_connect.py
```

### Linux (ARM64)

```bash
cd examples/py
python3.10 ex_connect.py
```

ARM64 示例自动选择 `lib/linux/arm64/python/` 下的绑定文件，用法与 x86 一致。

### Windows

```powershell
cd examples\py
py -3.10 ex_connect.py
```

项目根目录运行：

```powershell
py -3.10 examples\py\ex_connect.py
```

所有示例支持通过命令行参数指定控制器 IP（默认 `192.168.1.13`）：

```bash
python3.10 ex_connect.py -ip 192.168.1.13   # 等价于默认值
python3.10 ex_connect.py -ip 10.0.0.5       # 指定其他控制器
python3.10 ex_connect.py --ip 10.0.0.5      # --ip 与 -ip 等价
```

> **参数说明**: 参数名 `-ip` / `--ip`，后跟 IP 地址字符串；不传时使用默认 `192.168.1.13`。
> `ex_handeye_calibration.py` / `ex_handeye_verify.py` 默认取 `config/cal_config.yaml` 中的 `robot.ip`，命令行 `-ip` 参数优先于配置文件。

## 示例一览

| 示例 | 核心功能 | 适合谁看 |
|---|---|---|
| `ex_connect.py` | 连接控制器 → 获取 SDK 版本 → 断开 | 初次接触，验证通信是否正常 |
| `ex_info_query.py` | 查询位置、运行状态、关节角度、DH 参数 | 需要读取机器人信息时参考 |
| `ex_servo.py` | 伺服状态管理：就绪→上电→运行→下电→停止，速度查询/设置 | 了解伺服状态机与速度控制 |
| `ex_move_control.py` | MoveJ（关节运动）、MoveL（直线运动） | 基础运动控制 |
| `ex_queue_motion.py` | 队列运动完整流程：开启模式 → 编排 MoveJ/MoveL → 下发执行 | 需要预编排多条轨迹时参考 |
| `ex_servoj.py` | servoJ 关节实时跟踪（7000 端口高频透传，10ms 周期） | 需要实时关节控制时参考 |
| `ex_servo_movej.py` | 扩展伺服 MoveJ：250Hz 插值关节空间伺服运动，支持关节/笛卡尔坐标 | 需要平滑伺服运动时参考 |
| `ex_servo_movel.py` | 扩展伺服 MoveL：250Hz 插值笛卡尔空间直线伺服运动 | 需要平滑直线伺服时参考 |
| `ex_digital_io.py` | 数字 IO 输出/输入读写，模式切换 | IO 控制 |
| `ex_inspire_claw.py` | 因时电爪（Inspire EG2）Modbus RTU 控制：开度/速度/抓力设置，0=闭合完 1000=张开完 | 需要通过 Modbus 控制夹爪时参考 |
| `ex_omnipicker_claw.py` | OmniPicker 夹爪 Modbus RTU 控制：目标位置/速度/力矩/加减速度 + 运动触发，读状态与错误码 | 需要通过 Modbus 控制夹爪时参考 |
| `ex_job_control.py` | 创建作业 → 插入 MoveJ/L/C → 获取作业列表 → 运行作业 | 作业文件管理 |
| `ex_coord_tool.py` | 工具手参数、用户坐标系、坐标编号、当前坐标系设置 | 坐标系与工具管理 |
| `ex_coord_transform.py` | RPY ↔ 四元数 ↔ 旋转矩阵，TR ↔ R，正逆解 + servoJ 运动 | 坐标变换算法 |
| `ex_handeye_calibration.py` | 基于 Intel D435i 的眼在手（Eye-in-Hand）手眼标定 | 需要手眼标定时参考 |
| `ex_handeye_verify.py` | 手眼标定验证：识别棋盘格后自动解算目标位姿并前往（目标点偏移量可配置），验证手眼标定结果 | 验证手眼标定时参考 |
| `ex_error_callback.py` | 注册错误/警告回调 → 通过逆解不可达点触发回调 | 异常处理与消息监听 |
| `ex_error_zero.py` | 清除错误、设置指定关节零位 | 错误恢复与零位标定 |
| `ex_interference.py` | 干涉区检测 + MoveJ：get_tool_trigger_interfer 判断工具手位姿是否触发干涉区，触发则不执行运动 | 干涉区防护参考 |
| `ex_log_download.py` | 从控制器下载日志文件到本地 | 需要诊断/调试时参考 |
| `ex_teach_control.py` | 切换示教模式 → 关节点动 | 手动操作与示教 |
| `ex_teleop_f710_servoj.py` | F710 手柄遥操作（SDL2），通过 set_servoJ_pos 实现 250Hz 笛卡尔速度积分控制 | 需要手柄实时遥操作时参考 |
| `ex_set_tool_hand_number.py` | 设置/获取工具手编号 | 多工具手切换时参考 |
| `ex_track_queue_drag.py` | 轨迹记录与回放（track_record_playback 接口） | 需要示教复现时参考 |
| `ex_six_axis_force.py` | 六维力传感器通讯参数设置与数据读取 | 需要力传感器数据时参考 |
| `ex_six_axis_force_drag.py` | 六维力传感器拖动示教（与 C++ 示例功能对应） | 力控拖动示教参考 |

---

## 安全注意事项

以下接口涉及机器人标定、网络配置，误用可能导致断连、定位偏差、机械碰撞或设备损坏：

| 接口 | 风险 | 示例 | 说明 |
|---|---|---|---|
| `set_axis_zero_position` / `set_sync_axis_zero_position` | **极高** — 修改关节零位标定，所有运动基准偏移 | `ex_error_zero.py` | 默认已注释，仅出厂校准或更换关节后使用 |
| `set_zero_pos_deviation` | **极高** — 修改零位偏差值，直接改变关节零点 | — | 与零位标定同等危险 |
| `set_robot_dh_param` | **极高** — 修改 DH 参数，运动学解算完全错误，可能导致不可预测运动 | — | 仅在更换连杆或重新标定后使用 |
| `set_robot_joint_param` | **极高** — 修改关节参数（限位、减速比等），可能导致超限或碰撞 | — | 确认参数来源，错误值会导致物理损坏 |
| `set_controller_ip` / `set_controller_network_config` | **极高** — 修改控制器 IP 或网络配置，立即断连且可能无法重新连接 | — | 需通过其他方式恢复网络时极为麻烦 |
| `set_result_for_DH` | **极高** — 应用 DH 标定结果，错误标定导致运动学完全混乱 | — | 仅在完整的 DH 标定流程后调用 |
| `set_collision_para` | **高** — 碰撞检测参数设得过大则碰撞不检测，设得过小则频繁误触发 | — | 在安全前提下通过实验调整 |
| `set_tool_hand_param` | **中** — 工具参数影响 TCP 位姿计算 | `ex_coord_tool.py` | 确认工具实际尺寸后再设置 |
| `set_tool_coordinate_range` | **中** — 工具坐标范围限制，错误值限制关节运动 | — | 确认工具可达范围后再设置 |
| `set_user_coordinate_data` | **中** — 用户坐标系影响路径规划 | `ex_coord_tool.py` | 确认坐标系定义后再设置 |
| `set_position_dragParams` / `set_darg_mode` | **中** — 拖动示教参数，错误值导致拖动过重或过轻 | — | 根据实际负载和工具配置 |
| `set_four_point_mark` | **中** — 四点标定标记，错误标记影响工具/用户坐标标定 | — | 在完整标定流程中按步骤使用 |
| `set_global_position` / `set_global_sync_position` | **低** — 修改全局路点数据，错误路点导致运动到意外位置 | `ex_global_position.py` | 确认路点值在安全工作范围内 |
| `set_reconnect` | **低** — 重连机制，网络异常时行为可能不符合预期 | — | 确认网络策略后再启用 |

> **通用原则**: 在真实机械臂上运行任何示例前，请先确认工作区域安全、速度设置在合理范围内，并随时准备按下急停按钮。
