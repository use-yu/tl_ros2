# Changelog

## [2.6.0] - 2026-10-23

### 依赖版本
- **对应控制器版本**: rtl114-24.03.42-2026091411

### 新增功能
- **周期同步速度/扭矩模式（CSV/CST）**: 新增 `open_servo_csv_mode` / `close_servo_csv_mode` / `set_servo_csv_velocity` 与 `open_servo_cst_mode` / `close_servo_cst_mode` / `set_servo_cst_torque`，经 7000 端口周期下发关节速度（度/秒）或扭矩（额定扭矩的 0.1%）；速度/扭矩向量固定 7 元素（6 轴机器人第 7 位补 0）。
- **拖动轨迹文件管理**: 新增 `drag_trajectory_export` / `drag_trajectory_import` / `drag_trajectory_playback` / `drag_trajectory_delete`，可将控制器 `dragPos` 目录中的轨迹批量导出/导入本地目录，按文件名回放（速度 1~100）或删除。

### 接口变更
- **`DragParam`（破坏性变更，C++ ABI 不兼容）**: 字段随控制器协议更新——移除 `dragDamper` / `dragMass`，新增 `dragSensorCoeff`（拖动灵敏度），`dragChangeRate` 改为变化率比例，两者均为 6 元素向量，取值范围 0~100。结构体布局改变，**使用方须配合新头文件重新编译，不可与旧版库混用**；超长/超短向量返回参数错误。

## [2.5.3] - 2026-10-08

### 依赖版本
- **对应控制器版本**: rtl114-24.03.42-2026091411

### 改进
- **Release 版库开启编译器优化**: 此前发布产物未显式指定构建配置（等价于无优化），现按 Release 配置构建（GCC/Clang `-O3 -DNDEBUG`、MSVC `/O2`），运行性能提升。
- **Windows 分发包提供 Debug 版 C++ 库**: `lib/windows/debug/` 下提供调试版 `tl_host.dll` / `tl_host.lib` 与调试符号 `tl_host.pdb`，便于在 Visual Studio 调试器中单步跟踪 SDK 内部调用。调试版依赖 Visual Studio 的调试运行库（且须与自建程序的调试版配置配套使用），正式发布仍用 `lib/windows/` 下的 Release 版库；Python 绑定只提供 Release 版（`lib/windows/python/`）。Linux 平台只提供 Release 版库。
- **手眼标定示例保存原始画面**: Python 手眼标定示例（`ex_handeye_calibration.py`）保存图片时不再在图上绘制检测到的角点，改为直接保存原始画面（实时预览窗口仍叠加显示角点），便于对采集图像作目视核对。

## [2.5.2] - 2026-09-21

### 依赖版本
- **对应控制器版本**: rtl114-24.03.42-2026091411

### 新增功能
- **OmniPicker 夹爪示例**: 新增 `ex_omnipicker_claw`（C++ 与 Python），经控制器 Modbus 主站（RTU）控制 OmniPicker 夹爪：配置主站参数并打开，按寄存器表下发目标位置/速度/力矩/加减速度并写运动触发位，轮询状态寄存器等待到位，读回错误码/位置/速度/力矩。

### 改进
- **状态订阅示例仅保留 TCP**: `ex_robot_state_callback`（C++）移除 UDP 连接模式分支，改为单一 TCP（7000 端口）状态订阅演示，示例流程更直观；`connect_robot_udp` 接口本身不受影响，仍可用于 7000 端口连接与状态订阅。

## [2.5.1] - 2026-09-16

### 依赖版本
- **对应控制器版本**: rtl114-24.03.42-2026072316

### 新增功能
- **队列运动/作业条件指令补齐**: 新增 `queue_motion_push_back_until/while/if/jump`（条件排队运动）、`queue_motion_push_back_moveCA/imove/samov/cil/moveComm` 等 13 个队列运动扩展指令，以及 `job_insert_until/while/if/jump/label/end_until/end_while/end_if` 等作业指令插入接口；C++/Python 同步可用，Python 侧条件与逻辑组参数支持嵌套列表。
- **新增 Python 示例**: 干涉区检测 + MoveJ（`ex_interference.py`）—— 通过 `get_tool_trigger_interfer` 判断工具手位姿是否触发干涉区，触发则不执行该位姿的 MoveJ 运动，避免与干涉区物体碰撞。
- **拖拽/碰撞/传感器/网络/电机接口补齐**: 新增 `set/get_drag_param`（力矩拖拽）、`set_position_dragParams`（位置拖拽）、`set_collision_para`、`get_sensor_data`、`get_sensor_joint_thresholds`、`get_payload_param_by_sensor`、`set/get_controller_network_config`、`restore_network_factory_settings`、`set/get_axis_sdo`、`get_axis_sdo_string`、`get_remote_function`（与 `set_remote_function` 配套）、`get_current_motor_speed`/`get_current_motor_payload`/`get_current_encoded_pos`/`get_joint_voltage` 等电机状态查询，以及 `tool_hand_point_calibrate` 系列工具标定接口；同步新增 `DragParam` / `DragTorqueParam` / `PayloadParamBySensor` Python 类型。

### 改进
- **Python 绑定导出面补全（`tl_interface.py` / `tl_interface.pyi`）**: 补齐 155 个已绑定但未导出的符号——`__all__` 从 204 增至 359，星号导入可用的接口同步扩大；类型存根重新生成，补 142 个缺失函数与 8 个缺失类型（`SafeIO` / `AlarmdIO` / `DragParam` / `DragTorqueParam` / `PayloadParamBySensor` / `RemoteProgramSetting` / `VectorAlarmdIO` / `VectorRemoteProgramSetting`），覆盖手爪/灵巧手控制、作业流程控制与指令插入（`job_pause` / `job_step` / `job_stop` / `job_continue` / `job_insert_*`）、队列运动扩展指令（`queue_motion_push_back_*`）、工具标定（`tool_hand_*_calibrate*`）、Modbus 主站扩展、安全 IO、SDO/轴参数、全局字符串/变量、远程程序与系统备份恢复等接口；IDE 补全与静态检查（Pylance/pyright）恢复对全部接口的识别。

### Bug 修复
- **Python 绑定分发包导入失败**: `_tl_host` 扩展模块此前依赖同级的 `libtl_host.so`，分发包拷贝到其他机器后 `import tl_interface` 报 `libtl_host.so: cannot open shared object file`；现改为静态并入 SDK 实现（`tl_host_static`），模块自包含、运行时只依赖系统库，无 RUNPATH / `LD_LIBRARY_PATH` / `os.add_dll_directory()` 需求，构建树与 `cmake --install` 分发包任意位置移动后均可正常导入。
- **Python 示例 `ex_info_query.py`**: 修复查询远程 IO 功能设置时的解包错误（`ValueError: too many values to unpack (expected 3)`）——`get_remote_function` 绑定返回 `(ret, num, time, general, program)` 五元组，示例此前只解包 3 个值导致脚本中断；现按返回值取全部出参，正常打印远程 IO 数量、IO 重复触发屏蔽时间、通用功能端口与远程程序列表。

### 接口变更
- **实验性接口退出使用说明文档**: `servo_movej` / `servo_movel` 等 `tl_extension` 伺服插值封装（含 `open_servo_movel` / `close_servo_movej` / `wait_servo` / `cancel_servo` / `servo_busy`）与恒力驻留控制（`open_constforce` / `close_constforce` / `set_constforce_target_pose` / `get_constforce_status`）标注为实验性接口——接口仍随 SDK 提供（C++ / Python 均可调用，示例源码仍在 `examples/`），但《使用说明文档》不再提供对应章节，生产环境使用前请充分验证并自行承担风险。
- **`get_full_solution`（逆运动学全解）暂不可用**: 接口随 SDK 提供，但控制器固件暂不支持该命令（调用返回接收失败/超时）；《使用说明文档》与示例均不含该接口，待固件支持后再开放。

## [2.5.0] - 2026-08-20

### 依赖版本
- **对应控制器版本**: rtl114-24.03.42-2026072316

### 新增功能
- **Python 绑定切换 nanobind（linux-x86）**: `tl_interface` / `tl_extension` 由 SWIG/pybind11 迁移到 nanobind 自产绑定，行为对齐原 linux-x86 SWIG 现状（`VectorDouble` 等类型、out 参数语义、`Result` 常量均兼容）；`auto_test.py` 验收门通过（86/105，其余为无真机环境跳过/通信边界）。
- **恒力驻留控制接口（实验性接口，不纳入使用说明文档）**: 新增 `open_constforce` / `close_constforce`，后台线程持续运行恒力伺服闭环（力传感器回调 → 滤波 → 导纳补偿 → IK → servoj），支持传感器断连超时自动停止，控制频率与导纳 `control_period` 自动同步。
- **恒力控制扩展接口（实验性接口，不纳入使用说明文档）**: 新增 `set_constforce_target_pose`（运行时实时更新目标位姿，实现恒力下的扫动/移动）、`get_constforce_status`（实时查询当前原始/滤波后接触力与目标位姿）、`ConstantForceParams.force_control_axis`（显式指定力控轴，开启时校验导纳刚度 K=0）。
- **恒力驻留示例（实验性接口）**: 新增 `ex_constant_force.cpp`，演示恒力驻留完整流程（无扫动）。
- **新增 C++ 示例**: six_axis_force_drag（六维力传感器拖动示教）—— 设置传感器通讯参数并启用拖拽，进入力矩拖拽模式后实时读取六维力数据，可按指定时长或检测拖拽结束自动退出。
- **新增 Python 示例**: six_axis_force_drag（六维力传感器拖动示教，与 C++ 示例功能对应）。
- **tl_extension 高级伺服封装库（实验性接口，不纳入使用说明文档）**: servo_movej/servo_movel 等高级封装随 SDK 统一分发，支持 C++ 和 Python 调用。
- **Windows 平台支持**: 示例程序和测试程序现已支持 Windows 编译与运行。
- **碰撞检测参数接口**: 新增 `get_collision` / `set_collision`（24.03 版本接口，含 `_robot` 多机变体）获取与设置碰撞防护参数（各轴碰撞检测系数与伺服执行响应时间）。
- **新增 C++ 示例**: info_query（信息查询）、coord_tool（坐标系标定）、teach_control（拖动示教）、error_zero（六维力传感器标定）、digital_io（IO 控制）、modbus_tcp（Modbus TCP 主站）、modbus_rtu（Modbus RTU 主站）、tool_hand_number（工具手编号）、track_queue_drag（拖拽示教+轨迹记录回放）、queue_motion_control（队列运动控制）、robot_state_callback（状态回调）、servo（DH 参数/关节参数/笛卡尔参数/网络配置）、user_coord_ik（工具手+用户坐标下逆解关节角并用 MoveJ 下发运动）、tool_coord_ik（末端系逆解示例：接收末端坐标系下目标位姿（如手眼变换结果），复合当前直角坐标（基座标系）后逆解关节角并用 MoveJ 下发运动，内置末端系逆解接口 solve_endframe_ik）、collision_detect（碰撞检测与碰撞后清错自动恢复，碰撞阈值在示教器上设置）。
- **新增 C++ 示例**: info_query（信息查询）、coord_tool（坐标系标定）、teach_control（拖动示教）、error_zero（六维力传感器标定）、digital_io（IO 控制）、modbus_tcp（Modbus TCP 主站）、modbus_rtu（Modbus RTU 主站）、tool_hand_number（工具手编号）、track_queue_drag（拖拽示教+轨迹记录回放）、queue_motion_control（队列运动控制）、robot_state_callback（状态回调）、servo（DH 参数/关节参数/笛卡尔参数/网络配置）、user_coord_ik（工具手+用户坐标下逆解关节角并用 MoveJ 下发运动）、tool_coord_ik（末端系逆解示例：接收末端坐标系下目标位姿（如手眼变换结果），复合当前直角坐标（基座标系）后逆解关节角并用 MoveJ 下发运动，逆解目标坐标系可选基坐标或用户坐标）、collision_detect（碰撞检测与碰撞后清错自动恢复，碰撞阈值在示教器上设置）。
- **ex_tool_coord_ik 示例增强**: `solve_endframe_ik` 新增 `-c base|user` 逆解路径选项；user 模式走控制器原生 3→0 用户坐标逆解（与 1→0 结果可能不同），使用控制器当前激活的用户坐标系并运行时提醒，无需指定编号；示例不修改控制器用户坐标系状态。
- **新增 Python 示例**: 同步新增与 C++ 对应的 info_query、coord_tool、teach_control、error_zero、digital_io、modbus、servo 等示例。
- **新增 Python 示例**: 手眼标定验证（handeye_verify）—— 识别棋盘格后自动解算目标位姿并前往（目标点偏移量可配置），用于验证手眼标定结果。
- **`get_origin_coord_to_target_coord` 简化重载**: 新增不带 `convert_state` 参数的重载（int 与 `Coord` 枚举双版本），`form`/`reference_pos` 默认值对齐协议层（form=0、空参考点），调用方只需检查返回值；逆解失败（convert_state=false）时统一返回 `PARAM_ERR`。需区分"转换成功但逆解失败"时仍使用原带 `convert_state` 的重载（C++）。Python 侧同步简化：`arg6`/`arg7` 仅作 `form`/`reference_pos`，**统一返回单值 `Result`**（不再返回 `(ret, convert_state)` 元组），逆解失败映射 `PARAM_ERR` 与 C++ 一致。
- **手眼标定示例增强**: 画面实时显示棋盘格相对相机位姿与距离；保存的标定图片自动绘制角点，便于检查检测是否准确；保存时终端输出棋盘格相对相机位姿。
- **Python 示例支持 Linux ARM64**: 所有 21 个 Python 示例新增 `platform.machine()` 检测，ARM64 设备自动加载 `lib/linux/arm64/python/` 下的绑定文件。
- **补齐信息查询接口**: 新增 `get_robot_type`（机器人类型）、`get_robot_configuration`（机器人形态）、`get_robot_switch`（机器人切换）、`get_current_extra_position`（外部轴位置）、`get_current_positon_and_extra_position`（本体+外部轴位置）、`get_robot_joint_param`（关节参数）、`get_cartesian_params`（笛卡尔参数）、`get_four_point`（四点标定结果）、`get_single_cycle`（单圈值）、`get_user_coord_number`/`get_user_coord_para`（用户坐标）、`get_global_sync_position`/`get_global_variant`（全局点位/变量）、`get_target_coord_pos_value`/`get_local_pos_p_value`（点位值）、`get_drag_thread_is_end`/`get_axis_sum`/`get_independent_axis_param`/`get_axis_position`（拖拽/独立轴），以及 `RobotJointParam`/`CartesianParam`/`IndependentAxisParam` 三个参数结构体；C++/Python 同步可用。

### 改进
- **`Coord` 枚举重载扩展**: `get_target_coord_pos_value`（目标坐标系）、`get_origin_coord_to_target_coord`（原/目标双坐标系）新增 `Coord` 枚举重载，C++ 与 Python 均可直接传 `Coord::JOINT` / `Coord::CARTESIAN` / `Coord::TOOL` / `Coord::USER`，原 int 传参保持不变。
- **坐标系枚举 `Coord`**: 新增 `tl::Coord` 枚举（`Coord::JOINT` / `Coord::CARTESIAN` / `Coord::TOOL` / `Coord::USER`，对应 0/1/2/3），`get_current_position`、`get_current_coord`、`set_current_coord`、`get_current_positon_and_extra_position`、`servo_movej`、`servo_movel` 新增枚举重载，原 int 参数接口保持不变；`MoveCmd.coord` 字段仍为 int，可用 `static_cast<int>(Coord::xxx)` 赋值。Python 侧同步导出 `Coord` 枚举，6 个接口支持 `Coord` 枚举与 int 双签名直接传参（`get_current_coord` 返回 `Coord` 枚举），既有 int 传参代码不受影响；类型存根生成脚本新增重载函数 `@overload` 多签名支持。
- **Python 绑定补全**: 新增 `ModbusTCPParameter` / `ModbusRTUParameter` 类型导出，`ModbusMasterParameter` 现暴露 `TCP` / `RTU` 子参数属性，`SixDimensionalForceCommunicationParams` 补全全部字段（`YDirection` / `ZDirection` / `etherCat_mapNum` / `modbus_rtu_*` / `modbus_tcp_*`），与 C++ 接口完全对齐。
- **Python 绑定补全**: 新增 `RobotState` / `IOtype` / `AdmittanceParams` / `AdmittanceController` / `ConstantForceParams` / `ConstForceStatus` 类型导出；`ToolParam` 补全负载质心字段（`payloadMassCenter_X` / `payloadMassCenter_Y` / `payloadMassCenter_Z`）；`RemoteControl` 暴露 `program` 属性。`get_robot_state` / `get_io_type` / `get_constforce_status` / `open_constforce` 现可在 Python 中完整使用，其中 `open_constforce` 支持传入 Python 回调函数读取六维力数据（回调返回 `[Fx,Fy,Fz,Mx,My,Mz]` 六元素列表）。
- **Python 类型存根**: 自动生成中文注释（取自头文件 Doxygen `@brief`），IDE 悬浮提示可显示各函数中文说明。
- **队列运动示例延时对齐**: `ex_queue_motion.py` 开启队列模式后等待状态稳定（200ms，与 C++ 示例一致）、关闭队列模式后等待完全关闭（200ms），避免状态未稳定即执行下一步操作。
- **servo 示例移除 `tl_extension` 依赖**: `ex_servo_movej.py` / `ex_servo_movel.py` 改用 SDK 统一入口 `from tl_interface import *`，不再依赖 `tl_extension` 模块。
- **新增因时夹爪示例**: `ex_modbus.py` 更名为 `ex_inspire_claw.py`，改为因时电爪（Inspire EG2-4C2，RS485/Modbus RTU）控制测试示例：开度/速度/抓力寄存器控制（06H 写单个 + 03H 轮询实际开度），开度范围 0~1000（0=闭合完 1000=张开完），寄存器地址与机械臂 IP 集中于文件头部配置区（EG2-4C2 实测地址：目标开度=10，其余同段顺延）。
- **示例统一使用枚举重载接口**: 全部 C++/Python 示例中 `get_current_position` / `get_current_coord` / `set_current_coord` / `get_origin_coord_to_target_coord` 的坐标系参数改用 `Coord::JOINT` / `Coord::CARTESIAN` / `Coord::TOOL` / `Coord::USER` 枚举传参（替代 0/1/2/3 魔法数字），示例演示推荐的枚举用法。
- **示例程序支持 `-ip` 参数指定控制器 IP**: C++ 与 Python 全部 57 个示例均可通过 `-ip <地址>`（或 `--ip`）命令行参数指定控制器 IP，默认 `192.168.1.13`，不再需要修改源码硬编码 IP；手眼标定示例（`ex_handeye_calibration` / `ex_handeye_verify`）命令行参数优先于 `cal_config.yaml`。
- 优化上电时序，示例程序提供更健壮的实现。
- **`lib/` 目录改为构建产物，移出版本控制**: 动态库与 Python 绑定（`_tl_host` + `tl_interface.py`）构建后自动同步到仓库根 `lib/<platform>/`，Python 示例开箱即用；`BUILD_PYTHON` 默认开启，如需纯 C++ 构建可 `-DBUILD_PYTHON=OFF`。
- 删除废弃示例：`examples/py/ex_clamp_jaw.py`（ROS2 夹爪示例，依赖外部 ROS2 环境）与 `examples/py/ex_hand_gripper.py`（手爪示例），均非 SDK 核心功能。

### Bug 修复
- **Windows C++ 示例编译**: 修复 `ex_admittance_control` / `ex_tool_coord_ik` / `ex_user_coord_compare` / `ex_user_coord_ik` / `ex_teleop_f710_servoj` 在 Windows（MSVC）下编译失败（C2589）——`windows.h` 的 `min`/`max` 宏与 `std::min`/`std::max` 冲突，现已通过 `NOMINMAX` 宏隔离解决。
- **Python 绑定 `RobotDHParam`**: 补全 `L8`~`L20` 及耦合系数、动力学限制、丝杆导程、转换比等全部字段，新增 `HanYu` / `Alpha` 子结构导出（`hanyu` / `alpha` 属性，`hanyu.SP` / `hanyu.TL` 为三元素列表）；修复 `ex_info_query.py` 查询 DH 参数时 `AttributeError: 'RobotDHParam' object has no attribute 'L8'` 崩溃。
- **Linux ARM64 / Windows 源码构建**: 修正协议静态库路径（`third_party/nrc/lib/` 下 windows 库 `nrc_host.lib`、aarch64 库位于 `linux/aarch64/`），两平台现可从源码构建 `tl_host` 及示例。
- **Linux ARM64 Python 绑定**: 修正交叉编译产物扩展名平台标签（`_tl_host.cpython-310-aarch64-linux-gnu.so`），ARM64 设备上可直接 `import _tl_host`。
- **分发包内 C++ 示例无法独立编译**: 分发包（`cmake --install` 产物）无 `src/` 目录，`examples/cpp` 独立构建仍引用 `../../src/tl_host` 报 `add_subdirectory ... not an existing directory`；现自动检测：源码树内从源码构建（开发模式不变），分发包内改链 `lib/<platform>/` 预编译动态库，并将动态库拷贝到可执行文件目录（Linux `$ORIGIN` / Windows 同目录搜索），示例编译后任意位置可运行；示例安装改为整目录拷贝，遥操作（F710 手柄）示例源文件无条件进入分发包，目标机缺少依赖文件时防御性跳过、不再中断示例编译。
- **`get_robot_state`**: 修复调用方查询配置（channel/mode/position 等）未传入控制器的问题（此前协议层调用使用未初始化临时变量，查询配置全部丢失）。
- **Modbus 读取接口**: `modbus_read_coil_status` 等读取接口在控制器返回空数据（如 Modbus 从站未连接/无响应）时不再抛异常导致程序崩溃（C++ 异常 / Python `RuntimeError`），改为返回错误码 `EXCEPTION(-5)` 并清空输出数据，调用方可正常容错。
- **Modbus 主站参数传递**: 修复 `modbus_set_master_parameter` 传入 TCP/RTU 参数时的内存损坏崩溃（嵌套结构体含字符串字段被浅拷贝，析构时双重释放）；`ModbusMasterParameter` 的 `type`/`startAddress` 字段现可正确传递。
- **Python 绑定 `Sensor6DData`**: 补全去皮数据 6 字段（`fxDataSubBase` ~ `mzDataSubBase`）与 `torqueConvertData`，修复 `ex_info_query.py` / `ex_six_axis_force.py` / `ex_six_axis_force_drag.py` 查询六维力数据时 `AttributeError: 'Sensor6DData' object has no attribute 'fxDataSubBase'` 崩溃。
- 修复 Windows 平台 Python 逆解问题。
- 修复 `open_servoJ` 约束向量长度校验：`vmax`/`amax`/`jmax` 任一长度 ≠ 7 时返回 `PARAM_ERR`（不再直接下发导致控制器协议解析错乱、断开 7000 连接）；`set_servoJ_pos` 的 q 长度约定同步写入文档注释。
- 修复 ex_coord_transform 示例 servoJ 段 7 元素约定：约束参数与 q 发送统一为 7 元素（6 轴第 7 元素补 0、按实际轴数动态写入），Python 示例 q 发送逻辑不再硬编码 6 元素。
- 修复 `set_receive_error_or_warnning_message_callback` 接口：注册错误/警告回调后 Python 程序退出时崩溃（Fatal Python error）并误报 nanobind 泄漏的问题已修复——回调改为堆持有、解释器终结早期（Python atexit）释放，退出干净无泄漏报告。
- 修复 servoJ 示例对 7 轴的兼容问题。
- 修复 `get_robot_dh_param` 接口。
- 解决 Python `get_origin_coord_to_target_coord` 的 SWIG `bool&` 重载兼容性问题，三平台统一调用方式。
- 修复 Windows 下 Python 扩展模块输出路径问题，解决中文日志显示乱码。
- 修复 Windows 下 tl_extension Python 绑定重复加载 tl_host 库导致冲突的问题。
- 修复 ex_error_zero 六维力传感器标定示例的错误清理流程。
- 修复 ex_coord_transform 位姿矩阵往返验证逻辑：TR→R→TR 链路中平移不参与往返，删除对平移误差的错误断言；旋转误差对比改用旋转块索引，避免误将平移列当旋转元素比较。
- 修复 Python 绑定 `list` 不能直接传给 `VectorDouble` 参数的问题（nanobind 迁移回归，SWIG 时代可隐式转换），现已支持 list/tuple 直接传参；`VectorDouble`/`VectorInt`/`VectorString` 补齐 `push_back` 方法。
- 修复 `get_origin_coord_to_target_coord` Python 参数格式：恢复 SWIG 的 7 参数标准格式 `(sock, srcCoord, srcPos, dstCoord, dstPos, form, referencePos)`，可选第 6 参传 `True` 时返回 `(ret, convert_state)` 元组。
- 修复 Python 绑定缺失 `robot_go_to_reset_position` 接口与 `PosType_*` 枚举常量（nanobind 迁移回归，SWIG 时代为模块级全局常量）：`auto_test.py` 中「回到复位点」「MoveJ/MoveL 往返」用例不再抛 `NameError`。

### 接口变更
- **`get_library_version`**: 返回值改为同时包含 SDK 封装层版本和底层控制器库版本，格式 `"<sdk版本>(Base:<底层库版本>)"`（如 `"V2.5.1-0-gabc1234-20260916(Base:2.0.4-20260914-6cf4a3b)"`）。SDK 版本在 CMake configure 时由 `git describe --tags --always --long` + 提交日期自动生成（与 `package.sh` 打包命名同源），git 不可用时降级为 `PROJECT_VERSION` + 构建日期。调用方如需单独解析，可按 `(` 与 `(Base:` 拆分。
- **日志控制接口更名与迁移**: `TlLogLevel` 枚举更名 `LogLevel`，`set_tl_extension_log_level` 更名 `set_log_level`，枚举值 `TL_LOG_LEVEL_DEBUG` 等同步去前缀（`LogLevel::DEBUG` / `tl::WARNING` 形式）；两者从 `tl_servo_ext.h` 迁移至主接口 `tl_interface.h`。C++ 中需改用 `tl::set_log_level(tl::WARNING)`；Python 侧新增 `LogLevel` 枚举导出（`set_log_level(LogLevel.WARNING)` 可调用，旧名 `set_tl_extension_log_level` 已移除）。
- **`Coord::CARTESIAN` 更名 `Coord::BASE`**: 坐标系 1 统一命名为"基坐标系（直角坐标系）"，枚举名 `Coord::CARTESIAN` 改为 `Coord::BASE`（数值仍为 1）。C++ 中 `Coord::CARTESIAN` 需改为 `Coord::BASE`；Python 中 `Coord.CARTESIAN` 需改为 `Coord.BASE`（顶层常量 `CARTESIAN` 同步为 `BASE`）。int 传参接口与枚举数值完全不受影响；相关头文件注释、示例、文档、运行时日志中的"直角坐标系"描述已统一为"基坐标系（直角坐标系）"。
- **`queue_motion_stop` 废弃标记**: 接口已正式标记废弃——C++ 调用处产生编译警告（`deprecated`，提示改用 `queue_motion_stop_not_power_off`），pyi 存根 docstring 标注 `@deprecated`（IDE 悬浮提示可见）。接口仍可用，后续版本将移除。新增 `TL_DEPRECATED` 宏用于接口废弃标注（C++ 头文件 `TL_API TL_DEPRECATED("提示语") Result f(...)` 即可标记）。
- **`MoveCmd.coord` 类型**: 由 `int` 改为 `Coord` 枚举（`Coord::JOINT` / `Coord::CARTESIAN` / `Coord::TOOL` / `Coord::USER`，默认 `Coord::JOINT`）。C++ 中 `cmd.coord = 1` 需改为 `cmd.coord = Coord::CARTESIAN`（`static_cast<int>(Coord::xxx)` 可获取原数值）；Python 中 `cmd.coord = 1` 仍兼容（绑定层自动转换），读回为 `Coord` 枚举。示例与 Python 自动化测试已同步为枚举写法。
- **焊接工艺接口**: `weld_set_enable` / `weld_set_feed_wire` / `weld_set_supply_gas` 不再对外提供，已从 C++ 头文件（`tl_craft_weld.h`）与 Python 绑定中移除，C++ 与 Python 均不可再调用。
- **`get_axis_position`**: 不再对外提供，已从 C++ 头文件（`tl_interface.h`）与 Python 绑定中移除，C++ 与 Python 均不可再调用。
- **信息查询接口收敛**: `get_axis_sum` / `get_current_extra_position` / `get_current_positon_and_extra_position` / `get_independent_axis_param` / `get_robot_configuration` 不再对外提供，已从 C++ 头文件（`tl_interface.h`）、`IndependentAxisParam` 类型与 Python 绑定中移除，C++ 与 Python 均不可再调用。
- `get_origin_coord_to_target_coord` 调用统一为 7 参数标准格式 `(sock, srcCoord, srcPos, dstCoord, dstPos, form, referencePos)`，移除 `try/except TypeError` 兼容包装。
- **ex_collision_detect**: 改用 `get_collision`/`set_collision`（24.03 版本接口）获取与设置碰撞防护参数，设置演示将各轴碰撞检测系数设为 98（伺服执行响应时间不建议手动设置，沿用原值）并读回对比验证。
