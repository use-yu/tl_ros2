# 关节与末端跟踪误差监测

`tracking_monitor.py` 是只读 ROS2 节点，不发送运动指令，无需 colcon 编译。
默认适配 TCB705L，订阅 `/tcb_group_controller/controller_state`。
需要 Python 3、rclpy、control_msgs、NumPy 和 Matplotlib（用于保存图片及实时曲线）。
当前开发环境已具备这些依赖。

## 使用

先启动真实机械臂驱动和 `real_hardware_demo.launch.py`，确认控制器已激活。
也可连接 `demo.launch.py` 验证脚本，但虚拟反馈不能用来评估实体机械臂精度。
仅启动 bringup 不会产生轨迹控制器状态话题。

另开终端：

```bash
cd ~/tl_robot_ros2_cpp
source install/setup.bash
python3 scripts/tracking_monitor.py --plot
```

四幅实时曲线分别显示关节目标/反馈角度、关节误差、末端位置误差模长和姿态误差。
默认曲线展示最近 20 秒，最多保留 10000 点；CSV 记录所有接收到的有效样本。
目标曲线为虚线，反馈为实线。关闭窗口或按 Ctrl+C 停止并保存整段记录的曲线图片和统计表。

无图形界面时记录 60 秒：

```bash
python3 scripts/tracking_monitor.py --duration 60 --output /tmp/tcb705l_tracking.csv
```

默认输出 `/tmp/tracking_<时间戳>.csv` 及同名 `.csv.png` 图片，不再生成 JSON，路径会在启动时打印。
`/tmp` 为临时目录，需要长期保留时用 `--output` 指定其他目录。
CSV 或 PNG 已存在时拒绝覆盖对应文件，请换一个文件名。没有数据时仍输出空 CSV 和标注无有效数据的图片。
无论是否传入 `--plot`，正常停止时都会保存 PNG；图片包含整段记录，不受 `--window` 限制。
保存时会将完整 CSV 读入内存，长时间记录需要预留相应内存。
每两秒提示当前误差或反馈缺失状态。

常用参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--topic` | `/tcb_group_controller/controller_state` | 控制器状态话题，可替换为命名空间下的话题 |
| `--urdf` | 仓库的 `src/tl_description/urdf/tcb705l.urdf` | 实际使用的纯 URDF，不直接支持 xacro |
| `--base-link` | `link0` | FK 基坐标系 |
| `--tip-link` | `link7` | FK 末端坐标系（默认模型末端，未附加工具偏置） |
| `--window` | `20` | 图形时间窗口，秒 |
| `--duration` | `0` | 记录秒数，0 表示手动停止 |
| `--plot` | 关闭 | 启用实时曲线 |

其他臂型必须同时指定正确 URDF 和末端 link，例如六轴模型通常使用 `link6`。
支持固定、旋转关节链，不支持 mimic 关节。模型必须与规划时使用的模型一致。

## 数据定义与限制

- 优先读取同一控制器状态消息中的 `reference` / `feedback`；兼容旧版 `desired` / `actual`。
- 按关节名称匹配数据；拒绝不完整、非有限值、重复或倒退时间戳的样本。
  仿真时间重置后需重启脚本。时间戳倒退可能也意味着存在重复发布者。
- CSV 包含 ROS 消息时间戳、接收端单调时钟经过时间、各关节目标/反馈/误差（度）、
  FK 位置差 XYZ（毫米）、位置差模长（毫米）及旋转差角度（度）。
- 关节误差 = 目标 − 反馈，不进行 ±180° 折返，适用于本工作空间有限转角机械臂。
- 分别对目标关节角和反馈关节角做 FK；位置误差在指定基坐标系中计算，
  姿态误差为两旋转矩阵的相对旋转角（0–180°），不是欧拉角直接相减。
- PNG 底部统计表给出每项误差的 RMSE 和最大绝对值，按收到的样本等权统计，
  包含静止阶段；不会自动分离运动和到位阶段，也不会自动补偿反馈延迟。
- 这是**基于控制器关节反馈的模型末端误差**，不订阅 `/tcp_pose`，不等于外部测量的
  TCP 绝对精度。带工具时需提供包含正确工具变换的 URDF 并指定工具 link。
- 控制器状态包含最新缓存的硬件反馈，不保证编码器采样与目标严格同步；
  控制器仍发布状态时，脚本不能仅凭该消息识别底层反馈是否已停更。
  测量前确认驱动正常、没有反馈超时告警，并避免不同机械臂共用同一话题。

若持续提示没有状态，使用 `ros2 topic list -t` 确认实际话题名，
并检查 `ROS_DOMAIN_ID` 与控制器一致。脚本对反馈采用 best-effort QoS，
高负载时可能丢样；统计仅覆盖实际收到的有效样本。
