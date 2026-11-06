# TL SDK Python 绑定类型存根（自动生成，勿手改）
from typing import Any, ClassVar, Iterator, List, Tuple, Callable, Optional, overload, Union

# ---- 连接 / 伺服 / 状态 / 运动 / 坐标 / 工具 / 传感器 ----
def set_log_level(level: LogLevel) -> None:
    """
    @brief 设置日志级别
    @param level 日志级别（LogLevel 枚举）
    """
def get_library_version() -> str:
    """
    @brief 获取版本信息（包含 SDK 封装层版本 + 底层控制器库版本）
    @return 版本信息字符串，格式 "SDK v<sdk版本> (Base: <底层库版本>)"
    """
def set_connect_timeout_seconds(timeoutSeconds: int) -> Result:
    """
    @brief 设置连接超时时间，连接超过限制时间后直接返回错误
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def connect_robot(ip: str, port: str) -> int:
    """
    @brief 连接控制器
    @param ip 控制器ip,"192.168.1.13"
    @param port 端口号,"6001"
    @use 此函数是同步方式连接，因此函数会阻塞，直到返回连接结果。
    @return -1-失败
    """
def connect_robot_udp(ip: str, port: str) -> int:
    """
    @brief 连接控制器 UDP（7000 伺服端口）
    @param ip 控制器ip,"192.168.1.13"
    @param port 端口号,"7000"
    @return 成功返回 socket 句柄；失败返回 -1
    """
def disconnect_robot(socketFd: int) -> Result:
    """
    @brief 断开控制器
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_connection_status(socketFd: int) -> Result:
    """
    @brief 获得控制器连接状态
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_reconnect(socketFd: int, reconnect: bool) -> Result:
    """
    @brief 设置是否打开断开后自动重连功能 默认关闭
    @param reconnect true 打开
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_receive_error_or_warnning_message_callback(socketFd: int, function: Callable[..., object]) -> Result:
    """
    @brief 设置接收错误或警告信息的回调函数。
    @param function 指向回调函数的指针。该回调函数在接收到错误或警告信息时被调用。
    @note 回调函数由用户实现，用于处理接收到的错误信息。
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def recv_message(socketFd: int, function: Callable[..., object]) -> Result:
    """
    @brief 收到控制器消息时触发设置的回调函数
    @param function 指向回调函数的指针，签名 `void (int messageID, const char* message)`。
           回调收到控制器主动推送的消息 id 与消息内容。
    @warning 回调函数内不能做耗时操作或阻塞（控制器消息线程内触发）。
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_controller_ip(socketFd: int, name: str, address: str, gateway: str, dns: str) -> Result:
    """
    @brief 配置控制器有线网口 IP
    @param name 配置名
    @param address ip地址
    @param gateway 网关
    @param dns DNS
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_controller_id(socketFd: int) -> Tuple:
    """
    @brief 获取控制器ID
    @param id 输出：控制器ID字符串
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_nexmotion_lib_version(socketFd: int) -> Tuple:
    """
    @brief 获取算法库版本
    @param version 输出：算法库版本字符串
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def clear_error(socketFd: int) -> Result:
    """
    @brief 伺服清错
    @note
    出错前如果时伺服运行状态，清错后需要手动进行下电操作，释放控制器的占用状态才可以继续上电（清错后不能直接上电，先下电再上电）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_servo_state(socketFd: int, state: int) -> Result:
    """
    @brief 设置伺服状态
    @param state 0 停止 1 就绪
    @warning 设置伺服就绪应该先确保系统没有错误 clear_servo_error(SOCKETFD socketFd)
    该函数只有伺服状态为0（停止状态）或1（就绪状态）时调用生效，伺服状态为2（报警状态）或3（运行状态）时不能直接设置伺服状态
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_servo_state(socketFd: int, status: int) -> Tuple:
    """
    @brief 获取伺服状态
    @param status 接收获取结果 0：停止状态 1：就绪状态 2：报警状态 3：运行状态
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_servo_poweron(socketFd: int) -> Result:
    """
    @brief 机器人上电
    @attention 调用该函数之前需要先调用set_servo_state(SOCKETFD socketFd,1)将伺服设置为1（就绪状态）
    			,机器人上电成功后调用 get_servo_state(SOCKETFD socketFd)为3伺服运行状态
    @return 机器人当前伺服状态servoStatus
    该函数只有伺服状态为1（就绪状态）时调用生效
    """
def set_servo_poweroff(socketFd: int) -> Result:
    """
    @brief 机器人下电
    @attention 机器人下电成功后调用 get_servo_state(SOCKETFD socketFd)为1伺服就绪状态
    @return 机器人当前伺服状态servoStatus
    该函数只有伺服状态为3（运行状态）时调用生效
    """
def get_robot_state(socketFd: int, param: RobotState) -> Result:
    """
    @brief 7000端口查询状态
    需要连接7000端口
      SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
    
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
     参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_robot_running_state(socketFd: int, status: int) -> Tuple:
    """
    @brief 获取程序运行状态
    @param status 程序运行状态
     - 0 停止
     - 1 暂停
     - 2 运行
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_robot_dh_param(socketFd: int, param: RobotDHParam) -> Result:
    """
    @brief 设置当前机器人DH参数
    @param param 结构体参数（标准 DH 参数：alpha[6]/a[6]/theta[6]/d[6] + eulerAngle/mountingAngle）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def restore_default_param_DH(socketFd: int, robotNum: int) -> Result:
    """
    @brief 恢复机械臂默认DH参数
    @param robotNum 机器人编号，0 为默认（单机器人模式）；多机器人模式下为机器人序号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_robot_dh_param(socketFd: int, param: RobotDHParam) -> Result:
    """
    @brief 获取当前机器人DH参数
    @param param 结构体参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_collision_detection_param(socketFd: int, param: CollisionPara) -> Result:
    """
    @brief 查询碰撞防护等级
    @param param 接收获取结果
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_collision(socketFd: int, param: CollisionSafeParam) -> Result:
    """
    @brief 获取碰撞防护参数（24.03+ 固件）
    @param param 输出：碰撞防护参数（详细参数在 CollisionSafeParam 结构体中说明）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_collision(socketFd: int, param: CollisionSafeParam) -> Result:
    """
    @brief 设置碰撞防护参数（24.03+ 固件）
    @param param 碰撞防护参数（详细参数在 CollisionSafeParam 结构体中说明）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_teach_sensitivity(socketFd: int, sensitivity: VectorDouble) -> Result:
    """
    @brief 获取电流环拖动示教灵敏度
    @param sensitivity 输出参数，用于接收各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为
    J1-J6，7轴为 J1-J7）
    @return Result 操作结果，SUCCESS表示成功，其他值表示失败
    """
def query_joint_software_version(socketFd: int, axisNum: int) -> Tuple:
    """
    @brief 查询指定关节（轴）的软件版本号 (SDO 0x100A)
    @param[out] version 输出参数，用于接收查询到的软件版本号字符串
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def get_robot_type(socketFd: int, type_: int) -> Tuple: ...
@overload
def get_robot_type(socketFd: int, type_: RobotType) -> Tuple:
    """
    @brief 获取机器人类型（枚举重载）
    @param type 输出：机器人类型 RobotType 枚举
     - RobotType::SIX_AXIS_SERIAL 六轴串联多关节
     - RobotType::FOUR_AXIS_SCARA 四轴 SCARA
     - RobotType::FOUR_AXIS_PALLETIZING 四轴码垛
     - RobotType::FOUR_AXIS_SERIAL 四轴串联多关节
     - RobotType::SINGLE_AXIS 单轴
     - RobotType::FIVE_AXIS_SERIAL 五轴串联多关节
     - RobotType::SIX_AXIS_COLLABORATIVE 六轴协作
     - RobotType::TWO_AXIS_SCARA 二轴 SCARA
     - RobotType::THREE_AXIS_SCARA 三轴 SCARA
     - RobotType::THREE_AXIS_CARTESIAN 三轴直角
     - RobotType::THREE_AXIS_SPECIAL 三轴异形
     - RobotType::SEVEN_AXIS_SERIAL 七轴串联多关节
     - RobotType::SCARA_SPECIAL SCARA 异形一
     - RobotType::FOUR_AXIS_PALLETIZING_LEAD 四轴码垛丝杆
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_robot_switch(socketFd: int, robot: int) -> Tuple:
    """
    @brief 获取当前机器人编号（多机器人模式下用于区分机器人）
    @param robot 输出：当前机器人编号（多机器人模式下的机器人序号）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_robot_joint_param(socketFd: int, id: int, param: RobotJointParam) -> Result:
    """
    @brief 设置当前机器人指定关节参数
    @param id 关节编号，取值范围 [1,6]（6轴，对应 J1~J6）；7轴为 [1,7]（对应 J1~J7）
    @param param 关节参数（结构体）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_robot_joint_param(socketFd: int, id: int, param: RobotJointParam) -> Result:
    """
    @brief 获取指定关节参数
    @param id 关节编号，取值范围 [1,6]（6轴，对应 J1~J6）；7轴为 [1,7]（对应 J1~J7）
    @param param 输出：关节参数（结构体）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_cartesian_params(socketFd: int, param: CartesianParam) -> Result:
    """
    @brief 获取笛卡尔空间参数
    @param param 输出：用于接收笛卡尔空间运动参数的结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_state_callback(socketFd: int, function: Any) -> Result:
    """
    @brief 7000端口状态返回的回调函数
    需要连接7000端口
      SOCKETFD fd7000 = connect_robot("192.168.1.13","7000");
    
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
     参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_joint_temperature(socketFd: int, temperatures: VectorDouble) -> Result:
    """
    @brief 获取关节当前温度
    @param temperatures 输出：关节温度向量，按实际轴数排列（6轴为 J1-J6，7轴为 J1-J7）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_motor_current(socketFd: int, motor_current: VectorDouble, motor_current_sync: VectorDouble) -> Result:
    """
    @brief 获取当前电机电流（24.03接口）
    @param motor_current 输出：机器人电机电流，长度7，单位[‰]
    @param motor_current_sync 输出：外部轴电机电流，长度5，单位[‰]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_motor_torque(socketFd: int, motorTorque: VectorInt, motorTorqueSync: VectorInt) -> Result:
    """
    @brief 获取当前电机扭矩
    @param motorTorque 输出：机器人扭矩，长度7，单位[%]
    @param motorTorqueSync 输出：外部轴扭矩，长度5，单位[%]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_movej(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 关节运动
    @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
    @param vel 速度，参数范围：0<vel≤100 单位 %
    @param coord 坐标系，参数范围：0≤coord≤3
    @param acc 加速度，参数范围：0<acc≤100
    @param dec 减速度，参数范围：0<dec≤100
    @param isSync 是否同步模式 true同步 false不同步
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_movel(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 直线运动
    @param targetPosValue 点位数组，n个轴就赋值前n位数组,其余置0
    @param vel 速度，参数范围：0<vel≤1000 单位mm/s
    @param coord 坐标系，参数范围：0≤coord≤3
    @param acc 加速度，参数范围：0<acc≤100
    @param dec 减速度，参数范围：0<dec≤100
    @param isSync 是否同步模式 true同步 false不同步
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_speed(socketFd: int, speed: int) -> Result:
    """
    @brief 设置当前模式的速度 有三种模式 示教模式，运行模式，远程模式
    @param speed 速度，参数范围：0<speed≤100
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_speed(socketFd: int, speed: int) -> Tuple:
    """
    @brief 获得当前模式的速度 有三种模式 示教模式，运行模式，远程模式
    @param speed 速度，参数范围：0<speed≤100
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_current_mode(socketFd: int, mode: int) -> Result:
    """
    @brief 设置机器人当前模式
    @param mode 模式 0：示教 1：远程 2：运行
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_mode(socketFd: int, mode: int) -> Tuple:
    """
    @brief 获取机器人当前模式
    @param mode 当前模式 0：示教 1：远程 2：运行
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def get_current_position(socketFd: int, coord: int, pos: VectorDouble) -> Result: ...
@overload
def get_current_position(socketFd: int, coord: Coord, pos: VectorDouble) -> Result:
    """
    @brief 获取机器人当前位置（枚举坐标系重载）
    @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @param pos 出参 存储返回结果点位的容器，长度7
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_joint_position(socketFd: int, axisNum: int, pos: VectorDouble) -> Result:
    """
    @brief 获取机器人当前指定关节点在基坐标系（直角坐标系）中的位置(2403版本专用)
    @param axisNum 指定需要查询的关节
    @param pos 存储返回结果点位的容器，长度7
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_pos_reachable(socketFd: int, pos: VectorDouble, movetype: str) -> Tuple:
    """
    @brief 获取点位是否可达
    @param pos 目标点位坐标数据，容器长度为14：[0]坐标系 0：关节 1：直角 2：工具 3：用户 [1]:0 角度制
    1弧度制 [2]形态 [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
    @param movetype 移动方式 "MOVJ" 或 "MOVL"
    @param result 输出：点位是否可达
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_line_speed_and_joint_speed(socketFd: int) -> Tuple:
    """
    @brief 获取当前末端线速度和关节速度
    @param lineSpeed 输出：末端线速度，单位[mm/s]
    @param jointSpeed 输出：关节速度，长度7，单位[度/s]
    @param jointSpeedSync 输出：外部轴关节速度，长度7，单位[度/s]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def get_current_coord(socketFd: int, coord: int) -> Tuple: ...
@overload
def get_current_coord(socketFd: int, coord: Coord) -> Tuple:
    """
    @brief 获取机器人当前坐标系（枚举重载）
    @param coord 出参 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def set_current_coord(socketFd: int, coord: int) -> Result: ...
@overload
def set_current_coord(socketFd: int, coord: Coord) -> Result:
    """
    @brief 设置机器人当前坐标系（枚举重载）
    @param coord 坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_global_position(socketFd: int, posName: str, pos: VectorDouble) -> Result:
    """
    @brief 查询全局GP点位
    @param posName 全局位置名 例如 "GP0001"
    @param pos 全局点位数组 长度14 前7位为点位的坐标、姿态等信息，后7位为机器人位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_global_position(socketFd: int, posName: str, pos: VectorDouble) -> Result:
    """
    @brief 设置全局GP点位
    @param posName 需要修改全局位置名 例如 "GP0001"
    @param pos[14] [0]坐标系 0：关节 1：基坐标 2：工具 3：用户  [1]:0 角度制 1弧度制 [2]形态
    [3]工具手坐标序号 [4]用户坐标序号 [5][6] 备用 [7-13] 点位信息
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_start_jogging(socketFd: int, axis: int, dir: bool) -> Result:
    """
    @brief 开始点动
    @param axis 轴号
    @param dir 方向
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_stop_jogging(socketFd: int, axis: int) -> Result:
    """
    @brief 停止点动
    @param axis 轴号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_current_teach_sensitivity(socketFd: int, sensitivity: VectorDouble) -> Result:
    """
    @brief 设置电流环拖动示教灵敏度
    @param sensitivity 包含各关节电流环拖动示教灵敏度参数的向量，按实际轴数排列（6轴为 J1-J6，7轴为
    J1-J7），范围应在0-3之间
    @return Result 操作结果，SUCCESS表示成功，其他值表示失败
    """
def set_darg_mode(socketFd: int, mode: int) -> Result:
    """
    @brief 设置拖拽示教的拖拽方式
    @param mode 拖拽模式  0-无  1-3D鼠标  2-力矩模式 3-位置 (22.07版本没有位置模式)
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_teach_type(socketFd: int, type_: int) -> Result:
    """
    @brief 设置示教模式类型
    @param type 0 点动 1 拖拽
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_teach_type(socketFd: int, type_: int) -> Tuple:
    """
    @brief 获取示教模式类型
    @param type 输出：0 点动 1 拖拽
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_go_home(socketFd: int) -> Result:
    """
    @brief 回到设定的零点
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def robot_go_to_reset_position(socketFd: int) -> Result:
    """
    @brief 回到控制器预设的复位点
    @note 若控制器未配置复位点，接口可能返回错误码
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_quat2rpy(socketFd: int, quat_vector: VectorDouble, rpy_res: VectorDouble) -> Result:
    """
    @brief 四元数转欧拉角
    @param quat_vector 被转换的四元数向量，vector长度 = 3
    @param rpy_res 接收欧拉角向量结果，vector长度 = 4
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_rpy2quat(socketFd: int, rpy_vector: VectorDouble, quat_res: VectorDouble) -> Result:
    """
    @brief 欧拉角转四元数
    @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
    @param quat_res 接收四元数向量结果，vector长度 = 3
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_r2tr(socketFd: int, r_matrix: VectorDouble, tr_res: VectorDouble) -> Result:
    """
    @brief 旋转矩阵转位姿
    @param r_matrix 被转换的旋转矩阵，vector长度 = 9（行主序）
    @param tr_res 接收位姿矩阵结果，vector长度 = 16（行主序）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_rpy2r(socketFd: int, rpy_vector: VectorDouble, r_res: VectorDouble) -> Result:
    """
    @brief 欧拉角转旋转矩阵
    @param rpy_vector 被转换的欧拉角向量，vector长度 = 4
    @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_tr2r(socketFd: int, tr_matrix: VectorDouble, r_res: VectorDouble) -> Result:
    """
    @brief 位姿转旋转矩阵
    @param tr_matrix 被转换的位姿矩阵，vector长度 = 16（行主序）
    @param r_res 接收旋转矩阵结果，vector长度 = 9（行主序）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_tool_hand_number(socketFd: int, toolNum: int) -> Tuple:
    """
    @brief 获取当前使用的工具手编号
    @param toolNum 工具手编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_tool_hand_number(socketFd: int, toolNum: int) -> Result:
    """
    @brief 设置工具手编号
    @param toolNum 工具手编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_tool_hand_param(socketFd: int, toolNum: int, param: ToolParam) -> Result:
    """
    @brief 设置工具手参数
    @param toolNum 工具手编号
    @param param 要修改的参数 长度6 X轴偏移量 Y轴偏移量 Z轴偏移量 绕A轴旋转量 绕B轴旋转量 绕C轴旋转量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_tool_hand_param(socketFd: int, toolNum: int, param: ToolParam) -> Result:
    """
    @brief 获取当前工具手参数
    @param toolNum 工具手编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_tool_coordinate_range(socketFd: int, tool_number: int, range_param: ToolCoordinateRange) -> Result:
    """
    @brief 获取工具坐标范围参数
    @param tool_number 工具编号
    @param[out] range_param 工具坐标范围参数结构体
    @return Result 操作结果
    """
def set_tool_coordinate_range(socketFd: int, tool_number: int, range_param: ToolCoordinateRange) -> Result:
    """
    @brief 设置工具坐标范围参数
    @param tool_number 工具编号
    @param range_param 工具坐标范围参数结构体
    @return Result 操作结果
    """
def set_user_coord_number(socketFd: int, userNum: int) -> Result:
    """
    @brief 设置用户坐标编号
    @param userNum 用户坐标编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_user_coordinate_data(socketFd: int, userNum: int, pos: VectorDouble) -> Result:
    """
    @brief 标定用户坐标
    @param userNum 用户坐标编号
    @param pos 坐标数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_default_cartesian_params(socketFd: int) -> Result:
    """
    @brief 设置笛卡尔参数为默认值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def calculate_user_coordinate(socketFd: int, userNumber: int) -> Result:
    """
    @brief 计算坐标
    @param userNumber 用户坐标编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def calibration_oxy(socketFd: int, userNum: int, xyo: str) -> Result:
    """
    @brief 标定OXY
    @param userNum 用户坐标编号
    @param xyo 值 'X' 'Y' 'O'
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def get_origin_coord_to_target_coord(socketFd: int, originCoord: int, originPos: VectorDouble, targetCoord: int, targetPos: VectorDouble, arg6: Optional[object] = None, arg7: Optional[object] = None) -> Any: ...
@overload
def get_origin_coord_to_target_coord(socketFd: int, originCoord: Coord, originPos: VectorDouble, targetCoord: Coord, targetPos: VectorDouble, arg6: Optional[object] = None, arg7: Optional[object] = None) -> Any:
    """
    @brief 原坐标值转换为其他坐标值（枚举坐标系，无 convert_state 简化版）
    @param originCoord 原坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @param originPos 要进行转换的坐标值 [0,1,2,3,4,5,6]
    @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @param targetPos 转换后的坐标值（点位数组）
    @param form 形态（默认 0，与 nrc 对齐）
    @param reference_pos 参考点（默认空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误或逆解失败；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @note 不带 convert_state 的简化版：逆解失败（convert_state=false）时统一返回 PARAM_ERR，
          调用方只需检查返回值；需要区分"转换执行成功但逆解失败"时请使用带 convert_state 的重载。
    @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
    """
def get_four_point(socketFd: int, result: VectorDouble) -> Result:
    """
    @brief 查询四点标定结果
    @param result 输出：四点标定结果（查询到的标记点数据）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_single_cycle(socketFd: int, single_cycle: VectorDouble) -> Result:
    """
    @brief 获取单圈值
    @param single_cycle 输出：单圈值数组，长度7
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_user_coord_number(socketFd: int, userNum: int) -> Tuple:
    """
    @brief 获取当前使用的用户坐标编号
    @param userNum 输出：当前使用的用户坐标编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_user_coord_para(socketFd: int, userNum: int, pos: VectorDouble) -> Result:
    """
    @brief 获取用户坐标参数
    @param userNum 用户坐标编号（入参，指定要查询的用户坐标）
    @param pos 输出：用户坐标参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_global_sync_position(socketFd: int, posName: str, pos: VectorDouble) -> Result:
    """
    @brief 查询全局 GE 点位
    @param posName 全局位置名，例如 "GE0001"
    @param pos
    输出：全局点位数组，长度21；前7位为点位的坐标、姿态等信息，中间7位为机器人位置，后7位为外部轴位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_global_variant(socketFd: int, varName: str, value: float) -> Tuple:
    """
    @brief 查询全局变量
    @param varName 全局变量名，支持形式如 "GI001" / "GD001" / "GB001"
    @param value 输出：变量值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
@overload
def get_target_coord_pos_value(socketFd: int, name: str, targetCoord: int, targetPos: VectorDouble) -> Result: ...
@overload
def get_target_coord_pos_value(socketFd: int, name: str, targetCoord: Coord, targetPos: VectorDouble) -> Result:
    """
    @brief 获取指定坐标下的位置（枚举坐标系重载）
    @param name 点位名（P点、GP点、E点、GE点），如 "GP0001"
    @param targetCoord 目标坐标系 Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER
    @param targetPos
    输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @note 推荐使用枚举重载（Coord::JOINT / Coord::BASE / Coord::TOOL / Coord::USER），避免魔法数字
    """
def get_local_pos_p_value(socketFd: int, name: str, pos: VectorDouble) -> Result:
    """
    @brief 获取局部 P 点
    @param name 点位名，范围 P0001 - P9999
    @param pos
    输出：转换后的位置，14位点位（坐标系,角度/弧度,形态/左右手,工具号,用户坐标号,预留,预留,1轴,2轴,3轴,4轴,5轴,6轴,7轴）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_drag_thread_is_end(socketFd: int, endFlag: bool) -> Tuple:
    """
    @brief 获取拖拽结束标志
    @param endFlag 输出：true 拖拽结束
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_sensor_6d_data(socketFd: int, sensorData: Sensor6DData) -> Result:
    """
    @brief 获取六维力传感器数据
    @param sensorData 用于接收六维力传感器数据的结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_sensor_base_param(socketFd: int, baseParam: SensorBaseParam) -> Result:
    """
    @brief 获取六维力传感器的基础参数（质量、质心、标零状态）
    @param baseParam 用于接收传感器基础参数的结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def mark_base_sensor(socketFd: int, success: bool) -> Tuple:
    """
    @brief 执行六维力传感器标定
    @param[out] success 标定是否成功的标志
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_six_dimensional_force_communication_params(socketFd: int, params: SixDimensionalForceCommunicationParams) -> Result:
    """
    @brief 设置六维力传感器通讯
    @param params 用于设置六维力传感器通讯参数的结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_six_dimensional_force_communication_params(socketFd: int, params: SixDimensionalForceCommunicationParams) -> Result:
    """
    @brief 获取六维力传感器通讯参数
    @param params 六维力通讯参数，详见 SixDimensionalForceCommunicationParams
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_reconnect_callback(socketFd: int, function: Any) -> Result:
    """
    @brief 设置重连成功后的回调函数
    @param function 指向回调函数的指针，重连成功后会被调用
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def send_message(socketFd: int, messageID: int, message: str) -> Result:
    """
    @brief 向控制器发送一条自定义消息
    @param messageID 消息ID
    @param message 消息内容
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_robots_parallel(socketFd: int, open: bool) -> Result:
    """
    @brief 设置多机器人并行模式
    @param open true 打开并行；false 关闭并行
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_axis_sdo(socketFd: int, axisNum: int, index: int, subindex: int, cmdvalue: int, size: int) -> Result:
    """
    @brief 设置伺服命令字（SDO）
    @param axisNum 机器人轴编号
    @param index 命令字编码（对象字典索引）
    @param subindex 命令字子编码
    @param cmdvalue 要设置进去的值
    @param size 命令字对应值的字节数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_axis_sdo(socketFd: int, axisNum: int, index: int, subindex: int, size: int, value: int) -> Tuple:
    """
    @brief 读取轴的 SDO 值
    @param axisNum 轴号
    @param index 对象字典索引
    @param subindex 子索引
    @param size 数据大小（8/16/32 位）
    @param value 输出：读取到的数值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_axis_sdo_string(socketFd: int, axisNum: int, index: int, subindex: int, size: int, value: str) -> Tuple:
    """
    @brief 读取轴的 SDO 字符串值
    @param axisNum 轴号
    @param index 对象字典索引
    @param subindex 子索引
    @param size 数据大小
    @param value 输出：读取到的字符串值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_teachbox_connection_status(socketFd: int, connected: bool) -> Tuple:
    """
    @brief 查询示教盒的连接状态
    @param connected 接收连接状态
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_controller_id_csharp(socketFd: int, id: List[str]) -> Tuple:
    """
    @brief 获取控制器序列号 ID（C# 兼容接口）
    @param id 返回控制器序列号 ID 的字符容器
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_axis_zero_position(socketFd: int, axis: int) -> Result:
    """
    @brief 设置零点位置
    @param axis 轴号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_sync_axis_zero_position(socketFd: int, axis: int) -> Result:
    """
    @brief 设置外部轴零点位置
    @param axis 轴号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_zero_pos_deviation(socketFd: int, axis: int, shift: float) -> Result:
    """
    @brief 设置零点偏移
    @param axis 需要偏移的轴数
    @param shift 偏移量，范围 -360° < shift < 360°
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_four_point_mark(socketFd: int, point: int, status: int) -> Result:
    """
    @brief 进行 4 点标记
    @param point 标记点位编号，取值范围 0-3
    @param status 标记状态：0 取消标记；1 标记
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def four_point_calculation(socketFd: int, L1: float, L2: float, result: VectorDouble) -> Tuple:
    """
    @brief 4 点标定计算
    @param L1 参数 L1
    @param L2 参数 L2
    @param result 输出：标定计算结果
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_result_for_DH(socketFd: int, apply: int) -> Result:
    """
    @brief 将 4 点标定计算的结果写入机器人 DH 参数
    @param apply 写入是否成功：成功/失败（1/0）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_robot_switch(socketFd: int, robot: int) -> Result:
    """
    @brief 切换当前机器人
    @param robot 切换到目标机器人编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_cartesian_params(socketFd: int, param: CartesianParam) -> Result:
    """
    @brief 设置机器人笛卡尔参数
    @param param 包含笛卡尔空间运动参数的结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_point_calibrate(socketFd: int, point: int, toolNum: int, calibrationType: int = 7) -> Result:
    """
    @brief 记录工具手标定点（4/6/7 点标定，24.03 版本）
    @param point 标定点下标，范围 0-6
    @param toolNum 工具手编号，范围 1-999
    @param calibrationType 标定类型：4、6 或 7（默认 7）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_point_calibrate_caculate(socketFd: int, toolNum: int, calibrationType: int = 7) -> Result:
    """
    @brief 计算工具手标定结果（4/6/7 点标定，24.03 版本）
    @param toolNum 工具手编号，范围 1-999
    @param calibrationType 标定类型：4、6 或 7（默认 7）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_point_calibrate_clear(socketFd: int, point: int, toolNum: int, calibrationType: int = 7) -> Result:
    """
    @brief 清除工具手标定点（4/6/7 点标定，24.03 版本）
    @param point 标定点下标，范围 0-6
    @param toolNum 工具手编号，范围 1-999
    @param calibrationType 标定类型：4、6 或 7（默认 7）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_2_or_20_point_calibrate(socketFd: int, point: int) -> Result:
    """
    @brief 2 点标定/20 点标定：记录当前选择的工具手标定点
    @param point 标定点 1、2
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_2_or_20_point_calibrate_caculate(socketFd: int, calNum: int = 1, noCalZero: bool = False) -> Result:
    """
    @brief 2 点标定/20 点标定：计算
    @param calNum 标定计算点数（默认 1）
    @param noCalZero true 校准工具尺寸+姿态，不校准零点；false 其它情况，校准零点
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_2_or_20_point_calibrate_clear(socketFd: int, pointNum: int) -> Result:
    """
    @brief 2 点标定/20 点标定：清除标定点
    @param pointNum 清除的标定点编号 1-20
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_7_point_calibrate(socketFd: int, point: int, toolNum: int) -> Result:
    """
    @brief 7 点标定：记录当前选择的工具手标定点
    @param point 标定点 1-7
    @param toolNum 工具手编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_7_point_calibrate_caculate(socketFd: int, toolNum: int, calibrationPointNum: int = 7) -> Result:
    """
    @brief 7 点标定：计算
    @param toolNum 工具手编号
    @param calibrationPointNum 标定点数量（默认 7）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def tool_hand_7_point_calibrate_clear(socketFd: int, pointNum: int, toolNum: int) -> Result:
    """
    @brief 7 点标定：清除标定点
    @param pointNum 清除的标定点 1-7
    @param toolNum 工具手编号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_global_sync_position(socketFd: int, posName: str, pos: VectorDouble) -> Result:
    """
    @brief 设置全局 GE 点位
    @param posName 全局位置名，例如 "GE0001"
    @param posInfo 全局点位数组，长度 21：前 7 位为点位坐标/姿态信息，中间 7 位为机器人位置，后 7
    位为外部轴位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_global_variant(socketFd: int, varName: str, varValue: float) -> Result:
    """
    @brief 设置全局变量
    @param varName 全局变量名，例如 "GI001" / "GD001" / "GB001"
    @param varValue 变量值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_global_string(socketFd: int, varName: str, varValue: str) -> Result:
    """
    @brief 设置全局字符串变量
    @param varName 全局变量名，例如 "GS001"
    @param varValue 变量值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_global_string(socketFd: int, varName: str, value: str) -> Tuple:
    """
    @brief 查询全局字符串变量
    @param varName 全局变量名，例如 "GS001"
    @param value 输出：变量值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_extra_position(socketFd: int, pos: VectorDouble) -> Tuple:
    """
    @brief 获取机器人外部轴当前位置
    @param pos 输出：点位数组，长度 5
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_positon_and_extra_position(socketFd: int, coord: int, pos: VectorDouble) -> Tuple:
    """
    @brief 获取机器人和外部轴的当前位置
    @param coord 指定需要查询的坐标系
    @param pos 输出：点位数组，长度 12
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_robot_configuration(socketFd: int, configuration: int) -> Tuple:
    """
    @brief 获取 4 轴机器人的形态（SCARA）
    @param configuration 输出：1-左手；2-右手
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_full_solution(socketFd: int, transRPY: VectorDouble, full_solution: List[VectorDouble]) -> Result:
    """
    @brief 获取机器人逆运动学全解
    @param transRPY 目标位姿，vector 长度 = 6（前 3 位位置，后 3 位欧拉角）
    @param full_solution 输出：逆运动学全解结果，每组 6 个关节角
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_static_search_position(socketFd: int, fileid: int, tableid: int, delaytime: int, pos: VectorDouble) -> Result:
    """
    @brief 获取静态寻位坐标
    @param fileid 寻位文件号
    @param tableid 寻位参数表号
    @param delaytime 参数表延时
    @param pos 输出：寻位坐标
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_pos_trigger_interfer(socketFd: int, pos: VectorDouble, isInterfer: bool) -> Tuple:
    """
    @brief 判断世界坐标系下的某点是否触发干涉区
    @param pos 点的世界坐标 [x, y, z]
    @param return_val 输出：true 触发；false 不触发
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_tool_trigger_interfer(socketFd: int, pos: VectorDouble, isInterfer: bool) -> Tuple:
    """
    @brief 判断工具手（带干涉区立方体）在某位置是否触发干涉区
    @param pos 工具手在世界坐标下的位姿 [x, y, z, a, b, c]
    @param return_val 输出：true 触发；false 不触发
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_sensor_data(socketFd: int, data: VectorInt) -> Tuple:
    """
    @brief 获取六维力传感器的数据
    @param data 输出：传感器数据数组
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_sensor_joint_thresholds(socketFd: int, thresholds: VectorInt) -> Tuple:
    """
    @brief 获取六维力传感器的关节摩擦力补偿阈值
    @param thresholds 输出：6 个关节阈值的数组
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_payload_param_by_sensor(socketFd: int, param: PayloadParamBySensor) -> Tuple:
    """
    @brief 获取通过传感器计算出的负载参数（质量、质心）
    @param payloadParam 输出：负载参数结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_motor_speed(socketFd: int, speed: VectorInt, syncSpeed: VectorInt) -> Tuple:
    """
    @brief 获取当前电机转速
    @param motorSpeed 输出：机器人电机转速，长度 7，单位 [RPM]
    @param motorSpeedSync 输出：外部轴电机转速，长度 5，单位 [RPM]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_motor_payload(socketFd: int, payload: VectorDouble, syncPayload: VectorDouble) -> Tuple:
    """
    @brief 获取当前电机负载
    @param motorPayload 输出：机器人电机负载，长度 7，单位 [%]
    @param motorPayloadSync 输出：外部轴电机负载，长度 5，单位 [%]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_current_encoded_pos(socketFd: int, pos: VectorDouble, syncPos: VectorDouble) -> Tuple:
    """
    @brief 获取当前编码位置
    @param current_encoded 输出：机器人当前编码位置，长度 7
    @param current_encoded_sync 输出：外部轴当前编码位置，长度 5
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_local_pos_e_value(socketFd: int, name: str, pos: VectorDouble) -> Result:
    """
    @brief 获取局部 E 点
    @param name 点位名，范围 E0001 - E9999
    @param pos 输出：转换后的位置，21 位点位
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_joint_voltage(socketFd: int, voltage: VectorDouble, syncVoltage: VectorDouble) -> Tuple:
    """
    @brief 获取关节电压
    @param joint_voltage 输出：机器人本体各关节电压数组
    @param positioner_voltage 输出：外部轴各关节电压数组
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_collision_para(socketFd: int, param: CollisionPara) -> Result:
    """
    @brief 设置碰撞检测阈值
    @param collisionpara
    结构体，包含指令位置响应时间、误差允许时间、碰撞检测阈值（点动/指令）、机器人轴数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_drag_param(socketFd: int, param: DragTorqueParam) -> Result:
    """
    @brief 设置力矩参数（拖拽示教，24.03 版本支持）
    @param param 力矩参数（详细参数在 DragTorqueParam 结构体中说明）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_drag_param(socketFd: int, param: DragTorqueParam) -> Tuple:
    """
    @brief 获取力矩参数（拖拽示教，24.03 版本支持）
    @param param 输出：力矩参数（详细参数在 DragTorqueParam 结构体中说明）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_position_dragParams(socketFd: int, param: DragParam) -> Result:
    """
    @brief 设置位置拖动参数（22.07 版本没有该功能）
    @param param 位置拖动参数（详细参数在 DragParam 结构体中说明；拖动灵敏度与变化率比例
    均为 0~100 的 6 元素向量）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_controller_network_config(socketFd: int, name: str, ip: str, gateway: str, dns: str) -> Result:
    """
    @brief 设置控制器网络配置
    @param name 控制器主机名
    @param ip 新的 IP 地址
    @param gateway 网关地址
    @param dns DNS 服务器地址
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_controller_network_config(socketFd: int, name: str, address: str, gateway: str, dns: str) -> Tuple:
    """
    @brief 获取控制器当前的网络配置信息
    @param name 输出：控制器主机名
    @param address 输出：IP 地址
    @param gateway 输出：网关地址
    @param dns 输出：DNS 服务器地址
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def restore_network_factory_settings(socketFd: int) -> Result:
    """
    @brief 恢复网络出厂设置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- IO（数字 / 模拟 / 远程） ----
def get_io_type(socketFd: int, io_type: IOtype) -> Result:
    """
    @brief IO型号查询
    @param io_type IO型号和端口号，详见 IOtype 结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_digital_output(socketFd: int, port: int, value: int) -> Result:
    """
    @brief 设置数字输出
    @param port 端口号 【1，最大端口数】
    @param value 输出端口值 0 or 1
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如端口号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_digital_output(socketFd: int, out_: VectorInt) -> Result:
    """
    @brief 一次获取所有数字输出
    @param out 存储结果的数组，长度为64
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_digital_input(socketFd: int, in_: VectorInt) -> Result:
    """
    @brief 一次获取所有数字输入
    @param in 存储结果的数组，长度为64
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_analog_output(socketFd: int, port: int, value: float) -> Result:
    """
    @brief 设置模拟输出
    @param port 端口号
    @param value 数值，参数范围：0≤value≤10
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如数值越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_analog_output(socketFd: int, aout: VectorDouble) -> Result:
    """
    @brief 查询模拟输出
    @param aout 模拟输出数组，最大长度为 64
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_analog_input(socketFd: int, ain: VectorDouble) -> Result:
    """
    @brief 查询模拟输入
    @param ain 模拟输入数组，最大长度为 64
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_remote_function(socketFd: int, robotNum: int, general: RemoteControl, program: VectorRemoteProgram, num: int = 10) -> Result:
    """
    @brief 设置远程IO功能
    @param robotNum 机器人编号(1-4)
    @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
    @param program 远程控制程序参数设置, 详见 RemoteProgram, program.size() 必须与 num 相等
    @param num 远程IO数量,若是24.03版本必须与控制器端远程IO参数设置中的 num 一致,22.07版本没有该参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如 program.size() 与 num 不一致）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
    异常；-6=TIMEOUT 超时
    """
def get_remote_function(socketFd: int, robotNum: int, num: int, time: int, general: RemoteControl, program: VectorRemoteProgram) -> Tuple:
    """
    @brief 获取远程IO功能设置数据
    @param robotNum 机器人编号(1-4)
    @param num 远程IO数量,22.07版本没有该参数,调用此接口num将返回-1
    @param time IO重复触发屏蔽时间,单位 ms
    @param general 通用功能远程IO参数设置,如启动、暂停、停止,清除报警等,详见 RemoteControl
    @param program 远程控制程序参数设置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_force_digital_input(socketFd: int, port: int, force: int, value: int) -> Result:
    """
    @brief 设置数字输入端口是否强制打开
    @param port 输入端口
    @param force 是否强制 1 强制 0 非强制
    @param value 设置端口强制状态 0 或 1
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_force_digital_input(socketFd: int, port: VectorInt, status: VectorDouble) -> Tuple:
    """
    @brief 获取当前已打开强制功能的输入端口及其状态
    @param port 当前打开强制输入的端口
    @param status 当前打开强制输入的端口的状态，下标与 port一一对应
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_IO_reset_function(socketFd: int, robotNum: int, type_: int, enable: VectorInt, value: VectorInt) -> Result:
    """
    @brief 设置IO复位功能
    @param robotNum 机器人编号(1-4)
    @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
    @param enable 是否复位容器,
    大小为所有IO板输出端口数,从第二块IO板开始，每一块IO板的起始位置为上一块IO板的末位端口的顺延
    @param value 复位值容器
    @attention 每一次设置,需要设置的端口均要设置，否则会覆盖上一次的修改
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_IO_reset_function(socketFd: int, robotNum: int, type_: int, enable: VectorInt, value: VectorInt) -> Tuple:
    """
    @brief 获取IO复位相关参数
    @param robotNum 机器人编号(1-4)
    @param type 设置类型, 1:远程IO复位, 2:切模式停止, 3:程序报错
    @param enable 是否复位容器
    @param value 复位值容器
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_error_msg_of_digital_input(socketFd: int, msg: VectorAlarmdIO) -> Result:
    """
    @brief 设置IO/报警信息数字输入端口报警功能
    @param msg 消息设置结构体数组,详见 AlarmdIO
    @attention 注意！std::vector<AlarmdIO> msg
    的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则会覆盖上一次的设置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_error_msg_of_digital_input(socketFd: int, msg: VectorAlarmdIO) -> Tuple:
    """
    @brief 获取IO/报警信息数字输入端口报警信息设置
    @param msg 详见 AlarmdIO
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_error_msg_of_digital_output(socketFd: int, msg: VectorAlarmdIO) -> Result:
    """
    @brief 设置IO/报警信息数字输出端口报警功能
    @param msg 消息设置结构体数组,详见 AlarmdIO
    @attention 注意！std::vector<AlarmdIO> msg
    的大小和IO板端口数一致，每一次设置相应端口均不能忽略，否则会覆盖上一次的设置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_error_msg_of_digital_output(socketFd: int, msg: VectorAlarmdIO) -> Tuple:
    """
    @brief 获取IO/报警信息数字输出端口报警信息设置
    @param msg 详见 AlarmdIO
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_remote_param(socketFd: int, robotNum: int, speed: int, start: bool, time: int, startTime: int, num: int = 10) -> Result:
    """
    @brief 远程参数设置
    @param robotNum 机器人编号(1-4)
    @param speed 远程模式默认速度[1,100]
    @param start 是否自动启动
    @param time IO重复触发屏蔽时间,单位 ms
    @param startTime 启动确认时间
    @param num 远程IO数量,22.07版本没有该参数,默认为10,24.03版本以上可修改该参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_remote_param(socketFd: int, robotNum: int, speed: int, start: bool, time: int, startTime: int, num: int) -> Tuple:
    """
    @brief 获取远程参数设置
    @param robotNum 机器人编号(1-4)
    @param speed 远程模式默认速度[1,100]
    @param start 是否自动启动
    @param time IO重复触发屏蔽时间,单位 ms
    @param startTime 启动确认时间
    @param num 远程IO数量,22.07版本没有该参数,22.07版本调用此接口num将返回-1
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_remote_status_tips(socketFd: int, robotNum: int, outagePort: int, outageValue: int, program: VectorRemoteProgram) -> Result:
    """
    @brief 设置远程状态提示功能
    @param robotNum 机器人编号(1-4)
    @param outagePort 断电保持数据恢复端口
    @param outageValue 断电保持数据恢复端口的有效值(0/1/2)
    @param program 远程状态提示参数设置,详见 RemoteProgram
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_remote_status_tips(socketFd: int, robotNum: int, num: int, outagePort: int, outageValue: int, program: VectorRemoteProgram) -> Tuple:
    """
    @brief 获取远程状态提示功能数据
    @param robotNum 机器人编号(1-4)
    @param num 远程IO数量(22.07版本默认10个，24.03版本以上可以设置该数量)
    @param outagePort 断电保持数据恢复端口
    @param outageValue 断电保持数据恢复端口有效值(0/1/2)
    @param program 远程状态提示参数设置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_remote_program(socketFd: int, robotNum: int, program: VectorRemoteProgramSetting) -> Result:
    """
    @brief IO远程程序选择
    @param robotNum 机器人编号(1-4)
    @param program 详见 RemoteProgramSetting
    @attention program.size() 22.07版本固定为10
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_remote_program(socketFd: int, robotNum: int, num: int, program: VectorRemoteProgramSetting) -> Tuple:
    """
    @brief 获取IO远程程序设置数据
    @param robotNum 机器人编号(1-4)
    @param num 远程IO数量(22.07版本默认10个，最多也只能是10个,24.03版本以上可以设置该数量)
    @param program 详见 RemoteProgramSetting
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_hard_enable_port(socketFd: int, enable: int, port1: int, port2: int) -> Result:
    """
    @brief 设置是否使能硬接及相关端口
    @param enable 是否打开使能硬接
    @param port1 使能硬接端口1
    @param port2 使能硬接端口2
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_hard_enable_port(socketFd: int, enable: int, port1: int, port2: int) -> Tuple:
    """
    @brief 获取使能硬接开关是否打开及相关绑定端口
    @param enable 是否打开使能硬接
    @param port1 使能硬接端口1
    @param port2 使能硬接端口2
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_safe_IO_function(socketFd: int, robotNum: int, safeIO: SafeIO) -> Result:
    """
    @brief 设置IO安全设置参数
    @param robotNum 机器人编号(1-4)
    @param safeIO 详见 SafeIO 结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_safe_IO_function(socketFd: int, robotNum: int, safeIO: SafeIO) -> Tuple:
    """
    @brief 获取IO安全设置参数
    @param robotNum 机器人编号(1-4)
    @param safeIO 详见 SafeIO 结构体
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- 高频伺服（servoJ / servo_movej / servo_movel） ----
def open_servo_movej(socket_udp: int, vmax_pct: float = 30.0, amax_pct: float = 30.0, jmax_pct: float = 30.0) -> Result:
    """
    @brief 开启伺服跟踪模式（servo_movej 前置）
    
    将百分比速度约束换算为绝对约束并开启底层 servoJ 跟踪，
    同时启动后台 250Hz 插值发送线程（引用计数管理，可嵌套开启）。
    
    @param socket_udp UDP socket（7000 伺服端口）
    @param vmax_pct 最大速度百分比，范围 (0,100]，默认 30
    @param amax_pct 最大加速度百分比，范围 (0,100]，默认 30
    @param jmax_pct 最大加加速度百分比，范围 (0,100]，默认 30
    @return SUCCESS / DISCONNECT（未连接）/ 底层 open_servoJ 错误码
    
    @warning 需先连接 7000 端口；与 close_servo_movej 成对调用
    """
def close_servo_movej(socket_udp: int) -> Result:
    """
    @brief 关闭伺服跟踪模式（servo_movej）
    @param socket_udp UDP socket（7000 伺服端口）
    @return SUCCESS / 底层 close_servoJ 错误码
    @note 引用计数归零时停止后台发送线程并关闭 servoJ
    """
def open_servo_movel(socket_udp: int, vmax_pct: float = 30.0, amax_pct: float = 30.0, jmax_pct: float = 30.0) -> Result:
    """
    @brief 开启伺服跟踪模式（servo_movel 前置）
    
    与 open_servo_movej 行为一致（同一底层 servoJ），仅作为 servo_movel 的配套开关。
    
    @param socket_udp UDP socket（7000 伺服端口）
    @param vmax_pct 最大速度百分比，范围 (0,100]，默认 30
    @param amax_pct 最大加速度百分比，范围 (0,100]，默认 30
    @param jmax_pct 最大加加速度百分比，范围 (0,100]，默认 30
    @return SUCCESS / DISCONNECT（未连接）/ 底层 open_servoJ 错误码
    
    @warning 需先连接 7000 端口；与 close_servo_movel 成对调用
    """
def close_servo_movel(socket_udp: int) -> Result:
    """
    @brief 关闭伺服跟踪模式（servo_movel）
    @param socket_udp UDP socket（7000 伺服端口）
    @return SUCCESS / 底层 close_servoJ 错误码
    @note 引用计数归零时停止后台发送线程并关闭 servoJ
    """
@overload
def servo_movej(socket_tcp: int, socket_udp: int, target: VectorDouble, coord: int = 0, step_size: float = 0.72) -> Result: ...
@overload
def servo_movej(socket_tcp: int, socket_udp: int, target: VectorDouble, coord: Coord, step_size: float = 0.72) -> Result:
    """
    @brief Servo MoveJ 关节空间伺服运动（枚举坐标系重载）
    
    语义与 int 版本一致，仅 coord 参数以 Coord 枚举传入。
    
    @param socket_tcp TCP socket（6001 控制端口）：仅用于查询当前关节角/逆解
    @param socket_udp UDP socket（7000 伺服端口）：servoJ 数据发送
    @param target 目标关节角，6 或 7 个元素（7 轴含真实 J7；6 轴时第 7 位补 0）
    @param coord 坐标系，仅支持 Coord::JOINT（关节角直接插值）/
    Coord::BASE（基坐标系，先逆解转关节角）
    @param step_size 插值步长（关节角，度），<=0 时使用默认 0.72
    @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movej）/
            DISCONNECT（未连接）/ PARAM_ERR（参数错误）
    
    @warning 需先调用 open_servo_movej 开启跟踪模式
    """
@overload
def servo_movel(socket_tcp: int, socket_udp: int, target_pose: VectorDouble, coord: int = 1, step_size: float = 2.0) -> Result: ...
@overload
def servo_movel(socket_tcp: int, socket_udp: int, target_pose: VectorDouble, coord: Coord, step_size: float = 2.0) -> Result:
    """
    @brief Servo MoveL 笛卡尔直线伺服运动（枚举坐标系重载）
    
    语义与 int 版本一致，仅 coord 参数以 Coord 枚举传入。
    
    @param socket_tcp TCP socket（6001 控制端口）：仅用于查询当前位姿/逆解
    @param socket_udp UDP socket（7000 伺服端口）：servoJ 数据发送
    @param target_pose 目标位姿 [X,Y,Z,RX,RY,RZ]（mm, rad），6 或 7 个元素
    @param coord 坐标系，仅支持 Coord::JOINT（按关节角直线插值）/ Coord::BASE（基坐标系直线插值）
    @param step_size 插值步长（关节角，度），<=0 时使用默认 0.72
    @return SUCCESS / OPERATION_NOT_ALLOWED（未先 open_servo_movel）/
            DISCONNECT（未连接）/ PARAM_ERR（参数错误）/ 逆解失败错误码
    
    @warning 需先调用 open_servo_movel 开启跟踪模式
    """
def wait_servo() -> None:
    """
    @brief 阻塞等待排队运动执行完毕
    
    等待当前批次发送完成且队列为空；未开启跟踪模式或已关闭时立即返回。
    """
def cancel_servo() -> None:
    """
    @brief 取消当前运动并清空排队任务
    
    置取消标志并清空队列，当前批次在下一个插值点停止发送。
    """
def servo_busy() -> bool:
    """
    @brief 是否有任务执行或排队中
    @return true=后台线程正在发送或队列非空 / false=空闲或跟踪模式未开启
    """
def open_servoJ(socketFd: int, vmax: VectorDouble, amax: VectorDouble, jmax: VectorDouble) -> Result:
    """
    @brief 打开关节跟踪模式（底层 servoJ 透传）
    @param socketFd UDP socket（7000 伺服端口）
    @param vmax 速度约束，7 元素向量（度/秒）
    @param amax 加速度约束，7 元素向量（度/秒²）
    @param jmax 加加速度约束，7 元素向量（度/秒³）
    @return SUCCESS / PARAM_ERR（任一约束向量长度 ≠ 7）/ 底层错误码
    @note SDK 底层协议统一使用 7 元素 vector 兼容 6/7 轴机械臂，三个约束向量必须固定为 7 元素：
          6 轴机器人在第 7 元素（索引 6）补 0（外部轴），7 轴机器人第 7 元素为真实 J7 约束值。
          与 set_servoJ_pos 的 q 长度约定保持一致。
    @warning 约束向量传 6 元素会导致 vector marshaling 长度不足，控制器协议解析错乱并断开 7000 连接；
          不要硬编码 6 轴写死，应根据实际轴数动态写入约束值，剩余元素保持 0（vector 初始化即置零）
    """
def close_servoJ(socketFd: int) -> Result:
    """
    @brief 关闭关节跟踪模式（底层 servoJ 透传）
    @param socketFd UDP socket（7000 伺服端口）
    @return SUCCESS / 底层错误码
    """
def set_servoJ_pos(socketFd: int, q: VectorDouble) -> Result:
    """
    @brief 发送跟踪关节位置（底层 servoJ 透传，10ms 周期高频调用）
    @param socketFd UDP socket（7000 伺服端口）
    @param q 目标关节角，7 元素向量（度）；6 轴机器人在第 7 元素（索引 6）补 0（外部轴），
             7 轴机器人第 7 元素为真实 J7 关节角
    @return SUCCESS / 底层错误码
    @note 与 open_servoJ 的 vmax/amax/jmax 长度约定一致：固定 7 元素，
          按实际轴数动态写入，剩余元素保持 0（vector 初始化即置零）
    """
def open_servo_csv_mode(socketFd: int) -> Result:
    """
    @brief 打开周期同步速度模式（CSV）
    @note 需控制器 24.03 及以上版本（VERSION_2403 / VERSION_DEV）
    @param socketFd socket（7000 伺服端口）
    @return SUCCESS / 底层错误码
    @warning 需先连接 7000 端口；与 close_servo_csv_mode 成对调用；同一 7000 连接上不要与
    servoJ / servo_movej / servo_movel / 恒力驻留等其他伺服模式同时使用，切换前先关闭当前模式
    """
def close_servo_csv_mode(socketFd: int) -> Result:
    """
    @brief 关闭周期同步速度模式
    @param socketFd socket（7000 伺服端口）
    @return SUCCESS / 底层错误码
    """
def set_servo_csv_velocity(socketFd: int, targetVelocity: VectorDouble) -> Result:
    """
    @brief 下发周期同步速度（CSV 模式下周期调用）
    @note 与 open_servoJ 的 7 元素约定一致
    @param socketFd socket（7000 伺服端口）
    @param targetVelocity 各关节目标速度，7 元素向量（度/秒）；6 轴机器人在第 7 元素
    （索引 6）补 0（外部轴），7 轴机器人第 7 元素为真实 J7 速度
    @return SUCCESS / PARAM_ERR（向量长度 ≠ 7）/ 底层错误码
    """
def open_servo_cst_mode(socketFd: int) -> Result:
    """
    @brief 打开周期同步扭矩模式（CST）
    @note 需控制器 24.03 及以上版本（VERSION_2403 / VERSION_DEV）
    @param socketFd socket（7000 伺服端口）
    @return SUCCESS / 底层错误码
    @warning 需先连接 7000 端口；与 close_servo_cst_mode 成对调用；同一 7000 连接上不要与
    servoJ / servo_movej / servo_movel / 恒力驻留等其他伺服模式同时使用，切换前先关闭当前模式
    """
def close_servo_cst_mode(socketFd: int) -> Result:
    """
    @brief 关闭周期同步扭矩模式
    @param socketFd socket（7000 伺服端口）
    @return SUCCESS / 底层错误码
    """
def set_servo_cst_torque(socketFd: int, targetTorque: VectorDouble) -> Result:
    """
    @brief 下发周期同步扭矩（CST 模式下周期调用）
    @note 与 open_servoJ 的 7 元素约定一致
    @param socketFd socket（7000 伺服端口）
    @param targetTorque 各关节目标扭矩，7 元素向量；单位为额定扭矩的 0.1%，范围 [-1000,1000]；
    6 轴机器人在第 7 元素（索引 6）补 0（外部轴），7 轴机器人第 7 元素为真实 J7 扭矩
    @return SUCCESS / PARAM_ERR（向量长度 ≠ 7）/ 底层错误码
    """
# ---- 恒力驻留控制 ----
def open_constforce(socket_tcp: int, socket_udp: int, base_pose: VectorDouble, admittance: AdmittanceController, sensor_callback: Callable[..., object], params: ConstantForceParams, sensor_user_data: Optional[object] = None) -> Result:
    """
    @brief 开启恒力驻留控制（异步，立即返回）
    
    后台线程持续运行，无时长上限：
      sensor_callback → EMA 滤波 → 死区 → 导纳补偿 → 工具系→基系旋转 → IK → set_servoJ_pos
    
    @param socket_tcp       TCP socket (6001)：仅 IK 使用
    @param socket_udp       UDP socket (7000)：servoJ
    @param base_pose        初始目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，后台闭环以此为基准；
                            运行中可用 set_constforce_target_pose 实时更新
    @param admittance       已 Configure + SetDesiredWrench（含重力补偿 + 目标力）的导纳实例
    @param sensor_callback  六维力传感器读取回调（nullptr → PARAM_ERR）
    @param sensor_user_data 回调透传指针，可为 nullptr
    @param params           恒力参数（force_control_axis 指定时校验导纳刚度 K=0）
    @return SUCCESS / PARAM_ERR（参数错误）/ DISCONNECT（未连接）/
    OPERATION_NOT_ALLOWED（已在运行或与 servo 异步模式冲突）
    
    @warning admittance 必须存活到 close_constforce() 返回之后才能销毁
    @warning F_desired 必须在 open_constforce() 前通过 SetDesiredWrench
    设置，运行中不支持修改（并发数据竞争）
    @warning 与 servo_movej/movel 异步模式互斥：两者均写 7000 端口，检测到冲突返回
    OPERATION_NOT_ALLOWED
    @warning 互斥检测仅恒力侧进行（open_constforce 检查异步模式）；反向（恒力运行中调用
             open_servo_movej/movel）不受保护，须由调用方保证：先 close_constforce 再开异步伺服
    @warning 控制频率由 admittance.GetControlPeriod() 决定，循环定时与导纳积分共享同一时间源
    @warning open_constforce 与 close_constforce 需串行调用（非线程安全）
    """
def set_constforce_target_pose(target_pose: VectorDouble) -> Result:
    """
    @brief 实时更新恒力控制的目标位姿（运动接口）
    
    后台闭环每周期读取最新目标位姿，下一周期即以新目标为基准做导纳补偿。
    主线程可据此实现任意运动（直线移动、扫动、圆弧等），轨迹平滑由调用方保证
    （大跨度跳变会直接反映到关节指令上）。
    
    @param target_pose 新目标位姿 [X,Y,Z,RX,RY,RZ] (mm, rad)，至少 6 个元素
    @return SUCCESS / PARAM_ERR（长度不足或含非有限值）/ OPERATION_NOT_ALLOWED（未在运行）
    
    @warning 仅恒力控制运行中（open_constforce 后、close_constforce 前）可调用
    @warning 与导纳 F_desired 类似，目标位姿更新不保证原子性之外的平滑性，
             大跳变请由调用方分步逼近
    """
def get_constforce_status(status: ConstForceStatus) -> Result:
    """
    @brief 查询恒力控制运行时状态（线程安全）
    
    返回后台闭环最近一个控制周期更新的快照：运行标志、原始/滤波后六维力、
    当前目标位姿、结束原因。未 open 时返回 SUCCESS 且 running=false。
    
    @param status [out] 状态快照
    @return SUCCESS
    """
def close_constforce() -> Result:
    """
    @brief 关闭恒力控制并阻塞等待后台循环退出（幂等，可随时调用）
    
    三种场景：
      1. 后台循环运行中 → 置停止标志 → join → 返回结束原因
      2. 线程已自行退出（断连超时 / open_servoJ 失败）→ join 已退出线程 → 返回结束原因
      3. 未 open 过或已 close 过 → 无可清理内容，返回 SUCCESS
    
    @return SUCCESS（用户主动关闭，或无可清理内容）/ EXCEPTION（传感器断连超时自动停止）/ open_servoJ
    原始错误码
    
    @note 必须调用本函数完成清理（join 后台线程）；不调用则 std::thread 泄漏，
          进程退出时 std::terminate。
    @warning 禁止在传感器回调（ForceSensorCallback）内调用本函数：回调运行于后台
             线程，此处 join 自身线程会抛出 std::system_error 并终止进程。
             回调中需要停止时请返回 false（计入断连计数，触发安全自动停止）。
    """
# ---- 队列运动 ----
def queue_motion_set_status(socketFd: int, status: bool) -> Result:
    """
    @brief 打开or关闭控制器的队列运动模式
    @param status 0-关闭 1-打开
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @warning 无论是打开还是关闭，都将清空远端控制器已经存储的队列
    @note 打开队列模式（status=true）时，控制器会先执行伺服下电再上电；
          关闭队列模式（status=false）退出后，控制器会执行伺服下电，
          后续如需继续运动需重新执行上电流程（set_servo_state(1) → set_servo_poweron）
    """
def queue_motion_get_status(socketFd: int, status: bool) -> Tuple:
    """
    @brief 查询控制器当前是否打开队列运动模式
    @param status 用来存储返回值 0-关闭 1-已打开
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_clear_Data(socketFd: int) -> Result:
    """
    @brief 清空缓存的运动队列数据
    @note 仅清除上位机侧缓存的队列数据，不清除已经下发给控制器的队列；
          控制器已接收的队列会继续执行，如需停止需调用 queue_motion_stop
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_size(socketFd: int, size: int) -> Tuple:
    """
    @brief 查询当前运动队列的长度
    @param[out] size 当前运动队列长度
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_get_queuelen(socketFd: int, len: int) -> Tuple:
    """
    @brief 查询当前运动队列剩余的指令数量
    @param[out] len 当前运动队列剩余的指令数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_send_to_controller(socketFd: int, size: int, isContinue: bool = False) -> Result:
    """
    @brief 将本地队列的前size个数据发送到控制器
    @param size 要发送的队列大小 size = 0时将当前运动队列全部指令发送  范围[0,31];
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（传入的size超过队列长度）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
    异常；-6=TIMEOUT 超时
    @warning 控制器接受到队列后，将会立马开始运动。调用前请先调用queue_motion_set_status(true);
    @iscontinue 是否选择继续发送，true -继续；false
    -不继续，继续发送时可以继续发送点位，机器人不运动;不继续时机器人接收点位后立刻运动,默认为false
    """
def queue_motion_suspend(socketFd: int) -> Result:
    """
    @brief 暂停连续运动
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_restart(socketFd: int) -> Result:
    """
    @brief 暂停后再次运行运动队列
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_stop_not_power_off(socketFd: int) -> Result:
    """
    @brief 停止连续运动保持上电状态
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_moveJ(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条moveJ运动
    @param moveCmd 运动指令结构体，字段说明：
     targetPos 目标点坐标 长度7位
     velocity 参数范围(1,100] 度/s
     acc，dec 参数范围(1,100]
     pl 多条轨迹之间平滑参数 参数范围[0,5]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_moveL(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条moveL运动
    @param moveCmd 运动指令结构体，字段说明：
     targetPos 目标点坐标 长度7位
     velocity 参数范围(1,1000] mm/s
     acc，dec 参数范围(1,100]
     pl 多条轨迹之间平滑参数 参数范围[0,5]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_moveC(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条moveC运动
    @param moveCmd 运动指令结构体，字段说明：
     targetPos 目标点坐标 长度7位
     velocity 参数范围(1,1000] mm/s
     acc，dec 参数范围(1,100]
     pl 多条轨迹之间平滑参数 参数范围[0,5]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_moveS(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条moveS曲线运动
    @param moveCmd 运动指令，详见 MoveCmd
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_moveCA(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条moveCA运动
    @param moveCmd 运动指令结构体，字段说明：
     targetPos 目标点坐标 长度7位
     velocity 参数范围(1,1000] mm/s
     acc，dec 参数范围(1,100]
     pl 多条轨迹之间平滑参数 参数范围[0,5]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_imove(socketFd: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条imove增量运动
    @param moveCmd 运动指令结构体，字段说明：
     targetPos 目标点坐标 长度7+7 本体7位 外部轴7位
     velocity 参数范围(1,1000] mm/s
     acc，dec 参数范围(1,100]
     pl 多条轨迹之间平滑参数 参数范围[0,5]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_samov(socketFd: int, moveCmd: MoveCmd, posData: Any) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条SAMOV(定点移动)指令
    @param moveCmd 运动指令结构体
    @param posData 点位数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_cil(socketFd: int, moveCmd: MoveCmd, id: int) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条CIL(相贯线)指令
    @param moveCmd 运动指令结构体
    @param id 工艺号
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_moveComm(socketFd: int, moveType: str, m_vel: float, m_acc: float, m_dec: float, m_time: int, m_pl: int) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入外部点指令
    @param moveType 插补方式 "MovJ", "MovL"
    @param m_vel 速度 "MOVJ"速度范围[1, 100] "MOVL"速度范围[1, 1000]
    @param m_acc 加速度
    @param m_dec 减速度
    @param m_time 时间
    @param m_pl 平滑参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_until(socketFd: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条UNTIL(直到)指令
    @param conditionGroups 条件组的二维向量
    @param logic 第一层的逻辑类型，0为"与"，1为"或"
    @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_end_until(socketFd: int) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条ENDUNTIL(结束直到)指令
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_while(socketFd: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条WHILE(循环)指令
    @param conditionGroups 条件组的二维向量
    @param logic 第一层的逻辑类型，0为"与"，1为"或"
    @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_end_while(socketFd: int) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条ENDWHILE(结束循环)指令
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_if(socketFd: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条IF(如果)指令
    @param conditionGroups 条件组的二维向量
    @param logic 第一层的逻辑类型，0为"与"，1为"或"
    @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_end_if(socketFd: int) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条ENDIF(结束如果)指令
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_label(socketFd: int, label: str) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条LABEL(标签)指令
    @param label 标签名字
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def queue_motion_push_back_jump(socketFd: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable, jumpConditionFlag: bool, label: str) -> Result:
    """
    @brief 队列运动模式的本地队列最后插入一条JUMP(跳转)指令
    @param conditionGroups 条件组的二维向量
    @param logic 第一层的逻辑类型，0为"与"，1为"或"
    @param logicGroup 第二层的逻辑类型，0为"与"，1为"或"
    @param jumpConditionFlag 是否使用跳转条件
    @param label 跳转目标标签名字
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- 作业文件 ----
def job_get_all_jobfile_name(socketFd: int, robotsFile: VectorVectorString) -> Result:
    """
    @brief 获取所有作业文件名
    @param robotsFile 二维数组，一维长度是4，对应4个机器人
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_create(socketFd: int, jobName: str) -> Result:
    """
    @brief 新建作业文件
    @param jobName 作业文件名 只允许字母开头，字母数字组合
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如作业文件名不合法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
    异常；-6=TIMEOUT 超时
    @test 新建QQQ.JBR job_create(SOCKETFD socketFd,"QQQ");
    """
def job_delete(socketFd: int, jobName: str) -> Result:
    """
    @brief 删除指定的作业文件
    @param jobName 作业文件名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @test 删除QQQ.JBR job_delete(SOCKETFD socketFd,"QQQ");
    """
def job_open(socketFd: int, jobName: str) -> Result:
    """
    @brief 打开指定的作业文件
    @param jobName 作业文件名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @test 打开QQQ.JBR job_open(SOCKETFD socketFd,"QQQ");
    """
def job_run(socketFd: int, jobName: str) -> Result:
    """
    @brief 运行指定的作业文件
    @param jobName 作业文件名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @test 运行QQQ.JBR job_run(SOCKETFD socketFd,"QQQ");
    """
def job_upload_by_file(socketFd: int, filePath: str) -> Result:
    """
    @brief 根据文件名上传一个作业文件
    @param filePath 文件的完整路径
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如文件不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_download_by_directory(socketFd: int, directoryPath: str, isCover: bool) -> Result:
    """
    @brief 下载所有作业文件到指定文件夹
    @param directoryPath 目录的完整路径
    @param isCover 是否覆盖同名文件，true=覆盖 / false=不覆盖
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如目录不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def log_download_by_quantity(socketFd: int, counts: int, directoryPath: str) -> Result:
    """
    @brief 下载指定数量的日志文件到指定文件夹
    @param counts 文件数量
    @param directoryPath 目录的完整路径
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_moveJ(socketFd: int, line: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 向作业文件插入一条moveJ关节运动
    @param line 插入的行号
    @param moveCmd 运动指令参数，详见 MoveCmd
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
    异常；-6=TIMEOUT 超时
    """
def job_insert_moveL(socketFd: int, line: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 向作业文件插入一条moveL
    @param line 插入的行号
    @param moveCmd 运动指令参数，详见 MoveCmd
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如行号越界或参数非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION
    异常；-6=TIMEOUT 超时
    """
def job_insert_moveC(socketFd: int, line: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 向作业文件插入一条moveC
    @param line 插入的行号
    @param moveCmd 运动指令参数，详见 MoveCmd
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_imove(socketFd: int, line: int, moveCmd: MoveCmd) -> Result:
    """
    @brief 向作业文件插入一条增量指令IMOVE
    @param line 插入的行号
    @param moveCmd 运动指令参数，详见 MoveCmd
    @note moveCmd.targetPosType 必须设置为 PosType::RP_TYPE(3)，moveCmd.targetPosName 如 "RP0001"
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_upload_by_directory(socketFd: int, directoryPath: str) -> Result:
    """
    @brief 根据文件夹上传一整个文件夹的作业文件
    @param directoryPath 目录的完整路径
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如目录不存在）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_sync_job_file(socketFd: int) -> Result:
    """
    @brief 上传作业文件后同步刷新示教器中的作业列表
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def backup_system(socketFd: int) -> Result:
    """
    @brief 一键备份系统，备份文件保存至当前执行程序所在目录下
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_step(socketFd: int, jobName: str, line: int) -> Result:
    """
    @brief 单步运行指定作业文件的某一行
    @param jobName 作业文件名
    @param line 行号，范围 [1, 最大行号]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如行号越界）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    @test 运行 QQQ.JBR 的第一行 job_step(socketFd, "QQQ", 1);
    """
def job_pause(socketFd: int) -> Result:
    """
    @brief 暂停作业文件
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_continue(socketFd: int) -> Result:
    """
    @brief 继续运行已暂停的作业文件
    @note 需要处于运行模式
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_stop(socketFd: int) -> Result:
    """
    @brief 停止作业文件（不会下电）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_run_times(socketFd: int, index: int) -> Result:
    """
    @brief 设置作业文件运行次数
    @param index 运行次数，0 表示无限次
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_break_point_run(socketFd: int, jobName: str) -> Result:
    """
    @brief 从断点处继续运行作业文件
    @param jobName 作业文件名
    @note 需要处于运行模式
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_get_current_file(socketFd: int, jobName: str) -> Tuple:
    """
    @brief 获取当前打开的作业文件名称
    @param jobName 输出参数，当前打开的作业文件名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_get_current_file_csharp(socketFd: int, jobName: List[str]) -> Tuple:
    """
    @brief 获取当前打开的作业文件名称（字符容器形式，供 C# 侧使用）
    @param jobName 输出参数，当前打开的作业文件名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_get_current_line(socketFd: int, line: int) -> Tuple:
    """
    @brief 获取当前打开的作业文件运行到的行数
    @param line 输出参数，运行到的行数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_local_position(socketFd: int, posData: Any) -> Result:
    """
    @brief 向作业文件插入一个局部点位
    @param posData 点位数据参数（key/posData/coord/toolNum/userNum/type，完整定义见协议层参数头）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误（如点位数据非法）；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT
    超时
    """
def job_insert_moveComm(socketFd: int, line: int, moveType: str, m_vel: float, m_acc: float, m_dec: float, m_time: int, m_pl: int) -> Result:
    """
    @brief 向作业文件插入一条外部点指令 moveComm
    @param line 插入的位置
    @param moveType 插补方式，如 "MovJ"、"MovL"
    @param m_vel 速度
    @param m_acc 加速度
    @param m_dec 减速度
    @param m_time 时间
    @param m_pl 平滑级别
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_samov_command(socketFd: int, line: int, moveCmd: MoveCmd, posData: Any) -> Result:
    """
    @brief 向作业文件插入一条 SAMOV（定点移动）指令
    @param line 插入的位置
    @param moveCmd 运动指令参数，详见 MoveCmd
    @param posData 点位数据参数（详见协议层参数头 PositionData）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_timer_command(socketFd: int, line: int, time: float) -> Result:
    """
    @brief 在作业文件中插入一条延时指令
    @param line 插入的位置
    @param time 延时时间，单位 s
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_io_out_command(socketFd: int, line: int, params: Any) -> Result:
    """
    @brief 在作业文件中插入一条 IO 输出指令
    @param line 插入的位置
    @param params IO
    输出参数（groupType/errorHanding/paraGroupNum/paraGroupTime/paraGroupValue，完整定义见协议层参数头）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_cil(socketFd: int, line: int, moveCmd: MoveCmd, id: int) -> Result:
    """
    @brief 向作业文件插入一条 CIL（相贯线）指令
    @param line 插入的位置
    @param moveCmd 运动指令参数，详见 MoveCmd
    @param id 工艺号参数
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_until(socketFd: int, line: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 向作业文件插入一条 UNTIL（直到）指令
    @param line 插入的位置
    @param conditionGroups 条件组的二维向量
    @param logic 第一层逻辑类型，0 为“与”，1 为“或”
    @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_end_until(socketFd: int, line: int) -> Result:
    """
    @brief 向作业文件插入一条 ENDUNTIL（结束直到）指令
    @param line 插入的位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_while(socketFd: int, line: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 向作业文件插入一条 WHILE 指令
    @param line 插入的位置
    @param conditionGroups 条件组的二维向量
    @param logic 第一层逻辑类型，0 为“与”，1 为“或”
    @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_end_while(socketFd: int, line: int) -> Result:
    """
    @brief 向作业文件插入一条 ENDWHILE（结束循环）指令
    @param line 插入的位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_if(socketFd: int, line: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable) -> Result:
    """
    @brief 向作业文件插入一条 IF 指令
    @param line 插入的位置
    @param conditionGroups 条件组的二维向量
    @param logic 第一层逻辑类型，0 为“与”，1 为“或”
    @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_end_if(socketFd: int, line: int) -> Result:
    """
    @brief 向作业文件插入一条 ENDIF（结束条件）指令
    @param line 插入的位置
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_label(socketFd: int, line: int, label: str) -> Result:
    """
    @brief 向作业文件插入一条 LABEL（标签）指令
    @param line 插入的位置
    @param label 标签名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_jump(socketFd: int, line: int, conditionGroups: Iterable, logic: Iterable, logicGroup: Iterable, jumpConditionFlag: bool, label: str) -> Result:
    """
    @brief 向作业文件插入一条 JUMP（跳转）指令
    @param line 插入的位置
    @param conditionGroups 条件组的二维向量
    @param logic 第一层逻辑类型，0 为“与”，1 为“或”
    @param logicGroup 第二层逻辑类型，0 为“与”，1 为“或”
    @param jumpConditionFlag 是否带跳转条件
    @param label 跳转的目标标签名
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_tasks(socketFd: int, line: int, crafId: int, robotGroup: VectorInt) -> Result:
    """
    @brief 向作业文件插入声明协调参数指令
    @param line 插入的位置
    @param crafId 协调任务号，范围 1-999
    @param robotGroup 需要同步的机器人编号列表（1-4）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def job_insert_wait_sync_task(socketFd: int, line: int, crafId: int, waitSignal: str) -> Result:
    """
    @brief 向作业文件插入等待任务同步点指令
    @param line 插入的位置
    @param crafId 协调任务号，范围 1-999
    @param waitSignal 同步信号量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- 轨迹记录 ----
def drag_trajectory_export(socketFd: int, directoryPath: str) -> Result:
    """
    @brief 导出控制器中的全部拖动轨迹
    @param directoryPath 本地保存目录；末级目录不存在时自动创建，父目录须存在
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def drag_trajectory_import(socketFd: int, directoryPath: str) -> Result:
    """
    @brief 导入本地目录中的全部拖动轨迹
    @note 控制器中存在同名轨迹时将被覆盖
    @param directoryPath 包含 .json 轨迹文件的本地目录
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def drag_trajectory_playback(socketFd: int, trajectoryName: str, velocity: int) -> Result:
    """
    @brief 回放指定的拖动轨迹
    @param trajectoryName 控制器 dragPos 目录中的轨迹文件名，包含 .json 后缀
    @param velocity 回放速度，范围 [1,100]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def drag_trajectory_delete(socketFd: int, trajectoryName: str) -> Result:
    """
    @brief 删除指定的拖动轨迹
    @note 删除后不可恢复
    @param trajectoryName 控制器 dragPos 目录中的轨迹文件名，包含 .json 后缀
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def track_record_start(socketFd: int, maxSamplingNum: float, samplingInterval: float) -> Result:
    """
    @brief 轨迹记录开始
    @param maxSamplingNum 最大采样点数 [200,12000]
    @param samplingInterval 采样间隔  [0.03,1]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def track_record_stop(socketFd: int) -> Result:
    """
    @brief 轨迹记录关闭
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def get_track_record_status(socketFd: int, recordStart: bool) -> Tuple:
    """
    @brief 轨迹记录开启状态查询
    @param recordStart 记录状态  true 开启  false 关闭
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def track_record_save(socketFd: int, trajName: str) -> Result:
    """
    @brief 轨迹记录保存
    @param trajName 保存的轨迹名称
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def track_record_playback(socketFd: int, vel: int) -> Result:
    """
    @brief 轨迹回放
    @param vel 回放速度
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def track_record_delete(socketFd: int) -> Result:
    """
    @brief 轨迹清除
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- Modbus 主站 ----
def modbus_set_master_parameter(socketFd: int, id: int, param: ModbusMasterParameter) -> Result:
    """
    @brief 设置主站参数
    @param id 配方id参数，最多保存9个id
    @param param 主站参数，详见 ModbusMasterParameter（type："TCP"/"RTU"）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_open_master(socketFd: int, id: int) -> Result:
    """
    @brief 打开主站
    @param id 配方id参数，最多保存9个id，需与 modbus_set_master_parameter 设置的 id 对应
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_read_input_status(socketFd: int, id: int, address: int, quantity: int, data: VectorInt) -> Result:
    """
    @brief 读取输入状态（功能码 02H）
    @param id 配方id参数，最多保存9个id
    @param address 起始地址
    @param quantity 读取数量
    @param data 读取数据（失败时输出向量被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_read_coil_status(socketFd: int, id: int, address: int, quantity: int, data: VectorInt) -> Result:
    """
    @brief 读取线圈状态（功能码 01H）
    @param id 配方id参数，最多保存9个id
    @param address 起始地址
    @param quantity 读取数量
    @param data 读取数据（失败时输出向量被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_read_holding_registers(socketFd: int, id: int, address: int, quantity: int, data: VectorInt) -> Result:
    """
    @brief 读取保持寄存器（功能码 03H）
    @param id 配方id参数，最多保存9个id
    @param address 起始地址
    @param quantity 读取数量
    @param data 读取数据（失败时输出向量被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_read_input_registers(socketFd: int, id: int, address: int, quantity: int, data: VectorInt) -> Result:
    """
    @brief 读取输入寄存器（功能码 04H）
    @param id 配方id参数，最多保存9个id
    @param address 起始地址
    @param quantity 读取数量
    @param data 读取数据（失败时输出向量被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_write_signal_coil_status(socketFd: int, id: int, address: int, data: int) -> Result:
    """
    @brief 写单个线圈状态（功能码 05H）
    @param id 配方id参数，最多保存9个id
    @param address 线圈地址
    @param data 写入的线圈状态值
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_write_signal_holding_registers(socketFd: int, id: int, address: int, data: int) -> Result:
    """
    @brief 写单个保持寄存器（功能码 06H）
    @param id 配方id参数，最多保存9个id
    @param address 寄存器地址
    @param data 写入的寄存器数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_write_multiple_holding_registers(socketFd: int, id: int, address: int, data: VectorInt) -> Result:
    """
    @brief 写多个保持寄存器（功能码 10H）
    @param id 配方id参数，最多保存9个id
    @param address 起始寄存器地址
    @param data 写入的寄存器数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_terminal_io_can_param(socketFd: int, id: int, port: int, arbitration_baudrate: int, data_baudrate: int) -> Result:
    """
    @brief 设置末端IO的CAN通讯参数
    @param id 需要设置的ID
    @param port 端口
    @param arbitration_baudrate 需要设置的仲裁波特率
    @param data_baudrate 需要设置的数据波特率
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def set_terminal_io_485_param(socketFd: int, id: int, port: int, baudrate: int) -> Result:
    """
    @brief 设置末端IO的485通讯参数
    @param id 需要设置的ID
    @param port 端口
    @param baudrate 485波特率
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def terminal_io_read_modbus_data(socketFd: int, func_code: int, start_addr: int, read_addr_num: int, read_data: VectorInt) -> Tuple:
    """
    @brief 末端IO modbus 读数据
    @param func_code 功能码
    @param start_addr 起始地址
    @param read_addr_num 读取数量
    @param read_data 读到的数据（int 数组），范围 [0, 65535]（失败时输出向量被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def terminal_io_write_modbus_data(socketFd: int, func_code: int, start_addr: int, write_addr_num: int, read_data: VectorInt) -> Result:
    """
    @brief 末端IO modbus 写数据
    @param func_code 功能码
    @param start_addr 起始地址
    @param write_addr_num 写入数量
    @param read_data 写数据（int 数组），范围 [0, 65535]
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_get_master_connection_status(socketFd: int, id: int, status: int) -> Tuple:
    """
    @brief 获取主站连接状态
    @param id 配方id参数，最多保存9个id
    @param status 连接状态（输出）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def modbus_write_multiple_coil_status(socketFd: int, id: int, address: int, data: VectorInt) -> Result:
    """
    @brief 写多个线圈状态（功能码 0FH）
    @param id 配方id参数，最多保存9个id
    @param address 起始地址
    @param data 写入的线圈状态数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def read_hand_gesture_sequence(socketFd: int, id: int, start_address: int, address_length: int, data: str) -> Tuple:
    """
    @brief 获取运行灵巧手目标手势序列号
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 接收数据（失败时输出被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def read_hand_action_sequence(socketFd: int, id: int, start_address: int, address_length: int, data: str) -> Tuple:
    """
    @brief 获取运行灵巧手动作序列号
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 接收数据（失败时输出被清空）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_hand_freedom_angle(socketFd: int, id: int, start_address: int, address_length: int, data: VectorString) -> Result:
    """
    @brief 设置灵巧手各自由度角度
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 设置的数据（字符串数组）
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_hand_speed(socketFd: int, id: int, start_address: int, address_length: int, data: VectorInt) -> Result:
    """
    @brief 设置灵巧手速度
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 设置的数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_hand_force_threshold(socketFd: int, id: int, start_address: int, address_length: int, data: VectorInt) -> Result:
    """
    @brief 设置灵巧手力阈值
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 设置的数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_hand_angle_follow(socketFd: int, id: int, start_address: int, address_length: int, data: List[bool]) -> Result:
    """
    @brief 设置灵巧手角度跟随控制
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 设置的数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_hand_position_follow(socketFd: int, id: int, start_address: int, address_length: int, data: List[bool]) -> Result:
    """
    @brief 设置灵巧手位置跟随控制
    @param id 从站ID
    @param start_address 起始地址
    @param address_length 地址长度
    @param data 设置的数据
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_gripper_stroke(socketFd: int, id: int, data: VectorInt, addr_list: VectorInt, addr_length: int) -> Result:
    """
    @brief 设置夹爪行程
    @param id 从站ID
    @param data 设置的数据
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_gripper_release(socketFd: int, id: int, data: List[bool], addr_list: VectorInt, addr_length: int) -> Result:
    """
    @brief 松开夹爪
    @param id 从站ID
    @param data 设置的数据
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_gripper_force_control_grab(socketFd: int, id: int, data: VectorInt, addr_list: VectorInt, addr_length: int) -> Result:
    """
    @brief 夹爪力控夹取
    @param id 从站ID
    @param data 设置的数据（包含三个 int 值）
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_gripper_continuous_force_control_grab(socketFd: int, id: int, data: VectorInt, addr_list: VectorInt, addr_length: int) -> Result:
    """
    @brief 夹爪持续力控夹取
    @param id 从站ID
    @param data 设置的数据
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def write_gripper_move_to_position(socketFd: int, id: int, data: VectorInt, addr_list: VectorInt, addr_length: int) -> Result:
    """
    @brief 设置夹爪移动到指定位置
    @param id 从站ID
    @param data 设置的数据
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
def read_gripper_status(socketFd: int, id: int, data: VectorString, addr_list: VectorInt, addr_length: int) -> Tuple:
    """
    @brief 查询夹爪状态
    @param id 从站ID
    @param data 接收数据（hex 格式，失败时输出被清空）
    @param addr_list 要写入的寄存器地址数组
    @param addr_length 要写入的连续寄存器数量
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
# ---- 废弃接口（建议使用替代接口）----
def queue_motion_stop(socketFd: int) -> Result:
    """
    @brief 停止连续运动
    
    @deprecated 该接口已废弃，建议使用 `queue_motion_stop_not_power_off` 替代。
    @return 0=SUCCESS 成功；-1=RECEIVE_FAILED 接收失败；-2=DISCONNECT 未连接；-3=PARAM_ERR
    参数错误；-4=OPERATION_NOT_ALLOWED 操作不允许；-5=EXCEPTION 异常；-6=TIMEOUT 超时
    """
class AdmittanceController:
    def Configure(self, params: AdmittanceParams) -> Result: ...
    def SetDesiredWrench(self, desired_wrench: List[float]) -> Result: ...
    def Step(self, sensor_wrench: List[float], delta_pose: VectorDouble) -> Result: ...
    def Reset(self) -> Result: ...
    def GetRelativePose(self, relative_pose: VectorDouble) -> Result: ...
    def GetControlPeriod(self) -> float: ...
    def GetStiffness(self, axis: int) -> float: ...
    def CompensatePose(self, sensor_wrench: List[float], target_pose: List[float], out_pose: VectorDouble, out_delta: Any = None) -> Any: ...
class AdmittanceParams:
    mass: VectorDouble
    damping: VectorDouble
    stiffness: VectorDouble
    max_delta: VectorDouble
    control_period: float
class AlarmdIO: ...
class Alpha:
    alpha1: float
    alpha2: float
    alpha3: float
    alpha4: float
    alpha5: float
    alpha6: float
class CartesianParam:
    maxSpeed: float
    maxAcc: float
    maxDec: float
    maxAttitudeVel: float
    speedLimitMode: int
class CollisionPara:
    collisionDetection_run: VectorDouble
    collisionDetection_teach: VectorDouble
    position_delay_time_ms_value: float
    error_enable_time_ms_value: float
    axisum: int
class CollisionSafeParam:
    safe_coeff: VectorDouble
    servo_execution_delay: VectorDouble
class ConstForceStatus:
    running: bool
    sensor_wrench: VectorDouble
    filtered_wrench: VectorDouble
    target_pose: VectorDouble
    last_result: int
class ConstantForceParams:
    filter_alpha: VectorDouble
    deadband: VectorDouble
    force_control_axis: int
    sv_pct: float
    sa_pct: float
    sj_pct: float
    sensor_loss_timeout_ms: int
class Coord:
    JOINT: ClassVar[Coord]
    BASE: ClassVar[Coord]
    TOOL: ClassVar[Coord]
    USER: ClassVar[Coord]
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
class DragParam:
    def __init__(self) -> None: ...
    dragMode: int
    startThresholdF: float
    startThresholdM: float
    dragInPosMaxVel: float
    dragInPosMaxAngleVel: float
    dragSensorCoeff: VectorDouble
    dragChangeRate: VectorDouble
class DragTorqueParam: ...
class HanYu:
    PC: float
    SP: VectorDouble
    TL: VectorDouble
class IOtype:
    num: int
    type: VectorString
    io_port_sum: List[List[int]]
class LogLevel:
    DEBUG: ClassVar[LogLevel]
    INFO: ClassVar[LogLevel]
    WARNING: ClassVar[LogLevel]
    ERROR: ClassVar[LogLevel]
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
class ModbusMasterParameter:
    type: int
    startAddress: int
    TCP: ModbusTCPParameter
    RTU: ModbusRTUParameter
class ModbusRTUParameter:
    slaveId: int
    port: int
    baudrate: int
    checkBit: str
    dataBit: int
    stopBit: int
class ModbusTCPParameter:
    IP: str
    port: int
class MoveCmd:
    targetPosType: PosType
    targetPosValue: VectorDouble
    targetPosName: str
    coord: Union[int, Coord]
    velocity: float
    acc: float
    dec: float
    pl: int
class PayloadParamBySensor: ...
class PosType:
    data: ClassVar[PosType]
    PType: ClassVar[PosType]
    E_TYPE: ClassVar[PosType]
    RP_TYPE: ClassVar[PosType]
    AP_TYPE: ClassVar[PosType]
    GPType: ClassVar[PosType]
    GEType: ClassVar[PosType]
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
class RemoteControl:
    clearStashPort: int
    faultResetPort: int
    pausePort: int
    startPort: int
    stopPort: int
    clearStashValue: int
    faultResetValue: int
    pauseValue: int
    startValue: int
    stopValue: int
    program: VectorRemoteProgram
class RemoteProgram:
    port: int
    value: int
class RemoteProgramSetting: ...
class Result:
    SUCCESS: ClassVar[Result]
    RECEIVE_FAILED: ClassVar[Result]
    DISCONNECT: ClassVar[Result]
    PARAM_ERR: ClassVar[Result]
    OPERATION_NOT_ALLOWED: ClassVar[Result]
    EXCEPTION: ClassVar[Result]
    TIMEOUT: ClassVar[Result]
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
class RobotDHParam:
    L1: float
    L2: float
    L3: float
    L4: float
    L5: float
    L6: float
    L7: float
    L8: float
    L9: float
    L10: float
    L11: float
    L12: float
    L13: float
    L14: float
    L15: float
    L16: float
    L17: float
    L18: float
    L19: float
    L20: float
    Couple_Coe_1_2: float
    Couple_Coe_2_3: float
    Couple_Coe_3_2: float
    Couple_Coe_3_4: float
    Couple_Coe_4_5: float
    Couple_Coe_4_6: float
    Couple_Coe_5_6: float
    dynamicLimit_max: float
    dynamicLimit_min: float
    pitch: float
    sliding_lead_value: float
    uplift_lead_value: float
    spray_distance: float
    threeAxisDirection: float
    fiveAxisDirection: float
    twoAxisConversionRatio: float
    threeAxisConversionRatio: float
    amplificationRatio: float
    conversionratio_x: float
    conversionratio_y: float
    conversionratio_z: float
    conversionratio_J1: float
    conversionratio_J2: float
    conversionratio_J3: float
    upsideDown: int
    hanyu: HanYu
    alpha: Alpha
class RobotJointParam:
    reducRatio: float
    encoderResolution: int
    posSWLimit: float
    negSWLimit: float
    ratedRotSpeed: float
    ratedDeRotSpeed: float
    maxRotSpeed: float
    maxDeRotSpeed: float
    ratedVel: float
    deRatedVel: float
    maxAcc: float
    maxDecel: float
    direction: int
class RobotState:
    channel: int
    mode: int
    interval: int
    position: int
    posSum: int
    stop: bool
    ioState: bool
    dataildmotionpos: bool
    programRunStatus: bool
    servoStatus: bool
    operationMode: bool
    globalSpeed: bool
    syncPosition: bool
    ioPort: VectorString
    optional: VectorString
class RobotType:
    SIX_AXIS_SERIAL: ClassVar[RobotType]
    FOUR_AXIS_SCARA: ClassVar[RobotType]
    FOUR_AXIS_PALLETIZING: ClassVar[RobotType]
    FOUR_AXIS_SERIAL: ClassVar[RobotType]
    SINGLE_AXIS: ClassVar[RobotType]
    FIVE_AXIS_SERIAL: ClassVar[RobotType]
    SIX_AXIS_COLLABORATIVE: ClassVar[RobotType]
    TWO_AXIS_SCARA: ClassVar[RobotType]
    THREE_AXIS_SCARA: ClassVar[RobotType]
    THREE_AXIS_CARTESIAN: ClassVar[RobotType]
    THREE_AXIS_SPECIAL: ClassVar[RobotType]
    SEVEN_AXIS_SERIAL: ClassVar[RobotType]
    SCARA_SPECIAL: ClassVar[RobotType]
    FOUR_AXIS_PALLETIZING_LEAD: ClassVar[RobotType]
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
class SafeIO: ...
class Sensor6DData:
    sensorConnected: bool
    fxData: float
    fyData: float
    fzData: float
    mxData: float
    myData: float
    mzData: float
    fxDataSubBase: float
    fyDataSubBase: float
    fzDataSubBase: float
    mxDataSubBase: float
    myDataSubBase: float
    mzDataSubBase: float
    torqueConvertData: float
class SensorBaseParam:
    sensorMass: float
    sensorMassCenterX: float
    sensorMassCenterY: float
    sensorMassCenterZ: float
    saveZero: bool
class SixDimensionalForceCommunicationParams:
    sensorDragEnable: bool
    originDataInitialPara: int
    sensorCommunicationType: int
    startupAutoConnectSensor: bool
    YDirection: int
    ZDirection: int
    etherCat_mapNum: int
    modbus_rtu_slaveID: int
    modbus_rtu_port: int
    modbus_rtu_baudRate: int
    modbus_rtu_addressType: int
    modbus_rtu_firstAddress: int
    modbus_rtu_addressNum: int
    modbus_rtu_endian: int
    modbus_rtu_checkBit: str
    modbus_rtu_dataBit: int
    modbus_rtu_stopBit: int
    modbus_tcp_IP: str
    modbus_tcp_port: int
    modbus_tcp_addressType: int
    modbus_tcp_firstAddress: int
    modbus_tcp_addressNum: int
    modbus_tcp_endian: int
class ToolCoordinateRange:
    max_range_x: float
    min_range_x: float
    max_range_y: float
    min_range_y: float
    max_range_z: float
    min_range_z: float
    tool_hand_method: int
class ToolParam:
    X: float
    Y: float
    Z: float
    A: float
    B: float
    C: float
    payloadMass: float
    payloadInertia: float
    payloadMassCenter_X: float
    payloadMassCenter_Y: float
    payloadMassCenter_Z: float
class VectorAlarmdIO: ...
class VectorDouble:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, l: List[float]) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, i: int) -> float: ...
    def __setitem__(self, i: int, x: float) -> None: ...
    def __iter__(self) -> Iterator[float]: ...
    def append(self, x: float) -> None: ...
    def push_back(self, x: float) -> None: ...
class VectorInt:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, l: List[int]) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, i: int) -> int: ...
    def __setitem__(self, i: int, x: int) -> None: ...
    def __iter__(self) -> Iterator[int]: ...
    def append(self, x: int) -> None: ...
    def push_back(self, x: int) -> None: ...
class VectorRemoteProgram:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, l: List[RemoteProgram]) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, i: int) -> RemoteProgram: ...
    def __setitem__(self, i: int, x: RemoteProgram) -> None: ...
    def __iter__(self) -> Iterator[RemoteProgram]: ...
    def append(self, x: RemoteProgram) -> None: ...
    def push_back(self, x: RemoteProgram) -> None: ...
class VectorRemoteProgramSetting: ...
class VectorString:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, l: List[str]) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, i: int) -> str: ...
    def __setitem__(self, i: int, x: str) -> None: ...
    def __iter__(self) -> Iterator[str]: ...
    def append(self, x: str) -> None: ...
    def push_back(self, x: str) -> None: ...
class VectorVectorString:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, l: List[List[str]]) -> None: ...
    def __len__(self) -> int: ...
    def __getitem__(self, i: int) -> List[str]: ...
    def __setitem__(self, i: int, x: List[str]) -> None: ...
    def __iter__(self) -> Iterator[List[str]]: ...
    def append(self, x: List[str]) -> None: ...
    def push_back(self, x: List[str]) -> None: ...

# 常量/枚举值
BASE = ...  # type: ignore
DISCONNECT = ...  # type: ignore
EXCEPTION = ...  # type: ignore
FIVE_AXIS_SERIAL = ...  # type: ignore
FOUR_AXIS_PALLETIZING = ...  # type: ignore
FOUR_AXIS_PALLETIZING_LEAD = ...  # type: ignore
FOUR_AXIS_SCARA = ...  # type: ignore
FOUR_AXIS_SERIAL = ...  # type: ignore
JOINT = ...  # type: ignore
OPERATION_NOT_ALLOWED = ...  # type: ignore
PARAM_ERR = ...  # type: ignore
PosType_AP_TYPE = ...  # type: ignore
PosType_E_TYPE = ...  # type: ignore
PosType_GEType = ...  # type: ignore
PosType_GPType = ...  # type: ignore
PosType_PType = ...  # type: ignore
PosType_RP_TYPE = ...  # type: ignore
PosType_data = ...  # type: ignore
RECEIVE_FAILED = ...  # type: ignore
SCARA_SPECIAL = ...  # type: ignore
SEVEN_AXIS_SERIAL = ...  # type: ignore
SINGLE_AXIS = ...  # type: ignore
SIX_AXIS_COLLABORATIVE = ...  # type: ignore
SIX_AXIS_SERIAL = ...  # type: ignore
SUCCESS = ...  # type: ignore
THREE_AXIS_CARTESIAN = ...  # type: ignore
THREE_AXIS_SCARA = ...  # type: ignore
THREE_AXIS_SPECIAL = ...  # type: ignore
TIMEOUT = ...  # type: ignore
TOOL = ...  # type: ignore
TWO_AXIS_SCARA = ...  # type: ignore
USER = ...  # type: ignore
