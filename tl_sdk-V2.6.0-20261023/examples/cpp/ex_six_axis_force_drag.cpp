/**
 * ex_six_axis_force_drag.cpp
 * @brief 六维力传感器拖动示教示例程序
 *
 * 通过六维力传感器实现拖动示教：
 *   1. 设置六维力传感器通讯参数（启用传感器拖拽 sensorDragEnable=true）
 *   2. 读取传感器数据确认传感器连接正常
 *   3. 设置拖拽模式（力矩模式），进入六维力拖动示教
 *   4. 拖动过程中实时读取六维力数据并检测拖拽结束
 *   5. 关闭拖拽、下电、断开连接
 *
 * @attention
 *   - 运行前需保证六维力传感器已正确接线并完成通讯配置
 *   - 拖动示教期间机械臂会随外力移动，请确保安全区域无人、无碰撞风险
 *   - 本示例会直接使能伺服上电，请谨慎运行
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_six_axis_force_drag
 *       可调参数（控制器 IP、拖动时长、拖拽模式）见文件开头常量区，按需求修改后重新编译即可
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include "tl_api.h"


#ifdef _WIN32
#include <windows.h>
#endif

// ============================================================
// ---- 可调参数（按需求修改后重新编译即可） ----
// ============================================================
static const std::string CONTROLLER_IP   = "192.168.1.13";  // 控制器 IP 地址
static const std::string CONTROLLER_PORT = "6001";          // 控制/状态端口（六维力接口使用 6001）
static const int         DRAG_SECONDS    = 30;               // 拖动示教时长（秒），到点自动结束
static const int         DRAG_MODE       = 2;                // 拖拽模式：0=无 1=3D鼠标 2=力矩模式 3=位置
                                                             // 六维力拖动示教使用力矩模式(2)

/**
 * @brief 使能伺服上电（两步流程）
 * @param socket_fd 控制连接 socket
 * @note set_servo_state(1) 必须先于 set_servo_poweron 调用
 */
void enable_servo(const int socket_fd)
{
    std::cout << "--- 检查伺服状态 ---" << std::endl;

    // 1. 先清除控制器错误和报警，再下电复位确保状态同步
    tl::clear_error(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::set_servo_poweroff(socket_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. 检查伺服状态
    int state = -1;
    auto ret = tl::get_servo_state(socket_fd, state);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 获取伺服状态失败: " << ret << std::endl;
        return;
    }
    std::cout << "  [信息] 当前伺服状态: " << state << " (0=停止 1=就绪 2=报警 3=运行)" << std::endl;

    if (state == 2) {
        std::cerr << "[ERROR] 伺服处于报警状态，无法上电，请排查报警原因后重试" << std::endl;
        return;
    }

    if (state == 3) {
        std::cout << "[INFO] 机械臂已在使能上电状态" << std::endl;
        return;
    }

    // 3. 上电流程
    if (state == 0) {
        std::cout << "  [信息] 伺服处于停止状态，设置就绪..." << std::endl;
        ret = tl::set_servo_state(socket_fd, 1);
        if (ret != tl::Result::SUCCESS && ret != tl::Result::OPERATION_NOT_ALLOWED) {
            std::cerr << "[ERROR] 设置伺服就绪失败: " << ret << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "  [信息] 上电使能..." << std::endl;
    ret = tl::set_servo_poweron(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 上电失败: " << ret << std::endl;
        return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 4. 确认结果
    tl::get_servo_state(socket_fd, state);
    if (state == 3) {
        std::cout << "[INFO] 机械臂使能上电成功（servo_state = " << state << "）" << std::endl;
    } else {
        std::cerr << "[ERROR] 机械臂使能上电失败（servo_state = " << state << "）" << std::endl;
    }
}

/**
 * @brief 使能伺服下电
 * @param socket_fd 控制连接 socket
 */
void disable_servo(const int socket_fd)
{
    int state = -1;
    tl::get_servo_state(socket_fd, state);
    switch (state)
    {
        case 1:
            std::cout << "[INFO] 机械臂已使能下电" << std::endl;
            break;
        case 3:
            tl::set_servo_poweroff(socket_fd);
            tl::get_servo_state(socket_fd, state);
            std::cout << "[INFO] 机械臂使能下电成功（servo_state = " << state << "）" << std::endl;
            return;
        default:
            break;
    }
    std::cout << "[INFO] 机械臂使能下电失败（servo_state = " << state << "）" << std::endl;
}

/**
 * @brief 打印六维力传感器通讯参数
 * @param tag 打印标签
 * @param p   六维力通讯参数结构体
 */
static void PrintSixDimensionalForceParam(const char* tag, const tl::SixDimensionalForceCommunicationParams& p)
{
    std::cout << "========== [ " << tag << " ] SixDimensionalForceCommunicationParams ==========" << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "sensorDragEnable" << " = " << (p.sensorDragEnable ? "true" : "false") << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "originDataInitialPara" << " = " << p.originDataInitialPara << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "sensorCommunicationType" << " = " << p.sensorCommunicationType << "  (0=EtherCAT 1=ModbusRTU 2=ModbusTCP)" << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "startupAutoConnectSensor" << " = " << (p.startupAutoConnectSensor ? "true" : "false") << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "YDirection" << " = " << p.YDirection << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "ZDirection" << " = " << p.ZDirection << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "etherCat_mapNum" << " = " << p.etherCat_mapNum << std::endl;
    std::cout << "  -- Modbus RTU --" << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_slaveID" << " = " << p.modbus_rtu_slaveID << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_port" << " = " << p.modbus_rtu_port << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_baudRate" << " = " << p.modbus_rtu_baudRate << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_addressType" << " = " << p.modbus_rtu_addressType << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_firstAddress" << " = " << p.modbus_rtu_firstAddress << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_addressNum" << " = " << p.modbus_rtu_addressNum << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_endian" << " = " << p.modbus_rtu_endian << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_checkBit" << " = " << p.modbus_rtu_checkBit << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_dataBit" << " = " << p.modbus_rtu_dataBit << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_rtu_stopBit" << " = " << p.modbus_rtu_stopBit << std::endl;
    std::cout << "  -- Modbus TCP --" << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_IP" << " = " << p.modbus_tcp_IP << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_port" << " = " << p.modbus_tcp_port << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_addressType" << " = " << p.modbus_tcp_addressType << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_firstAddress" << " = " << p.modbus_tcp_firstAddress << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_addressNum" << " = " << p.modbus_tcp_addressNum << std::endl;
    std::cout << "  " << std::left << std::setw(30) << "modbus_tcp_endian" << " = " << p.modbus_tcp_endian << std::endl;
    std::cout << "================================================================" << std::endl;
}

/**
 * @brief 打印六维力传感器数据
 * @param data 六维力传感器数据结构体
 */
static void PrintSensor6DData(const tl::Sensor6DData& data)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  [传感器状态] " << (data.sensorConnected ? "已连接" : "未连接") << std::endl;
    std::cout << "  Fx=" << std::setw(8) << data.fxDataSubBase
              << "  Fy=" << std::setw(8) << data.fyDataSubBase
              << "  Fz=" << std::setw(8) << data.fzDataSubBase
              << " (N)   ";
    std::cout << "Mx=" << std::setw(8) << data.mxDataSubBase
              << "  My=" << std::setw(8) << data.myDataSubBase
              << "  Mz=" << std::setw(8) << data.mzDataSubBase
              << " (N·m)" << std::endl;
}

/**
 * @brief 设置六维力传感器通讯参数（先查询原值，再基于原值修改关键字段后写回）
 * @param socket_fd 控制连接 socket
 * @return true 设置成功
 * @note 先 get 查询当前值，在查询结果基础上修改字段，再 set 写回，避免覆盖无关配置
 */
static bool setup_six_dimensional_force_comm(const int socket_fd)
{
    // ---------- Phase 1: 查询当前通讯参数 ----------
    tl::SixDimensionalForceCommunicationParams orig_param;
    tl::Result ret = tl::get_six_dimensional_force_communication_params(socket_fd, orig_param);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 查询六维力传感器通讯参数失败! ret = " << ret << std::endl;
        return false;
    }
    PrintSixDimensionalForceParam("get(查询原值)", orig_param);

    // ---------- Phase 2: 基于原值修改拖动示教相关字段 ----------
    tl::SixDimensionalForceCommunicationParams set_param = orig_param;
    set_param.sensorDragEnable = true;          // 启用传感器拖拽（六维力拖动示教关键开关）
    set_param.sensorCommunicationType = 0;      // 通讯类型 0=EtherCAT（按实际传感器类型修改）
    set_param.startupAutoConnectSensor = true;  // 启动自动连接传感器
    set_param.YDirection = -1;                  // Y 轴方向（1=正，-1=反）
    set_param.ZDirection = -1;                  // Z 轴方向（1=正，-1=反）
    // 其余字段（EtherCAT/Modbus 参数）保持 orig_param 原值，不覆盖
    PrintSixDimensionalForceParam("set(下发值)", set_param);

    // ---------- Phase 3: 写回 ----------
    ret = tl::set_six_dimensional_force_communication_params(socket_fd, set_param);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 设置六维力传感器通讯参数失败! ret = " << ret << std::endl;
        return false;
    }
    std::cout << "[INFO] 设置六维力传感器通讯参数成功（fire-and-forget，成败靠 get 读回验证）" << std::endl;

    // ---------- Phase 4: 读回校验 ----------
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    tl::SixDimensionalForceCommunicationParams get_param;
    ret = tl::get_six_dimensional_force_communication_params(socket_fd, get_param);
    if (ret == tl::Result::SUCCESS) {
        PrintSixDimensionalForceParam("get(读回值)", get_param);
    }
    return true;
}

int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---- 运行参数（取自文件开头常量区，按需求修改常量后重新编译）----
    // 解析命令行参数: ./ex_xxx [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = CONTROLLER_IP;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    const std::string port         = CONTROLLER_PORT;
    const int         drag_seconds = DRAG_SECONDS;
    const int         drag_mode    = DRAG_MODE;

    // 获取 SDK 版本
    std::string version = tl::get_library_version();
    std::cout << "[INFO] SDK 版本: " << version << std::endl;

    // ---- 连接控制器 ----
    std::cout << "[INFO] 连接控制器 " << ip << ":" << port << " ..." << std::endl;
    auto socket_fd = tl::connect_robot(ip, port);
    if (socket_fd < 0) {
        std::cerr << "[ERROR] 连接失败，程序退出!" << std::endl;
        return -1;
    }
    std::cout << "[INFO] 连接成功 (socket=" << socket_fd << ")" << std::endl;

    // ---- 使能伺服上电 ----
    enable_servo(socket_fd);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ============================================================
    // 1. 设置六维力传感器通讯参数（启用传感器拖拽）
    // ============================================================
    std::cout << "\n========== 设置六维力传感器通讯参数 ==========" << std::endl;
    if (!setup_six_dimensional_force_comm(socket_fd)) {
        std::cerr << "[ERROR] 通讯参数设置失败，程序退出!" << std::endl;
        goto cleanup;
    }

    // ============================================================
    // 2. 读取传感器数据，确认传感器连接正常
    // ============================================================
    std::cout << "\n========== 读取传感器数据 ==========" << std::endl;
    {
        tl::Sensor6DData sensorData;
        tl::Result ret = tl::get_sensor_6d_data(socket_fd, sensorData);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] 获取六维力传感器数据失败! ret = " << ret << std::endl;
            goto cleanup;
        }
        PrintSensor6DData(sensorData);
        if (!sensorData.sensorConnected) {
            std::cerr << "[WARN] 传感器未连接，拖动示教可能无法正常生效，请检查接线与配置" << std::endl;
        }
    }

    // ============================================================
    // 3. 设置拖拽模式（力矩模式 = 六维力拖动示教）
    // ============================================================
    std::cout << "\n========== 设置拖拽模式 ==========" << std::endl;
    std::cout << "[INFO] 拖拽模式 " << drag_mode
              << " (0=无 1=3D鼠标 2=力矩模式 3=位置)，六维力拖动示教使用力矩模式(2)" << std::endl;
    {
        tl::Result ret = tl::set_darg_mode(socket_fd, drag_mode);
        if (ret != tl::Result::SUCCESS) {
            std::cerr << "[ERROR] set_darg_mode(mode=" << drag_mode << ") 返回: " << ret << std::endl;
            goto cleanup;
        }
        std::cout << "[INFO] set_darg_mode(mode=" << drag_mode << ") 返回: " << ret
                  << "（fire-and-forget，成败靠实际拖动验证）" << std::endl;
    }

    // ============================================================
    // 4. 六维力拖动示教：实时读取六维力数据，检测拖拽结束
    // ============================================================
    std::cout << "\n========== 开始六维力拖动示教（" << drag_seconds << " 秒）==========" << std::endl;
    std::cout << "[INFO] 请握住末端工具，施加外力拖动机械臂到目标位置（安全第一）..." << std::endl;

    {
        auto start_time = std::chrono::steady_clock::now();
        bool drag_finished = false;
        int  tick = 0;

        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();

            // 达到设定时长，结束拖动
            if (elapsed >= drag_seconds) {
                std::cout << "\n[INFO] 拖动时长已到（" << drag_seconds << " 秒），结束拖动" << std::endl;
                break;
            }

            // 查询拖拽线程是否结束
            tl::Result ret = tl::get_drag_thread_is_end(socket_fd, drag_finished);
            if (ret != tl::Result::SUCCESS) {
                std::cerr << "[ERROR] 查询拖拽结束状态失败! ret = " << ret << std::endl;
                break;
            }
            if (drag_finished) {
                std::cout << "\n[INFO] 检测到拖拽已结束" << std::endl;
                break;
            }

            // 每 500ms 读取一次六维力数据
            tl::Sensor6DData sensorData;
            ret = tl::get_sensor_6d_data(socket_fd, sensorData);
            if (ret == tl::Result::SUCCESS) {
                std::cout << "  [剩余 " << (drag_seconds - elapsed) << "s] ";
                PrintSensor6DData(sensorData);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            tick++;
        }
    }

    // ============================================================
    // 5. 关闭拖拽模式
    // ============================================================
    std::cout << "\n========== 关闭拖拽模式 ==========" << std::endl;
    {
        tl::Result ret = tl::set_darg_mode(socket_fd, 0);   // 0 = 关闭拖拽
        std::cout << "[INFO] set_darg_mode(0) 返回: " << ret << std::endl;
    }

cleanup:
    // ---- 使能下电 ----
    disable_servo(socket_fd);

    // ---- 断开控制器连接 ----
    std::cout << "\n========== 断开连接 ==========" << std::endl;
    auto ret = tl::disconnect_robot(socket_fd);
    if (ret != tl::Result::SUCCESS) {
        std::cerr << "[ERROR] 断开连接失败（下电可能已自动断连，返回 -1 视为成功）" << std::endl;
    } else {
        std::cout << "[INFO] 断开连接成功" << std::endl;
    }

    std::cout << "\n[INFO] 六维力拖动示教示例程序运行完毕" << std::endl;
    return 0;
}
