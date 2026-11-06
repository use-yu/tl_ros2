/**
 * ex_six_axis_force.cpp
 * @brief 六维力传感器示例程序 —— 设置通讯参数、读取传感器数据
 * @attention
 *   - 确保控制器网络通信正常
 *   - 传感器需正确连接并配置
 *   - 默认控制器 IP 为 192.168.1.13，可根据实际环境修改
 * @note 运行步骤
 *       编译: cd build && cmake .. && make
 *       运行: ./ex_six_axis_force
 */

#include <iostream>
#include <string>
#include <iomanip>
#include "tl_api.h"

#ifdef _WIN32
#include <windows.h>
#endif
int main(int argc, char* argv[])
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // 解析命令行参数: ./ex_six_axis_force [-ip 192.168.1.13]，默认 192.168.1.13
    std::string ip = "192.168.1.13";
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ip = argv[i + 1];
            break;
        }
    }
    // 定义端口
    std::string port = "6001";

    // 获取 SDK 版本
    std::string version = tl::get_library_version();
    std::cout << "[信息] SDK 版本: " << version << std::endl;

    // ---- 连接控制器 ----
    std::cout << "[信息] 连接控制器..." << std::endl;
    tl::SOCKETFD socketFd = tl::connect_robot(ip, port);
    if (socketFd < 0)
    {
        std::cerr << "[ERROR] 连接控制器失败!" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket = " << socketFd << ")" << std::endl;

    tl::Result ret;

    // ============================================================
    // 1. 设置六维力传感器通讯参数
    // ============================================================
    std::cout << "\n========== 设置传感器通讯参数 ==========" << std::endl;

    {
        tl::SixDimensionalForceCommunicationParams commParams;
        commParams.sensorCommunicationType = 0;       // 0: EtherCAT
        commParams.startupAutoConnectSensor = true;   // 启动自动连接传感器
        commParams.sensorDragEnable = true;           // 传感器拖拽使能
        commParams.etherCat_mapNum = 1;               // EtherCAT 映射数量
        commParams.YDirection = -1;                   // Y 方向（-1 反向）
        commParams.ZDirection = -1;                   // Z 方向（-1 反向）

        ret = tl::set_six_dimensional_force_communication_params(socketFd, commParams);
        if (ret == tl::Result::SUCCESS)
        {
            std::cout << "  [成功] 设置六维力传感器通讯参数" << std::endl;
        }
        else
        {
            std::cerr << "  [失败] 设置六维力传感器通讯参数失败! ret = " << ret << std::endl;
        }
    }

    // ============================================================
    // 2. 获取六维力传感器数据
    // ============================================================
    std::cout << "\n========== 获取传感器数据 ==========" << std::endl;

    {
        tl::Sensor6DData sensorData;
        ret = tl::get_sensor_6d_data(socketFd, sensorData);
        if (ret == tl::Result::SUCCESS)
        {
            std::cout << "  [信息] 传感器连接状态: "
                      << (sensorData.sensorConnected ? "已连接" : "未连接") << std::endl;

            std::cout << std::fixed << std::setprecision(3);

            std::cout << "\n  --- 传感器原始数据 ---" << std::endl;
            std::cout << "  Fx = " << sensorData.fxData << " N" << std::endl;
            std::cout << "  Fy = " << sensorData.fyData << " N" << std::endl;
            std::cout << "  Fz = " << sensorData.fzData << " N" << std::endl;
            std::cout << "  Mx = " << sensorData.mxData << " N·m" << std::endl;
            std::cout << "  My = " << sensorData.myData << " N·m" << std::endl;
            std::cout << "  Mz = " << sensorData.mzData << " N·m" << std::endl;

            std::cout << "\n  --- 去皮后数据 ---" << std::endl;
            std::cout << "  Fx = " << sensorData.fxDataSubBase << " N" << std::endl;
            std::cout << "  Fy = " << sensorData.fyDataSubBase << " N" << std::endl;
            std::cout << "  Fz = " << sensorData.fzDataSubBase << " N" << std::endl;
            std::cout << "  Mx = " << sensorData.mxDataSubBase << " N·m" << std::endl;
            std::cout << "  My = " << sensorData.myDataSubBase << " N·m" << std::endl;
            std::cout << "  Mz = " << sensorData.mzDataSubBase << " N·m" << std::endl;

            if (!sensorData.torqueConvertData.empty())
            {
                std::cout << "\n  --- 扭矩转换数据 ---" << std::endl;
                for (size_t i = 0; i < sensorData.torqueConvertData.size(); ++i)
                {
                    std::cout << "  J" << (i + 1) << " = "
                              << sensorData.torqueConvertData[i] << std::endl;
                }
            }
        }
        else
        {
            std::cerr << "  [失败] 获取六维力传感器数据失败! ret = " << ret << std::endl;
        }
    }

    // ---- 断开连接 ----
    std::cout << "\n========== 断开连接 ==========" << std::endl;
    tl::disconnect_robot(socketFd);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 六维力传感器示例程序运行完毕" << std::endl;
    return 0;
}
