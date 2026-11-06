/**
 * ex_inspire_claw.cpp
 * @brief 因时电爪（Inspire EG2-4C2，RS485 / Modbus RTU）控制测试示例
 * @attention
 *   - 确保控制器网络通信正常，夹爪通过 RS485 串口连接控制器
 *   - 主站参数（从站 ID、串口号、波特率等）请在下方"RTU 主站配置区"自行修改
 *   - 开度范围 0~1000：0=闭合完，1000=张开完；写目标开度后夹爪立即动作，
 *     需轮询实际开度寄存器等待到位
 *   - 夹爪会真实动作：默认测试序列首位置 0 为全闭合，运行前请确保夹爪空闲、
 *     指间无物体
 * @note 运行步骤
 *       编译: cd examples/cpp/build && cmake .. && make
 *       运行: ./ex_inspire_claw
 */
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tl_api.h"

#ifdef _WIN32
#include <windows.h>
#endif

// ============================================================
// 控制器连接配置（按实际修改）
// ============================================================
static std::string ROBOT_IP = "192.168.1.13";   // 机械臂控制器 IP（可用 -ip 参数覆盖）
const char* ROBOT_PORT = "6001";         // 控制器端口

// ============================================================
// RTU 主站配置区（请根据实际接线自行修改）
// ============================================================
constexpr int RTU_MASTER_ID = 1;         // 控制器侧主站编号
constexpr int RTU_SLAVE_ID = 1;          // 夹爪从站 ID
constexpr int RTU_PORT = 2;              // 控制器串口号（按接线修改）
constexpr int RTU_BAUDRATE = 115200;     // 波特率（EG2 默认 115200）
const std::string RTU_CHECK_BIT = "None"; // 校验位: "None"/"Even"/"Odd"
constexpr int RTU_DATA_BIT = 8;          // 数据位
constexpr int RTU_STOP_BIT = 1;          // 停止位

// ============================================================
// EG2-4C2 夹爪保持寄存器（控制器侧地址，EG2-4C2 固件实测）
// 注意: 通用手册（EG2-4XX V1.0.6）编号为 1010/1011/1012/1061/1064/1065，
//       本机 EG2-4C2 实测目标开度寄存器为地址 10（写 500 即张开到半开），
//       其余寄存器按同段顺延，若读数异常请按夹爪实际固件调整
// 功能码: 03H 读 / 06H 写单个 / 10H 写多个，CRC16-REV=0xA001
// ============================================================
constexpr int REG_OPENLEN_SET = 10;      // 目标开度 0~1000（0=闭合完 1000=张开完），写后立即动作
constexpr int REG_SPEED_SET = 11;        // 速度 10~1000，写后不动作
constexpr int REG_FORCE_SET = 12;        // 抓力 100~1000，写后不动作
constexpr int REG_OPENLEN_ACT = 61;      // 实际开度 0~1000（只读）
constexpr int REG_ERRORCODE = 64;        // 错误码（只读，位0堵转 位1过温 位2过流 位3运行故障 位4内部通信故障）
constexpr int REG_STATUS = 65;           // 状态码（只读，1张开到位 2闭合到位 3停止 4夹取中 5张开中 6夹取到位）
// 其它只读寄存器按需扩展: 60 实际抓力 / 62 电流 / 63 温度

constexpr int CLAW_SPEED = 300;          // 夹爪动作速度
constexpr int CLAW_FORCE = 300;          // 抓力阈值
// 测试开度序列: 0=闭合完, 1000=张开完；首位置 0 为全闭合，运行前确保指间无物体
const std::vector<int> CLAW_TEST_POS = {0, 1000};

// ============================================================
// 工具函数
// ============================================================

// @brief 打印操作结果
void print_result(const std::string& msg, tl::Result ret)
{
    if (ret == tl::Result::SUCCESS) {
        std::cout << "  [成功] " << msg << std::endl;
    } else {
        std::cout << "  [失败] " << msg << "，错误码: " << static_cast<int>(ret) << std::endl;
    }
}

// @brief 打印分隔标题
void print_separator(const std::string& title)
{
    std::cout << "\n========== " << title << " ==========" << std::endl;
}

// @brief 带重试的读取：最多尝试 3 次，每次间隔 500ms
tl::Result read_with_retry(tl::SOCKETFD sock, int id, int addr, int qty,
                       std::vector<int>& data, const std::string& desc)
{
    tl::Result ret = tl::Result::SUCCESS;
    for (int attempt = 1; attempt <= 3; attempt++) {
        data.clear();
        ret = tl::modbus_read_holding_registers(sock, id, addr, qty, data);
        if (ret == tl::Result::SUCCESS) {
            std::cout << "  [成功] " << desc << "（第" << attempt << "次）" << std::endl;
            return ret;
        }
        std::cout << "  [重试] " << desc << " 第" << attempt << "次失败，500ms后重试..."
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "  [失败] " << desc << "，错误码: " << static_cast<int>(ret)
              << "（重试3次均失败）" << std::endl;
    return ret;
}

// @brief 轮询实际开度寄存器（REG_OPENLEN_ACT）直到接近目标或超时
// @param last 输出: 最后一次成功读到的实际开度（无成功读数时为 -1）
// @return true 到位 / false 超时
bool wait_openlen_reach(tl::SOCKETFD sock, int master_id, int target, int& last,
                        int timeout_ms = 10000)
{
    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms);
    last = -1;
    bool read_ok = false;
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<int> data;
        auto ret = tl::modbus_read_holding_registers(sock, master_id, REG_OPENLEN_ACT, 1, data);
        if (ret == tl::Result::SUCCESS && !data.empty()) {
            read_ok = true;
            last = data[0];
            std::cout << "  当前开度: " << last << "（目标 " << target << "）\r" << std::flush;
            if (std::abs(last - target) <= 5) {
                std::cout << std::endl;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << std::endl;
    if (!read_ok) last = -1;
    return false;
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    // 解析命令行参数: ./ex_xxx [-ip 192.168.1.13]，默认 192.168.1.13
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip") {
            ROBOT_IP = argv[i + 1];
            break;
        }
    }

    // ---- 获取 SDK 版本 ----
    std::string version = tl::get_library_version();
    std::cout << "[信息] SDK 版本: " << version << std::endl;

    // ---- 连接控制器 ----
    print_separator("连接控制器");
    tl::SOCKETFD sock = tl::connect_robot(ROBOT_IP, ROBOT_PORT);
    if (sock < 0) {
        std::cerr << "[ERROR] 连接控制器失败" << std::endl;
        return -1;
    }
    std::cout << "[信息] 连接成功 (socket = " << sock << ")" << std::endl;

    tl::Result ret;

    // ---- 配置 RTU 主站 ----
    print_separator("配置 RTU 主站");
    {
        tl::ModbusMasterParameter param;
        param.type = "RTU";
        param.startAddress = true;              // 起始地址 0
        param.RTU.slaveId = RTU_SLAVE_ID;
        param.RTU.port = RTU_PORT;
        param.RTU.baudrate = RTU_BAUDRATE;
        param.RTU.checkBit = RTU_CHECK_BIT;
        param.RTU.dataBit = RTU_DATA_BIT;
        param.RTU.stopBit = RTU_STOP_BIT;

        ret = tl::modbus_set_master_parameter(sock, RTU_MASTER_ID, param);
        print_result("tl::modbus_set_master_parameter(id=1, RTU)", ret);
        if (ret != tl::Result::SUCCESS) {
            tl::disconnect_robot(sock);
            return -1;
        }
        std::cout << "  从站ID: " << param.RTU.slaveId
                  << ", 串口: " << param.RTU.port
                  << ", 波特率: " << param.RTU.baudrate << std::endl;
        std::cout << "  校验: " << param.RTU.checkBit
                  << ", 数据位: " << param.RTU.dataBit
                  << ", 停止位: " << param.RTU.stopBit << std::endl;
    }

    // ---- 打开 Modbus 主站 ----
    ret = tl::modbus_open_master(sock, RTU_MASTER_ID);
    print_result("tl::modbus_open_master(id=1)", ret);
    if (ret != tl::Result::SUCCESS) {
        tl::disconnect_robot(sock);
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- 03H 读当前开度 ----
    print_separator("读取夹爪当前开度");
    {
        std::vector<int> data;
        ret = read_with_retry(sock, RTU_MASTER_ID, REG_OPENLEN_ACT, 1, data, "读实际开度");
        if (ret != tl::Result::SUCCESS) {
            std::cout << "  [提示] 读取实际开度失败（地址 " << REG_OPENLEN_ACT
                      << " 为同段顺延假设，若持续失败请调整 REG_OPENLEN_ACT）" << std::endl;
        } else {
            std::cout << "  当前开度: " << data[0] << "（0=闭合完, 1000=张开完）" << std::endl;
        }
    }

    // ---- 06H 设置速度与抓力（地址 11/12 为同段顺延，失败不阻塞动作） ----
    print_separator("设置速度与抓力");
    ret = tl::modbus_write_signal_holding_registers(sock, RTU_MASTER_ID, REG_SPEED_SET, CLAW_SPEED);
    print_result("写速度 (addr=" + std::to_string(REG_SPEED_SET) + ", val="
                 + std::to_string(CLAW_SPEED) + ")", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cout << "  [提示] 速度寄存器可能不在此地址，跳过（不影响开度动作）" << std::endl;
    }
    ret = tl::modbus_write_signal_holding_registers(sock, RTU_MASTER_ID, REG_FORCE_SET, CLAW_FORCE);
    print_result("写抓力 (addr=" + std::to_string(REG_FORCE_SET) + ", val="
                 + std::to_string(CLAW_FORCE) + ")", ret);
    if (ret != tl::Result::SUCCESS) {
        std::cout << "  [提示] 抓力寄存器可能不在此地址，跳过（不影响开度动作）" << std::endl;
    }

    // ---- 06H 写目标开度（addr=10 为实测地址），并轮询实际开度到位 ----
    for (size_t i = 0; i < CLAW_TEST_POS.size(); i++) {
        int target = CLAW_TEST_POS[i];
        std::string title = "测试位置 " + std::to_string(i + 1) + "/"
                          + std::to_string(CLAW_TEST_POS.size())
                          + ": 目标开度=" + std::to_string(target);
        print_separator(title);

        ret = tl::modbus_write_signal_holding_registers(sock, RTU_MASTER_ID, REG_OPENLEN_SET, target);
        print_result("tl::modbus_write_signal_holding_registers(addr="
                     + std::to_string(REG_OPENLEN_SET) + ", val=" + std::to_string(target) + ")", ret);
        if (ret != tl::Result::SUCCESS) {
            tl::disconnect_robot(sock);
            return -1;
        }

        int last = -1;
        bool reached = wait_openlen_reach(sock, RTU_MASTER_ID, target, last);
        if (!reached) {
            std::cout << "  [提示] 未能确认到位（实际开度地址 " << REG_OPENLEN_ACT
                      << " 为同段顺延假设，若读数异常请调整 REG_OPENLEN_ACT；开度命令已下发）"
                      << std::endl;
        } else {
            std::cout << "  [到位] 开度 " << last << " 到达目标 " << target << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    // ---- 03H 读状态码与错误码 ----
    print_separator("读取夹爪状态");
    {
        std::vector<int> data;
        ret = read_with_retry(sock, RTU_MASTER_ID, REG_STATUS, 1, data, "读状态码");
        if (ret == tl::Result::SUCCESS) {
            int status = data[0];
            const char* desc = "未知";
            switch (status) {
                case 1: desc = "张开到位"; break;
                case 2: desc = "闭合到位"; break;
                case 3: desc = "停止"; break;
                case 4: desc = "夹取中"; break;
                case 5: desc = "张开中"; break;
                case 6: desc = "夹取到位"; break;
                default: break;
            }
            std::cout << "  状态码: " << status << "（" << desc << "）" << std::endl;
        }
    }
    {
        std::vector<int> data;
        ret = read_with_retry(sock, RTU_MASTER_ID, REG_ERRORCODE, 1, data, "读错误码");
        if (ret != tl::Result::SUCCESS) {
            std::cout << "  [提示] 读取错误码失败（地址 " << REG_ERRORCODE
                      << " 为同段顺延假设）" << std::endl;
            tl::disconnect_robot(sock);
            return 0;
        }
        int err = data[0];
        if (err == 0) {
            std::cout << "  错误码: 0（无错误）" << std::endl;
        } else {
            static const std::vector<std::pair<int, const char*>> bits = {
                {0, "堵转"}, {1, "过温"}, {2, "过流"}, {3, "运行故障"}, {4, "内部通信故障"}};
            std::string faults;
            for (const auto& b : bits) {
                if (err & (1 << b.first)) {
                    if (!faults.empty()) faults += ", ";
                    faults += b.second;
                }
            }
            std::cout << "  错误码: " << err << "（" << faults << "）" << std::endl;
        }
    }

    // ---- 断开连接 ----
    print_separator("断开连接");
    tl::disconnect_robot(sock);
    std::cout << "[信息] 已断开连接" << std::endl;

    std::cout << "\n[信息] 因时夹爪测试示例运行完毕" << std::endl;
    return 0;
}
