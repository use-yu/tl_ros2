/**
 * ex_omnipicker_claw.cpp
 * @brief OmniPicker 夹爪（Modbus RTU 从站）控制示例
 * @attention
 *   - 夹爪需先在上位机把 modbus_mode 置为 True（Modbus RTU）并保存、重新上电，
 *     拨码开关切到 RS485 或 UART；默认 CAN/CAN-FD 下不走 Modbus
 *   - 夹爪经串口接控制器，主站参数（串口号、波特率、从站 ID）请按实际接线修改
 *     下方"RTU 主站配置区"；从站地址复用夹爪 can_node_id
 *   - 位置/速度/力矩/加(减)速度取值均为 0x00~0xFF，位置 0x00=夹紧 0xFF=完全张开
 *   - 夹爪会真实动作：运行前请确保夹爪空闲、指间无物体
 * @note 运行步骤
 *       编译: cd examples/cpp && cmake --preset linux-native && cmake --build --preset linux-native
 *       运行: ./build/linux-native/ex_omnipicker_claw
 *       对照: ../py/ex_omnipicker_claw.py（Python 版示例）
 */
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "tl_api.h"

#ifdef _WIN32
#  include <windows.h>
#endif

// ============================================================
// 控制器连接配置（按实际修改）
// ============================================================
static std::string ROBOT_IP = "192.168.1.13"; // 控制器 IP（可用 -ip 参数覆盖）
const char *ROBOT_PORT = "6001";              // 控制器端口

// ============================================================
// RTU 主站配置区（请根据实际接线自行修改）
// ============================================================
constexpr int RTU_MASTER_ID = 1; // 控制器侧主站配方编号（1~9）
constexpr int RTU_SLAVE_ID = 1; // 夹爪从站地址，复用夹爪 can_node_id（上位机可查）
constexpr int RTU_PORT = 2;     // 控制器串口号（按接线修改）
constexpr int RTU_BAUDRATE = 115200;      // 波特率（固件 3.3.5 起可经上位机修改）
const std::string RTU_CHECK_BIT = "None"; // 校验位: "None"/"Even"/"Odd"
constexpr int RTU_DATA_BIT = 8;           // 数据位
constexpr int RTU_STOP_BIT = 1;           // 停止位

// ============================================================
// OmniPicker 保持寄存器（地址按手册编号直传，startAddress=true 表示 0 起始）
// 功能码: 03H 读保持寄存器 / 10H 写多个保持寄存器
// ============================================================
constexpr int REG_FIRMWARE_VERSION = 0; // 固件版本号（只读）
constexpr int REG_TARGET_POS = 10;      // 目标位置 0x00~0xFF（0x00=夹紧 0xFF=完全张开）
constexpr int REG_TARGET_SPEED = 11;    // 目标速度 0x00~0xFF（0xFF=最大速度）
constexpr int REG_TARGET_FORCE = 12;    // 目标力矩 0x00~0xFF（0xFF=最大力矩）
constexpr int REG_TARGET_ACC = 13;      // 目标加速度 0x00~0xFF（0xFF=最大加速度）
constexpr int REG_TARGET_DEC = 14;      // 目标减速度 0x00~0xFF（0xFF=最大减速度）
constexpr int REG_MOTION_TRIGGER = 15; // 运动触发：写非零后按 10~14 运动，开始运动后自动清零
constexpr int REG_FAULT_CODE = 20;    // 当前错误码（只读）
constexpr int REG_STATE = 21;         // 当前状态（只读）
constexpr int REG_CURRENT_POS = 22;   // 当前位置（只读）
constexpr int REG_CURRENT_SPEED = 23; // 当前速度（只读）
constexpr int REG_CURRENT_FORCE = 24; // 当前力矩（只读）

constexpr int POS_OPEN = 0xFF;  // 完全张开
constexpr int POS_HALF = 0x7F;  // 50% 行程
constexpr int POS_CLOSE = 0x00; // 夹紧
constexpr int POS_TOLERANCE = 3; // 到位判定的位置容差（约 1.4mm，行程 120mm 对应 0x00~0xFF）
constexpr int CMD_SPEED = 0xFF; // 运动指令速度
constexpr int CMD_FORCE = 0xFF; // 运动指令力矩
constexpr int CMD_ACC = 0xFF;   // 运动指令加速度
constexpr int CMD_DEC = 0xFF;   // 运动指令减速度

// 演示位置序列（按需修改）
const std::vector<int> DEMO_POSITIONS = {POS_OPEN, POS_HALF, POS_CLOSE};

// ============================================================
// 工具函数
// ============================================================

// @brief 0x00~0xFF 寄存器值的十六进制文本（如 "0x7F"）
std::string hex_byte(int value)
{
  constexpr char kDigits[] = "0123456789ABCDEF";
  std::string text = "0x";
  text += kDigits[(value >> 4) & 0x0F];
  text += kDigits[value & 0x0F];
  return text;
}

// @brief 打印操作结果
void print_result(const std::string& msg, tl::Result ret)
{
  if (ret == tl::Result::SUCCESS)
  {
    std::cout << "  [成功] " << msg << std::endl;
  }
  else
  {
    std::cout << "  [失败] " << msg << "，错误码: " << static_cast<int>(ret) << std::endl;
  }
}

// @brief 打印分隔标题
void print_separator(const std::string& title)
{
  std::cout << "\n========== " << title << " ==========" << std::endl;
}

// @brief 错误码含义
const char *fault_desc(int code)
{
  switch (code)
  {
    case 0:
      return "无故障";
    case 1:
      return "过温警报";
    case 2:
      return "超速警报";
    case 3:
      return "初始化故障警报";
    case 4:
      return "超限检测警报";
    default:
      return "未知";
  }
}

// @brief 状态码含义
const char *state_desc(int code)
{
  switch (code)
  {
    case 0:
      return "已到达目标位置";
    case 1:
      return "夹爪移动中";
    case 2:
      return "夹爪堵转";
    case 3:
      return "物体掉落";
    default:
      return "未知";
  }
}

// @brief 读保持寄存器（03H），失败返回 false
bool read_holding_registers(tl::SOCKETFD sock, int address, int quantity, std::vector<int>& data,
                            const std::string& desc)
{
  data.clear();
  auto ret = tl::modbus_read_holding_registers(sock, RTU_MASTER_ID, address, quantity, data);
  print_result(
      desc + "（addr=" + std::to_string(address) + ", qty=" + std::to_string(quantity) + "）", ret);
  return ret == tl::Result::SUCCESS && static_cast<int>(data.size()) >= quantity;
}

// @brief 读 20~24 地址并打印错误码/状态/位置/速度/力矩
bool print_status(tl::SOCKETFD sock)
{
  std::vector<int> data;
  if (!read_holding_registers(sock, REG_FAULT_CODE, 5, data, "读夹爪状态"))
  {
    std::cout << "  [提示] 状态读取失败，请检查接线与从站 ID (RTU_SLAVE_ID)" << std::endl;
    return false;
  }
  const int fault = data[0];
  const int state = data[1];
  std::cout << "  错误码: " << fault << "（" << fault_desc(fault) << "）" << std::endl;
  std::cout << "  状态: " << state << "（" << state_desc(state) << "）" << std::endl;
  std::cout << "  位置: 0x" << std::hex << (data[2] & 0xFF) << "  速度: 0x" << (data[3] & 0xFF)
            << "  力矩: 0x" << (data[4] & 0xFF) << std::dec << std::endl;
  return true;
}

// @brief 写 10~14 目标参数 + 15 触发（10H 写 6 个寄存器）
bool move_to(tl::SOCKETFD sock, int pos)
{
  const std::vector<int> cmd = {pos, CMD_SPEED, CMD_FORCE, CMD_ACC, CMD_DEC, 1};
  auto ret = tl::modbus_write_multiple_holding_registers(sock, RTU_MASTER_ID, REG_TARGET_POS, cmd);
  print_result("下发运动指令（pos=" + hex_byte(pos) + ", speed=" + hex_byte(CMD_SPEED) +
                   ", force=" + hex_byte(CMD_FORCE) + "）",
               ret);
  return ret == tl::Result::SUCCESS;
}

// @brief 轮询状态与当前位置，直到到位（状态 0 且位置进入容差）、堵转（2）/掉落（3）或超时
bool wait_motion_done(tl::SOCKETFD sock, int target_pos, int timeout_ms = 10000)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::vector<int> data;
    // 同时读 21（状态）与 22（位置）：刚下发指令时读到的可能仍是上一帧的「已到达目标位置」，
    // 只看状态会在夹爪尚未起步时误判到位
    auto ret = tl::modbus_read_holding_registers(sock, RTU_MASTER_ID, REG_STATE, 2, data);
    if (ret == tl::Result::SUCCESS && data.size() >= 2)
    {
      const int state = data[0];
      const int pos = data[1] & 0xFF;
      if (state == 0 && std::abs(pos - target_pos) <= POS_TOLERANCE)
      {
        return true;
      }
      if (state == 2 || state == 3)
      {
        std::cout << "  [警告] 运动异常: " << state_desc(state) << std::endl;
        return false;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "  [警告] 等待到位超时（" << (timeout_ms / 1000.0) << " s）" << std::endl;
  return false;
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char *argv[])
{
#ifdef _WIN32
  SetConsoleOutputCP(CP_UTF8);
#endif

  // 解析命令行参数: ./ex_omnipicker_claw [-ip 192.168.1.13]，默认 192.168.1.13
  for (int i = 1; i + 1 < argc; ++i)
  {
    if (std::string(argv[i]) == "-ip" || std::string(argv[i]) == "--ip")
    {
      ROBOT_IP = argv[i + 1];
      break;
    }
  }

  // ---- 获取 SDK 版本 ----
  std::cout << "[信息] SDK 版本: " << tl::get_library_version() << std::endl;

  // ---- 连接控制器 ----
  print_separator("连接控制器");
  tl::SOCKETFD sock = tl::connect_robot(ROBOT_IP, ROBOT_PORT);
  if (sock < 0)
  {
    std::cerr << "[ERROR] 连接控制器失败" << std::endl;
    return -1;
  }
  std::cout << "[信息] 连接成功 (socket = " << sock << ")" << std::endl;

  // ---- 配置 RTU 主站 ----
  print_separator("配置 RTU 主站");
  tl::ModbusMasterParameter param;
  param.type = "RTU";
  param.startAddress = true; // 寄存器地址按手册编号直传（0 起始）
  param.RTU.slaveId = RTU_SLAVE_ID;
  param.RTU.port = RTU_PORT;
  param.RTU.baudrate = RTU_BAUDRATE;
  param.RTU.checkBit = RTU_CHECK_BIT;
  param.RTU.dataBit = RTU_DATA_BIT;
  param.RTU.stopBit = RTU_STOP_BIT;

  auto ret = tl::modbus_set_master_parameter(sock, RTU_MASTER_ID, param);
  print_result("tl::modbus_set_master_parameter (id=" + std::to_string(RTU_MASTER_ID) + ", RTU)",
               ret);
  if (ret != tl::Result::SUCCESS)
  {
    tl::disconnect_robot(sock);
    return -1;
  }
  std::cout << "  从站 ID: " << param.RTU.slaveId << ", 串口: " << param.RTU.port
            << ", 波特率: " << param.RTU.baudrate << std::endl;
  std::cout << "  校验: " << param.RTU.checkBit << ", 数据位: " << param.RTU.dataBit
            << ", 停止位: " << param.RTU.stopBit << std::endl;

  // ---- 打开 Modbus 主站 ----
  ret = tl::modbus_open_master(sock, RTU_MASTER_ID);
  print_result("tl::modbus_open_master (id=" + std::to_string(RTU_MASTER_ID) + ")", ret);
  if (ret != tl::Result::SUCCESS)
  {
    tl::disconnect_robot(sock);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ---- 读固件版本与当前状态 ----
  print_separator("读取固件版本与状态");
  {
    std::vector<int> data;
    if (read_holding_registers(sock, REG_FIRMWARE_VERSION, 1, data, "读固件版本号"))
    {
      std::cout << "  固件版本号: " << data[0] << "（Modbus RTU 需固件 3.3.5 及以上）" << std::endl;
    }
  }
  print_status(sock);

  // ---- 逐位置下发运动指令并等待到位 ----
  for (size_t i = 0; i < DEMO_POSITIONS.size(); i++)
  {
    const int pos = DEMO_POSITIONS[i];
    print_separator("测试位置 " + std::to_string(i + 1) + "/" +
                    std::to_string(DEMO_POSITIONS.size()) + ": 目标位置=" + hex_byte(pos));
    if (!move_to(sock, pos))
    {
      continue;
    }
    if (wait_motion_done(sock, pos))
    {
      std::cout << "  [到位]" << std::endl;
    }
    print_status(sock);
  }

  // ---- 断开连接 ----
  print_separator("断开连接");
  tl::disconnect_robot(sock);
  std::cout << "[信息] 已断开连接" << std::endl;

  std::cout << "\n[信息] OmniPicker 夹爪示例运行完毕" << std::endl;
  return 0;
}
