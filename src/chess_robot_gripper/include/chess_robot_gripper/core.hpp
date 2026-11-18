// 夹爪驱动共用类型：RTU 协议、通信实现、设备反馈和动作完成判断。
#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace chess_robot_gripper {
using Clock = std::chrono::steady_clock;  // 用于耗时和样本新旧判断，不受系统校时影响。
using Bytes = std::vector<uint8_t>;

// 计算 RTU 校验值，不包含报文末尾已有的 CRC。
uint16_t crc16(const Bytes &data);
// 组装请求；value 在读请求中表示数量，在写请求中表示目标值。
Bytes request(uint8_t slave, uint8_t function, uint16_t address, uint16_t value);
// 校验对应回包，返回读出的值或写入回显值；校验失败时抛出异常。
uint16_t parse_reply(const Bytes &sent, const Bytes &reply);

// 底层收发接口；真实 TCP 与内存模拟实现使用相同的 RTU 字节格式。
class Transport {
public:
  virtual ~Transport() = default;
  // 完成一帧请求和应答；不负责判定设备动作是否完成。
  virtual Bytes exchange(const Bytes &request) = 0;
  // 释放通信连接，不表示夹爪已经停止。
  virtual void close() = 0;
};
// 创建 TCP 透传实现，timeout_ms 是单次请求的连接、发送和接收总时限。
std::unique_ptr<Transport> tcp_transport(const std::string &ip, int port, int timeout_ms);
// 创建模拟实现；contact_raw=-1 无物体，其余值表示模拟接触开度。
std::unique_ptr<Transport> mock_transport(int contact_raw = -1);

// 一轮顺序读取的结果；状态 255、开度 65535 表示未知。
struct Sample {
  // initialization：0 未初始化、1 完成、2 进行中。
  // status：0 移动、1 到位、2 夹住、3 掉落；position 使用原始单位 0～1000。
  uint16_t initialization = 255, status = 255, position = 65535;
  Clock::time_point sampled{};  // 本轮首条读取前的时刻，默认值表示尚未采样。
};

// 把设备操作转换成寄存器读写；调用者负责串行访问。
class Driver {
public:
  Driver(std::unique_ptr<Transport> transport, uint8_t slave);
  // 顺序读取状态与开度；初始化中跳过开度读取。
  Sample poll();
  // 发出初始化请求；完成情况需继续读取反馈。
  void initialize();
  // 先写力百分比，再写原始开度；不换算毫米、牛顿或模型关节角。
  void move(uint16_t position, uint8_t force);
private:
  uint16_t read(uint16_t address);
  void write(uint16_t address, uint16_t value);
  uint16_t transact(uint8_t function, uint16_t address, uint16_t value);
  std::unique_ptr<Transport> transport_;
  uint8_t slave_;
};

// 一个动作目标；是否允许发送，由节点参数和实时设备状态决定。
struct Command {
  uint8_t operation = 1;  // 0 初始化，1 开合。
  uint16_t position = 1000;  // 设备原始开度，不是毫米。
  uint8_t force = 20;  // 力设定百分比，不是实测力。
  bool require_contact = false;  // true 要求夹住物体，仅到位算空夹失败。
  double timeout = 5.;  // 写命令与后续等待的时限，单位 s。
};
// 返回空字符串表示有效，否则返回参数拒绝原因。
std::string validate(const Command &command, int max_force);

// 与 MoveGripper 的结果码对应；Pending 只供内部继续等待。
enum class Outcome : uint16_t {
  Pending = 65535, Initialized = 0, Reached = 1, Grasped = 2,
  NotInitialized = 10, EmptyGrasp = 11, UnexpectedContact = 12,
  Dropped = 13, Timeout = 14, CommunicationError = 15, Shutdown = 16
};

// 根据命令后的新样本判断完成；不读写设备，也不自行控制超时。
class Completion {
public:
  Completion(Command command, Sample before, int tolerance);
  // 相同结果连续出现两次才确认；夹持还要求状态或开度相对命令前有变化。
  Outcome observe(const Sample &sample);
private:
  Command command_;
  Sample before_;
  int tolerance_, stable_ = 0;  // 原始开度容差、连续相同判定次数。
  bool transition_ = false;  // 是否观察到命令前状态之外的新变化。
  Outcome previous_ = Outcome::Pending;
  Clock::time_point last_observed_;  // 防止重复样本再次计数。
};
}  // namespace chess_robot_gripper
