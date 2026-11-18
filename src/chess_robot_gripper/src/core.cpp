// AG95 协议与动作判断：组装 RTU 报文、读写寄存器、根据新反馈确认结果。

#include "chess_robot_gripper/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace chess_robot_gripper {

// 计算 Modbus RTU 的 CRC16；报文中按低字节在前附加。
uint16_t crc16(const Bytes &data) {
  uint16_t crc = 0xffff;
  for (auto byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
      // 最低位为 1 时，右移后再与 CRC 多项式异或。
      crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : crc >> 1;
  }
  return crc;
}

// 组装单寄存器请求；读时 value 是寄存器数量，写时是目标值。
Bytes request(uint8_t slave, uint8_t function, uint16_t address, uint16_t value) {
  // 地址和数值高字节在前；末尾 CRC 的字节顺序相反。
  Bytes bytes{slave, function, uint8_t(address >> 8), uint8_t(address),
    uint8_t(value >> 8), uint8_t(value)};
  auto crc = crc16(bytes);
  bytes.push_back(uint8_t(crc));
  bytes.push_back(uint8_t(crc >> 8));
  return bytes;
}

// 校验回包是否对应本次请求；读取返回寄存器值，写入返回已回显的目标值。
uint16_t parse_reply(const Bytes &sent, const Bytes &reply) {
  // 请求不是固定 8 字节，或回包连最短异常帧都不够长时，拒绝解析。
  if (sent.size() != 8 || reply.size() < 5)
    throw std::runtime_error("short RTU frame");
  Bytes body(reply.begin(), reply.end() - 2);
  // 内容计算出的 CRC 与回包末尾的 CRC 不一致，说明报文无效。
  if (crc16(body) != uint16_t(reply[reply.size()-2] | (reply.back() << 8)))
    throw std::runtime_error("RTU CRC mismatch");
  // 回包从站地址不是本次请求的设备时，拒绝使用。
  if (reply[0] != sent[0])
    throw std::runtime_error("wrong RTU slave");
  // 功能码最高位置 1，表示设备返回 Modbus 异常码。
  if (reply[1] == (sent[1] | 0x80))
    throw std::runtime_error("Modbus exception " + std::to_string(reply[2]));
  // 其余回包必须与请求使用相同功能码。
  if (reply[1] != sent[1])
    throw std::runtime_error("wrong RTU function");
  // 功能码 6：写单寄存器，设备应完整回显原请求。
  if (sent[1] == 6) {
    // 回显的地址、数值等任一字节不同，就不能确认本次写入。
    if (reply != sent)
      throw std::runtime_error("write echo mismatch");
    return uint16_t(sent[4] << 8 | sent[5]);
  }
  // 读取只支持功能码 3、一个 16 位寄存器，对应 7 字节回包。
  if (sent[1] != 3 || reply.size() != 7 || reply[2] != 2)
    throw std::runtime_error("invalid read frame length");
  return uint16_t(reply[3] << 8 | reply[4]);
}

// 接管通信对象并保存从站地址；构造时不读写设备。
Driver::Driver(std::unique_ptr<Transport> transport, uint8_t slave)
: transport_(std::move(transport)), slave_(slave) {
  // 通信对象缺失，或从站地址不在 1～247 时，拒绝创建驱动。
  if (!transport_ || slave == 0 || slave > 247)
    throw std::invalid_argument("slave must be 1..247");
}

// 完成一次请求与应答；出错关闭连接，向上抛出原因，不重发本次命令。
uint16_t Driver::transact(uint8_t function, uint16_t address, uint16_t value) {
  try {
    auto sent = request(slave_, function, address, value);
    return parse_reply(sent, transport_->exchange(sent));
  } catch (...) {
    transport_->close();
    throw;
  }
}

// 用功能码 3 读取一个寄存器。
uint16_t Driver::read(uint16_t address) {
  return transact(3, address, 1);
}

// 用功能码 6 写入一个寄存器，并检查设备回显。
void Driver::write(uint16_t address, uint16_t value) {
  transact(6, address, value);
}

// 依次读取初始化状态、运动/夹持状态和开度；这些值不是同一时刻的快照。
Sample Driver::poll() {
  Sample sample;
  sample.sampled = Clock::now();  // 从首条读取前计时，包含整轮读取耗时。
  sample.initialization = read(0x0200);
  sample.status = read(0x0201);
  // 初始化中（2）开度参考尚未建立，跳过开度读取，保留未知值 65535。
  // 此阶段设备曾返回超过 1000 的暂态值，不能当作有效开度。
  if (sample.initialization != 2)
    sample.position = read(0x0202);
  // 状态超出定义范围，或非初始化阶段开度超过 1000 时，本轮读取失败。
  if (sample.initialization > 2 || sample.status > 3 ||
      (sample.initialization != 2 && sample.position > 1000))
    throw std::runtime_error("out-of-range AG95 feedback: initialization=" +
      std::to_string(sample.initialization) + " status=" + std::to_string(sample.status) +
      " position=" + std::to_string(sample.position));
  return sample;
}

// 向初始化寄存器写 1；写回显不代表初始化运动已经完成。
void Driver::initialize() {
  write(0x0100, 1);
}

// 先设力百分比，再设原始开度；任一步失败就向上报错，不自动补写或重试。
void Driver::move(uint16_t position, uint8_t force) {
  // 开度超过 1000，或力设定不在 20～100% 时，禁止写入。
  if (position > 1000 || force < 20 || force > 100)
    throw std::invalid_argument("invalid raw position/force");
  write(0x0101, force);
  write(0x0103, position);
}

// 检查请求参数与配置的力上限；返回空字符串表示有效，否则返回拒绝原因。
std::string validate(const Command &c, int max_force) {
  // 仅接受初始化（0）和开合（1）。
  if (c.operation > 1)
    return "operation must be INITIALIZE(0) or MOVE(1)";
  // 超时必须是有限数值，并在 0.5～60 s 范围内。
  if (!std::isfinite(c.timeout) || c.timeout < .5 || c.timeout > 60.)
    return "timeout_sec must be finite, 0.5..60";
  // 初始化不使用目标开度和力设定，也不能要求夹住物体。
  if (c.operation == 0) {
    // 初始化请求附带“必须夹住物体”时，拒绝该请求。
    if (c.require_contact)
      return "initialization cannot require contact";
    return {};
  }
  // 开合目标只能使用设备的 0～1000 原始开度。
  if (c.position > 1000)
    return "position_raw must be 0..1000";
  // 力设定须在 20% 到配置上限之间，不能越过本节点设置的上限。
  if (c.force < 20 || c.force > max_force)
    return "force_percent outside configured 20..max_force_percent";
  return {};
}

// 保存目标、命令前反馈与容差，用于判断后续样本是否属于本次动作。
Completion::Completion(Command command, Sample before, int tolerance)
: command_(command), before_(before), tolerance_(tolerance), last_observed_(before.sampled) {}

// 检查一个新样本；连续两次得到相同判定后才返回确定结果。
Outcome Completion::observe(const Sample &s) {
  // 时间没有前进的旧样本不能再次计数。
  if (s.sampled <= last_observed_)
    return Outcome::Pending;
  last_observed_ = s.sampled;
  // 初始化状态、夹持状态或开度任一项改变，就记下本次命令后出现过变化。
  if (s.initialization != before_.initialization || s.status != before_.status ||
      s.position != before_.position)
    transition_ = true;
  Outcome result = Outcome::Pending;
  // 真正发出初始化命令后，要求已初始化、到位且开度有效。
  // 原本已初始化的请求由节点直接读确认，不进入这里。
  if (command_.operation == 0) {
    if (s.initialization == 1 && s.status == 1 && s.position <= 1000)
      result = Outcome::Initialized;
  // 开合过程中变为未初始化（0），判为失败。
  } else if (s.initialization == 0)
    result = Outcome::NotInitialized;
  // 只有已初始化（1）时，才判断开合结果；初始化中继续等待。
  else if (s.initialization == 1) {
    // 状态 3 表示设备报告物体掉落。
    if (s.status == 3)
      result = Outcome::Dropped;
    // 状态 2 表示夹住物体，且必须出现过新变化，不能沿用命令前的旧夹持状态。
    else if (s.status == 2 && transition_) {
      // 本次要求夹持则成功，否则属于非预期接触。
      result = command_.require_contact ? Outcome::Grasped : Outcome::UnexpectedContact;
    // 状态 1 表示到位，且实际开度必须落在目标容差内。
    } else if (s.status == 1 && std::abs(int(s.position) - int(command_.position)) <= tolerance_) {
      // 要求接触却只到位属于空夹；不要求接触时则是正常到位。
      result = command_.require_contact ? Outcome::EmptyGrasp : Outcome::Reached;
    }
  }
  // 连续判定相同则累加，否则从 1 重新计数；不要求两个原始读数完全相等。
  stable_ = result == previous_ ? stable_ + 1 : 1;
  previous_ = result;
  return stable_ >= 2 ? result : Outcome::Pending;
}
}  // namespace chess_robot_gripper
