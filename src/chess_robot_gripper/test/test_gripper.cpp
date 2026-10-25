#include "chess_robot_gripper/core.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <thread>
using namespace chess_robot_gripper;
namespace {
Bytes checked(Bytes b) { auto crc=crc16(b); b.push_back(uint8_t(crc)); b.push_back(uint8_t(crc >> 8)); return b; }
Sample sample(int init, int status, int pos, int tick=1) {
  return {uint16_t(init), uint16_t(status), uint16_t(pos), Clock::time_point{} + std::chrono::milliseconds(tick)};
}
class Broken : public Transport {
public:
  int writes=0, closes=0;
  Bytes exchange(const Bytes &) override { ++writes; throw std::runtime_error("lost write echo"); }
  void close() override { ++closes; }
};
}
TEST(Rtu, MatchesVendorInitializationExample) {
  EXPECT_EQ(request(1,6,0x100,1), (Bytes{1,6,1,0,0,1,0x49,0xf6}));
}
TEST(Rtu, ChecksWriteEcho) {
  auto q=request(1,6,0x103,500);
  EXPECT_EQ(parse_reply(q,q),500);
  EXPECT_THROW(parse_reply(q,request(1,6,0x103,499)),std::runtime_error);
}
TEST(Rtu, ValidatesReadValue) {
  EXPECT_EQ(parse_reply(request(1,3,0x202,1),checked({1,3,2,3,0xe8})),1000);
}
TEST(Rtu, RejectsWrongCrc) {
  auto b=checked({1,3,2,0,1}); b.back()^=1;
  EXPECT_THROW(parse_reply(request(1,3,0x200,1),b),std::runtime_error);
}
TEST(Rtu, RejectsWrongSlaveAndFunction) {
  auto q=request(1,3,0x200,1);
  EXPECT_THROW(parse_reply(q,checked({2,3,2,0,1})),std::runtime_error);
  EXPECT_THROW(parse_reply(q,checked({1,4,2,0,1})),std::runtime_error);
}
TEST(Rtu, RejectsExceptionsAndTruncation) {
  auto q=request(1,3,0x200,1);
  EXPECT_THROW(parse_reply(q,checked({1,0x83,2})),std::runtime_error);
  EXPECT_THROW(parse_reply(q,{1,3}),std::runtime_error);
  EXPECT_THROW(parse_reply(q,checked({1,3,4,0,1,0,2})),std::runtime_error);
}
TEST(Driver, DoesNotRetryUncertainWrite) {
  auto broken=std::make_unique<Broken>(); auto *observer=broken.get();
  Driver driver(std::move(broken),1);
  EXPECT_THROW(driver.move(0,20),std::runtime_error);
  EXPECT_EQ(observer->writes,1); EXPECT_EQ(observer->closes,1);
}
TEST(Driver, InvalidCommandsNeverWrite) {
  auto broken=std::make_unique<Broken>(); auto *observer=broken.get();
  Driver driver(std::move(broken),1);
  EXPECT_THROW(driver.move(1001,20),std::invalid_argument);
  EXPECT_THROW(driver.move(0,19),std::invalid_argument);
  EXPECT_EQ(observer->writes,0);
}
TEST(Config, RejectsInvalidValues) {
  Command c; EXPECT_TRUE(validate(c,30).empty());
  c.timeout=NAN; EXPECT_FALSE(validate(c,30).empty());
  c.timeout=0; EXPECT_FALSE(validate(c,30).empty());
  c.timeout=5; c.force=100; EXPECT_FALSE(validate(c,30).empty());
  c.force=20; c.position=1001; EXPECT_FALSE(validate(c,30).empty());
  c.position=0; c.operation=2; EXPECT_FALSE(validate(c,30).empty());
  c.operation=0; c.require_contact=true; EXPECT_FALSE(validate(c,30).empty());
}
TEST(Completion, RequiresTwoFreshMatchingPolls) {
  Completion c(Command{},sample(1,1,500),5);
  EXPECT_EQ(c.observe(sample(1,1,1000)),Outcome::Pending); // not newer
  EXPECT_EQ(c.observe(sample(1,1,1000,2)),Outcome::Pending);
  EXPECT_EQ(c.observe(sample(1,1,1000,2)),Outcome::Pending); // duplicate cannot confirm
  EXPECT_EQ(c.observe(sample(1,1,1000,3)),Outcome::Reached);
}
TEST(Completion, RejectsWrongPositionEvenWhenAtPositionFlagIsSet) {
  Completion c(Command{},sample(1,1,500),5);
  for(int i=2;i<10;++i) EXPECT_EQ(c.observe(sample(1,1,500,i)),Outcome::Pending);
}
TEST(Completion, EmptyClosureIsNotGraspSuccess) {
  Command goal; goal.position=0; goal.require_contact=true;
  Completion c(goal,sample(1,1,1000),5);
  EXPECT_EQ(c.observe(sample(1,1,0,2)),Outcome::Pending);
  EXPECT_EQ(c.observe(sample(1,1,0,3)),Outcome::EmptyGrasp);
}
TEST(Completion, ContactBeforeTargetCountsWhenRequested) {
  Command goal; goal.position=0; goal.require_contact=true;
  Completion c(goal,sample(1,1,1000),5);
  EXPECT_EQ(c.observe(sample(1,2,450,2)),Outcome::Pending);
  EXPECT_EQ(c.observe(sample(1,2,450,3)),Outcome::Grasped);
}
TEST(Completion, DoesNotReuseOldHeldFlag) {
  Command goal; goal.position=0; goal.require_contact=true;
  Completion c(goal,sample(1,2,450),5);
  for(int i=2;i<10;++i) EXPECT_EQ(c.observe(sample(1,2,450,i)),Outcome::Pending);
}
TEST(Completion, UnrequestedContactAndDropAreFailures) {
  Completion contact(Command{},sample(1,1,1000),5);
  contact.observe(sample(1,2,500,2));
  EXPECT_EQ(contact.observe(sample(1,2,500,3)),Outcome::UnexpectedContact);
  Completion dropped(Command{},sample(1,1,1000),5);
  dropped.observe(sample(1,3,500,2));
  EXPECT_EQ(dropped.observe(sample(1,3,500,3)),Outcome::Dropped);
}
TEST(Completion, InitializationAndLossAreExplicit) {
  Command goal; goal.operation=0;
  Completion init(goal,sample(0,1,1000),5);
  init.observe(sample(1,1,1000,2));
  EXPECT_EQ(init.observe(sample(1,1,1000,3)),Outcome::Initialized);
  Completion loss(Command{},sample(1,1,1000),5);
  loss.observe(sample(0,1,1000,2));
  EXPECT_EQ(loss.observe(sample(0,1,1000,3)),Outcome::NotInitialized);
}
TEST(Mock, PersistsInitializationAndConfirmsMeasuredMove) {
  Driver driver(mock_transport(),1);
  EXPECT_EQ(driver.poll().initialization,0);
  driver.initialize(); std::this_thread::sleep_for(std::chrono::milliseconds(220));
  EXPECT_EQ(driver.poll().initialization,1);
  driver.move(900,20); std::this_thread::sleep_for(std::chrono::milliseconds(120));
  auto s=driver.poll(); EXPECT_EQ(s.position,900); EXPECT_EQ(s.status,1);
}
TEST(Config, RejectsUnsupportedTransportConfig) {
  EXPECT_THROW(tcp_transport("",60000,150),std::invalid_argument);
  EXPECT_THROW(tcp_transport("127.0.0.1",0,150),std::invalid_argument);
  EXPECT_THROW(tcp_transport("127.0.0.1",60000,0),std::invalid_argument);
  EXPECT_THROW(mock_transport(1001),std::invalid_argument);
}
