#include "chess_robot_arm/checkpoint.hpp"
#include "chess_robot_arm/rotating_audit.hpp"
#include "chess_robot_arm/driver.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <sys/wait.h>

using namespace chess_robot_arm;
namespace {
class RecoveryReply:public CommandTransport {
  std::vector<std::string> &calls_;
  std::function<void()> alarm_,enable_;
public:
  RecoveryReply(std::vector<std::string> &calls,std::function<void()> alarm,std::function<void()> enable={})
      :calls_(calls),alarm_(std::move(alarm)),enable_(std::move(enable)){}
  std::string query(const std::string &command) override {
    calls_.push_back(command);
    if(command=="GetErrorID()") {if(alarm_)alarm_();return "0,{[]},GetErrorID();";}
    if(command=="EnableRobot()") {if(enable_)enable_();return "0,{},EnableRobot();";}
    throw std::runtime_error("unexpected fixture command");
  }
};
struct CheckpointTest:testing::Test {
  std::filesystem::path root;
  Config c;
  std::shared_ptr<const chess_robot_twist::Model> model;
  uint64_t stamp_counter=100;
  void SetUp() override {
    char path[]="/tmp/chess-checkpoint-test-XXXXXX";auto p=::mkdtemp(path);
  ASSERT_NE(p,nullptr);root=p;
    c.urdf=ARM_TEST_URDF;c.checkpoint_file=(root/"state.save").string();
    c.checkpoint_dwell=.002;c.dwell=.001;c.freshness=.2;c.stationary_speed=.002;
    c.flange_tcp.translation()=Eigen::Vector3d(0,0,.2);
    model=std::make_shared<chess_robot_twist::Model>(c.urdf,"base_link","Link6",c.flange_tcp,c.home);
  }
  void TearDown() override {std::filesystem::remove_all(root);}
  Snapshot saved(double wrist=.4) {
    chess_robot_twist::Tracker t(model,c.tracker_options);t.initializeAtHome(c.home,c.home_tolerance);
    for(int i=1;i<=120;++i) {auto q=c.home;q[5]+=wrist*i/120;t.advance(q);}
    Snapshot s(t);s.fresh=s.reference=true;s.feedback.q=t.lastJoints();s.feedback.mode=5;s.feedback.controller_stamp=55;
    s.feedback.received=Clock::now();return s;
  }
  void stopped(State &state,const Joints &q,uint64_t mode=4) {
    Feedback f;f.q=q;f.mode=mode;f.controller_stamp=++stamp_counter;f.received=Clock::now()-std::chrono::milliseconds(10);state.update(f);
    f.controller_stamp=++stamp_counter;f.received=Clock::now();state.update(f);
  }
  void enableFixture() {
    c.backend="hardware";c.test_fixture=true;c.motion_paused=false;
    c.enable=c.tcp_verified=c.limits_verified=c.execution_verified=c.servo_certified=true;
    c.checkpoint_enable_after_restore=c.checkpoint_no_unobserved_motion=true;
  }
};
}
TEST_F(CheckpointTest, PreservesFullWindingBeyondPrincipalAngle) {
  auto s=saved(330*pi/180);auto cp=s.tracker.checkpoint();
  chess_robot_twist::Tracker t(model,c.tracker_options);t.restoreCheckpoint(cp);
  EXPECT_DOUBLE_EQ(t.metrics().total,s.tracker.metrics().total);
  EXPECT_EQ(t.metrics().local,s.tracker.metrics().local);
  EXPECT_GT(t.metrics().total,pi);
}
TEST_F(CheckpointTest, RejectsCorruptGeometryWithoutReplacingExistingTracker) {
  auto s=saved();auto before=s.tracker.metrics();auto cp=s.tracker.checkpoint();
  cp.previous_phase[0]+=.1;
  EXPECT_THROW(s.tracker.restoreCheckpoint(cp),chess_robot_twist::Error);
  EXPECT_EQ(s.tracker.metrics().local,before.local);
  cp=s.tracker.checkpoint();cp.local[0]+=.2;
  EXPECT_THROW(s.tracker.restoreCheckpoint(cp),chess_robot_twist::Error);
  cp=s.tracker.checkpoint();cp.joints[0]=NAN;
  EXPECT_THROW(s.tracker.restoreCheckpoint(cp),chess_robot_twist::Error);
}
TEST_F(CheckpointTest, DurableRoundTripAndDirtyMarkerNoFallback) {
  CheckpointStore store(c);auto s=saved();store.save(s);auto cp=store.load();
  EXPECT_EQ(cp.local,s.tracker.metrics().local);store.markMotionPending();
  EXPECT_THROW(store.load(),Failure);
  store.save(s);
  EXPECT_NO_THROW(store.load());
}
TEST_F(CheckpointTest, ChecksumDetectsTruncationAndTampering) {
  CheckpointStore store(c);store.save(saved());
  {std::ofstream f(c.checkpoint_file,std::ios::app);f<<"corruption";}
  EXPECT_THROW(store.load(),Failure);
  store.save(saved());std::filesystem::resize_file(c.checkpoint_file,70);
  EXPECT_THROW(store.load(),Failure);
}
TEST_F(CheckpointTest, ModelControllerAndToolChangesRefuseOldSave) {
  {CheckpointStore store(c);store.save(saved());}
  auto changed=c;changed.flange_tcp.translation().z()+=.001;
  {CheckpointStore store(changed);
  EXPECT_THROW(store.load(),Failure);}
  changed=c;changed.controller_firmware="different";
  {CheckpointStore store(changed);
  EXPECT_THROW(store.load(),Failure);}
  changed=c;changed.home[0]+=.001;
  {CheckpointStore store(changed);
  EXPECT_THROW(store.load(),Failure);}
}
TEST_F(CheckpointTest, SingleWriterAndMissingFileDoNotCreateReference) {
  CheckpointStore store(c);
  EXPECT_THROW(store.load(),Failure);
  EXPECT_THROW(CheckpointStore duplicate(c),Failure);
}
TEST_F(CheckpointTest, NonHomeDisabledRecoveryPreservesTwistAndMeasuresJoin) {
  auto s=saved();auto cp=s.tracker.checkpoint();State live(c,model);auto q=cp.joints;q[1]+=.003;
  stopped(live,q);auto detail=live.restoreCheckpoint(cp,true);auto actual=live.snapshot();
  EXPECT_TRUE(actual.reference);
  EXPECT_NE(detail.find("without zeroing"),std::string::npos);
  s.tracker.advance(q);
  EXPECT_EQ(actual.tracker.metrics().local,s.tracker.metrics().local);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);
}
TEST_F(CheckpointTest, RequiresAttestationFreshStoppedDwellAndEndpointMatch) {
  auto cp=saved().tracker.checkpoint();
  {State live(c,model);stopped(live,cp.joints);
  EXPECT_THROW(live.restoreCheckpoint(cp,false),Failure);
  EXPECT_FALSE(live.snapshot().reference);}
  {State live(c,model);auto q=cp.joints;q[1]+=.006;stopped(live,q);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);}
  {State live(c,model);stopped(live,cp.joints,7);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);}
  {State live(c,model);Feedback f;f.q=cp.joints;f.mode=4;f.controller_stamp=1;f.received=Clock::now();live.update(f);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);}
  {State live(c,model);stopped(live,cp.joints);std::this_thread::sleep_for(std::chrono::milliseconds(210));
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);}
}
TEST_F(CheckpointTest, RejectsCurrentTwistLimitsAndBusyAdmission) {
  auto cp=saved().tracker.checkpoint();auto narrow=c;narrow.twist_limits={.01,.01};State live(narrow,model);stopped(live,cp.joints);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);
  State idle(c,model);stopped(idle,cp.joints);TaskGate gate;
  ASSERT_TRUE(gate.reserve("owned"));std::string detail;
  EXPECT_FALSE(gate.restoreCheckpoint(idle,cp,true,detail));gate.release();gate.latchFault("stop unconfirmed");
  EXPECT_FALSE(gate.restoreCheckpoint(idle,cp,true,detail));
}
TEST_F(CheckpointTest, NewDisabledClockEpochCannotRestoreReferenceByItself) {
  c.checkpoint_no_unobserved_motion=true;auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);
  live.restoreCheckpoint(cp,true);live.invalidate("power gap");
  Feedback f;f.q=cp.joints;f.mode=4;f.controller_stamp=1;f.received=Clock::now();
  EXPECT_NO_THROW(live.update(f));
  EXPECT_FALSE(live.snapshot().reference);
  EXPECT_THROW(live.restoreCheckpoint(cp,true),Failure);
  f.controller_stamp=2;f.received=Clock::now()+std::chrono::milliseconds(1);
  f.mode=7;
  EXPECT_NO_THROW(live.update(f));
  EXPECT_FALSE(live.snapshot().reference);
  f.controller_stamp=1;f.received=Clock::now();
  EXPECT_THROW(live.update(f),Failure);
}
TEST_F(CheckpointTest, CleanSaveOnlyAfterHealthyReferencedIdleDwell) {
  auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);
  EXPECT_THROW(live.checkpoint(),Failure);
  live.restoreCheckpoint(cp,true);
  EXPECT_NO_THROW(live.checkpoint()); // disabled stopped history is independently durable
  stopped(live,cp.joints,5);
  EXPECT_NO_THROW(live.checkpoint());
  TaskGate gate;gate.reserve("active");
  EXPECT_THROW(gate.checkpoint(live),Failure);
}
TEST_F(CheckpointTest, MotionMarkerSurvivesAbruptProcessExit) {
  {CheckpointStore store(c);store.save(saved());}
  const pid_t pid=::fork();
  ASSERT_GE(pid,0);
  if(pid==0){CheckpointStore store(c);store.markMotionPending();::_exit(0);}
  int status=0;
  ASSERT_EQ(::waitpid(pid,&status,0),pid);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status),0);
  CheckpointStore store(c);
  EXPECT_THROW(store.load(),Failure);
}
TEST_F(CheckpointTest, InterruptedTempsAndReusedProcessIdCannotBlockNewSaves) {
  CheckpointStore store(c);auto s=saved();store.save(s);
  const auto stale=c.checkpoint_file+".tmp."+std::to_string(::getpid());
  {std::ofstream out(stale);out<<"old interrupted payload";}
  EXPECT_NO_THROW(store.markMotionPending());
  EXPECT_THROW(store.load(),Failure);
  EXPECT_NO_THROW(store.save(s));
  EXPECT_NO_THROW(store.load());
  EXPECT_TRUE(std::filesystem::exists(stale)); // Never trust/read/remove unknown temp evidence.
  std::ifstream old(stale);std::string contents;std::getline(old,contents);
  EXPECT_EQ(contents,"old interrupted payload");
}
TEST_F(CheckpointTest, FeedbackLossDuringAlarmQueryPreventsEnableAndAllowsExplicitRetry) {
  enableFixture();auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  auto generation=live.snapshot().reference_generation;std::vector<std::string> calls;int queries=0;
  V4Driver driver(c,[&]{return std::make_unique<RecoveryReply>(calls,[&]{if(++queries==1)live.invalidate("feedback lost during alarm wait");});});
  EXPECT_THROW(driver.enableAfterHistoryRestore(true,[&]{live.validateRecovery(generation,true,c.checkpoint_dwell);}),Failure);
  EXPECT_EQ(calls,std::vector<std::string>{"GetErrorID()"});
  std::this_thread::sleep_for(std::chrono::milliseconds(12));stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  generation=live.snapshot().reference_generation;
  EXPECT_NO_THROW(driver.enableAfterHistoryRestore(true,[&]{live.validateRecovery(generation,true,c.checkpoint_dwell);}));
  EXPECT_EQ(calls.size(),3u);
  EXPECT_EQ(calls.back(),"EnableRobot()");
}
TEST_F(CheckpointTest, RecoveryGenerationCannotBeReusedAfterAnotherRestore) {
  enableFixture();auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  const auto generation=live.snapshot().reference_generation;std::vector<std::string> calls;
  V4Driver driver(c,[&]{return std::make_unique<RecoveryReply>(calls,[&]{
    live.invalidate("new recovery epoch");std::this_thread::sleep_for(std::chrono::milliseconds(12));stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  });});
  EXPECT_THROW(driver.enableAfterHistoryRestore(true,[&]{live.validateRecovery(generation,true,c.checkpoint_dwell);}),Failure);
  EXPECT_TRUE(live.snapshot().reference);
  EXPECT_NE(live.snapshot().reference_generation,generation);
  EXPECT_EQ(calls,std::vector<std::string>{"GetErrorID()"});
}
TEST_F(CheckpointTest, StaleOrMovingFeedbackAfterAlarmQueryPreventsEnable) {
  for(bool moving:{false,true}) {
    enableFixture();auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
    const auto generation=live.snapshot().reference_generation;std::vector<std::string> calls;
    V4Driver driver(c,[&]{return std::make_unique<RecoveryReply>(calls,[&]{
      if(!moving)std::this_thread::sleep_for(std::chrono::milliseconds(210));
      else {Feedback f;f.q=cp.joints;f.mode=4;f.controller_stamp=10000;f.received=Clock::now();f.speed[0]=.01;live.update(f);}
    });});
    EXPECT_THROW(driver.enableAfterHistoryRestore(true,[&]{live.validateRecovery(generation,true,c.checkpoint_dwell);}),Failure);
    EXPECT_EQ(calls,std::vector<std::string>{"GetErrorID()"});
  }
}
TEST_F(CheckpointTest, EnableAckCannotValidateAReferenceLostWhileInFlight) {
  enableFixture();auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  const auto generation=live.snapshot().reference_generation;std::vector<std::string> calls;
  V4Driver driver(c,[&]{return std::make_unique<RecoveryReply>(calls,[]{},[&]{live.invalidate("feedback lost awaiting Enable ACK");});});
  EXPECT_NO_THROW(driver.enableAfterHistoryRestore(true,[&]{live.validateRecovery(generation,true,c.checkpoint_dwell);}));
  EXPECT_THROW(live.validateRecovery(generation,false,0),Failure);
  EXPECT_FALSE(live.snapshot().reference);
  EXPECT_EQ(calls.size(),2u);
}
TEST_F(CheckpointTest, RecoveryEnableRequiresExplicitLiveGuard) {
  enableFixture();std::vector<std::string> calls;
  V4Driver driver(c,[&]{return std::make_unique<RecoveryReply>(calls,[]{});});
  EXPECT_THROW(driver.enableAfterHistoryRestore(true),Failure);
  EXPECT_TRUE(calls.empty());
}
TEST_F(CheckpointTest, RotationRetainsWholeBinaryRecordsAndBoundedFiles) {
  const auto path=(root/"feedback.bin").string();
  {RotatingAudit log(path,2896,3);for(int i=0;i<9;++i){std::string r(1448,char(i));
  ASSERT_TRUE(log.write(r));}}
  size_t total=0;for(const auto &entry:std::filesystem::directory_iterator(root)) {
    const auto name=entry.path().filename().string();if(name.find(".status")!=std::string::npos)continue;
    auto n=entry.file_size();
  EXPECT_LE(n,2896u);
  EXPECT_EQ(n%1448,0u);total+=n;
  }
  EXPECT_LE(total,3*2896u);
  EXPECT_EQ(std::filesystem::file_size(path),1448u);
  std::ifstream f(path,std::ios::binary);char first=0;f.get(first);
  EXPECT_EQ(first,8);
}
TEST_F(CheckpointTest, DiagnosticOverflowOrDiskFailureNeverThrowsIntoControl) {
  {RotatingAudit log((root/"short.csv").string(),1448,1,"header\n");
  EXPECT_FALSE(log.write(std::string(2000,'x')));
  EXPECT_EQ(log.dropped(),1u);}
  RotatingAudit broken((root/"missing"/"feedback.bin").string(),2896,2);
  for(int i=0;i<100 && broken.error().empty();++i)std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_FALSE(broken.error().empty());
  EXPECT_FALSE(broken.write("record"));
}
TEST_F(CheckpointTest, RotationAlsoPreservesAndBoundsPreviousBootLogs) {
  const auto path=(root/"restart.bin").string();
  {AuditDirectory directory(root.string());
  EXPECT_THROW(AuditDirectory duplicate(root.string()),Failure);}
  for(int i=0;i<5;++i) {
    RotatingAudit log(path,2896,3);
  ASSERT_TRUE(log.write(std::string(1448,char(i))));
  }
  EXPECT_TRUE(std::filesystem::exists(path+".2"));
  EXPECT_FALSE(std::filesystem::exists(path+".3"));
  std::ifstream previous(path+".1",std::ios::binary);char first=0;previous.get(first);
  EXPECT_EQ(first,3);
}
TEST_F(CheckpointTest, ReducedRetentionPrunesOnlyCanonicalOwnedArchives) {
  const auto path=(root/"reduce.bin").string();
  for(int i=0;i<16;++i){RotatingAudit log(path,1448,16);ASSERT_TRUE(log.write(std::string(1448,char(i))));}
  {std::ofstream other(path+".notes");other<<"preserve";}
  {std::ofstream other(path+".03");other<<"noncanonical preserve";}
  {std::ofstream other(root/"other.bin.15");other<<"other stream preserve";}
  for(size_t keep:{size_t(3),size_t(1)}) {
    {AuditDirectory directory(root.string());RotatingAudit reduced(path,1448,keep);ASSERT_TRUE(reduced.write(std::string(1448,'z')));}
    for(int i=int(keep);i<16;++i)EXPECT_FALSE(std::filesystem::exists(path+"."+std::to_string(i)));
    EXPECT_TRUE(std::filesystem::exists(path));
    EXPECT_TRUE(std::filesystem::exists(path+".notes"));
    EXPECT_TRUE(std::filesystem::exists(path+".03"));
    EXPECT_TRUE(std::filesystem::exists(root/"other.bin.15"));
  }
}
TEST_F(CheckpointTest, CheckpointPolicyAndConfiguredBoundsAreExplicit) {
  c.checkpoint_auto_restore=true;
  EXPECT_THROW(c.validate(),Failure);
  c.checkpoint_no_unobserved_motion=true;
  EXPECT_NO_THROW(c.validate());
  c.checkpoint_endpoint_tolerance=.01001;
  EXPECT_THROW(c.validate(),Failure);
  c.checkpoint_endpoint_tolerance=.005;c.tracker_options.max_joint_step_rad=.004;
  EXPECT_THROW(c.validate(),Failure);
  c.checkpoint_file.clear();c.checkpoint_auto_restore=false;c.tracker_options.max_joint_step_rad=.1;
  EXPECT_NO_THROW(c.validate());
}

TEST_F(CheckpointTest, DisabledPeriodicSaveRestartRetainsAllLocalPhases) {
  auto original=saved(330*pi/180);auto cp=original.tracker.checkpoint();
  for(uint64_t mode:{4u,5u}) {
    {CheckpointStore store(c);State live(c,model);stopped(live,cp.joints,mode);
      live.restoreCheckpoint(cp,true);TaskGate gate;
      gate.persistCheckpoint(live,[&](const Snapshot &s){store.save(s);});}
    {CheckpointStore store(c);State restarted(c,model);stopped(restarted,cp.joints,mode);
      restarted.restoreCheckpoint(store.load(),true);
      EXPECT_EQ(restarted.snapshot().tracker.metrics().local,original.tracker.metrics().local);
      EXPECT_DOUBLE_EQ(restarted.snapshot().tracker.metrics().total,original.tracker.metrics().total);
      EXPECT_THROW(restarted.restoreCheckpoint(store.load(),true),Failure);}
  }
}
TEST_F(CheckpointTest, ResetOrTimeoutCommandDoesNotEraseIndependentlyConfirmedHistory) {
  for(const std::string failure:{"recv: Connection reset by peer","command timed out"}) {
    auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);
    live.restoreCheckpoint(cp,true);auto before=live.snapshot();const auto completed=Clock::now();
    std::thread feedback([&]{std::this_thread::sleep_for(std::chrono::milliseconds(5));stopped(live,cp.joints);});
    EXPECT_NO_THROW(live.confirmRecoveryEndpoint(before.reference_generation,before.feedback,completed,.05));feedback.join();
    EXPECT_TRUE(live.snapshot().reference)<<failure;
    EXPECT_EQ(live.snapshot().tracker.metrics().local,cp.local);
    EXPECT_EQ(live.snapshot().feedback.mode,4u); // no mode5 ACK required for history
  }
}
TEST_F(CheckpointTest, EnableAttemptWithoutNewConfirmationInvalidatesOnlyItsHistory) {
  auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  auto before=live.snapshot();
  EXPECT_THROW(live.confirmRecoveryEndpoint(before.reference_generation,before.feedback,Clock::now(),.01),Failure);
  EXPECT_FALSE(live.snapshot().reference);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  EXPECT_THROW(live.confirmRecoveryEndpoint(before.reference_generation,before.feedback,Clock::now(),.01),Failure);
  EXPECT_TRUE(live.snapshot().reference); // a stale completion cannot invalidate a newer recovery
}
TEST_F(CheckpointTest, EnableConfirmationRejectsMotionEpochRegressionGapAndEndpointDrift) {
  for(int risk=0;risk<5;++risk) {
    auto cp=saved().tracker.checkpoint();State live(c,model);stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
    auto before=live.snapshot();Feedback f=before.feedback;f.received=Clock::now();f.controller_stamp++;
    if(risk==0)f.speed[0]=.01;
    if(risk==1)f.controller_stamp=1;
    if(risk==2)f.received+=std::chrono::milliseconds(250);
    if(risk==3)f.q[0]+=.006;
    if(risk==4)f.mode=7;
    try {live.update(f);} catch(const Failure &) {}
    EXPECT_THROW(live.confirmRecoveryEndpoint(before.reference_generation,before.feedback,Clock::now(),.01),Failure)<<risk;
    EXPECT_FALSE(live.snapshot().reference)<<risk;
  }
}
TEST_F(CheckpointTest, SaveFailureDoesNotBecomeSuccessOrOverwriteExistingGoodFile) {
  CheckpointStore store(c);auto original=saved();store.save(original);
  std::filesystem::rename(root,root.string()+"-moved");
  EXPECT_THROW(store.save(original),Failure);
  std::filesystem::rename(root.string()+"-moved",root);
  EXPECT_EQ(store.load().local,original.tracker.metrics().local);
}
TEST_F(CheckpointTest, DurableSaveHoldsAdmissionUntilWriterCompletes) {
  State live(c,model);auto cp=saved().tracker.checkpoint();stopped(live,cp.joints);live.restoreCheckpoint(cp,true);
  TaskGate gate;std::atomic<bool> saving{false},admitted{false};
  std::thread writer([&]{gate.persistCheckpoint(live,[&](const Snapshot &){saving=true;std::this_thread::sleep_for(std::chrono::milliseconds(25));EXPECT_FALSE(admitted);});});
  while(!saving)std::this_thread::yield();
  std::thread action([&]{admitted=gate.reserve("motion");});writer.join();action.join();
  EXPECT_TRUE(admitted);gate.release();
}
