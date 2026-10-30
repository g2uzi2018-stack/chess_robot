// Offline endpoint snapshots and original audits only; no Driver or sockets.
#include "chess_robot_arm/core.hpp"
#include "chess_robot_arm/driver.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
using namespace chess_robot_arm;
namespace {
// Repeated test snapshots keep the live side fresh while a large original audit
// replays. This never writes audit frames or claims new physical observations.
struct SnapshotPump {
  std::atomic<bool> stop{false};
  std::thread worker;
  SnapshotPump(State &state, Feedback f, double dwell) {
    state.update(f);
    worker=std::thread([this,&state,f]() mutable {
      while (!stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        ++f.controller_stamp;f.received=Clock::now();state.update(f);
      }
    });
    std::this_thread::sleep_for(std::chrono::duration<double>(dwell+.01));
  }
  ~SnapshotPump() {stop=true;worker.join();}
};
}
int main(int argc,char **argv) {
  try {
    if (argc<7) throw std::runtime_error(
        "usage: restore_reference_offline URDF TOLERANCE_RAD FAULT_EPOCH_JOIN_FILE STOPPED_JOIN_FILE LIVE_Q_MODE_STAMP_FILE [--fault-restart FILE OFFSET SHA256] AUDIT...");
    Config c;c.urdf=argv[1];c.power_cycle_handoff=true;
    c.power_cycle_endpoint_tolerance=std::stod(argv[2]);c.power_cycle_join_file=argv[3];
    c.power_cycle_stopped_join_file=argv[4];
    int audit_start=6;
    if (argc>6 && std::string(argv[6])=="--fault-restart") {
      if (argc<11)throw std::runtime_error("fault restart triple and audit list required");
      c.power_cycle_fault_restart_file=argv[7];c.power_cycle_fault_restart_offset_bytes=std::stoll(argv[8]);
      c.power_cycle_fault_restart_sha256=argv[9];audit_start=10;
    }
    c.freshness=.2;c.stationary_speed=.002;c.dwell=.4;c.joint_tolerance=.0005;
    c.flange_tcp.translation().z()=.2;
    c.flange_tcp.linear()=(Eigen::AngleAxisd(-pi/2,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pi/2,Eigen::Vector3d::UnitX())).toRotationMatrix();
    c.validate();
    auto m=std::make_shared<chess_robot_twist::Model>(c.urdf,"base_link","Link6",c.flange_tcp,c.home);
    Feedback f;std::ifstream live(argv[5]);
    for (double &q:f.q) if (!(live>>q)) throw std::runtime_error("six measured joint radians required");
    if (!(live>>f.mode>>f.controller_stamp)) throw std::runtime_error("snapshot mode and controller stamp required");
    f.received=Clock::now();
    State state(c,m);SnapshotPump snapshots(state,f,c.dwell);
    std::vector<std::string> files;
    for (int i=audit_start;i<argc;++i)files.emplace_back(argv[i]);
    std::cout<<"offline_snapshot_only=true; motion_commands=0\n";
    std::cout<<state.restore(files,true)<<'\n';
    const auto snapshot=state.snapshot();
    // Independent direct tracker replay checks that restore did not initialize
    // again at either epoch boundary or discard the session's measured motion.
    chess_robot_twist::Tracker direct(m,c.tracker_options);
    FeedbackSequence sequence;bool first=true;
    for (const auto &path:files) {
      if (path==c.power_cycle_join_file || path==c.power_cycle_stopped_join_file)
        sequence=FeedbackSequence{};
      std::ifstream audit(path,std::ios::binary);int64_t host=0,offset=0;
      while (audit.read(reinterpret_cast<char*>(&host),sizeof(host))) {
        FeedbackFrame bytes{};
        if (!audit.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))
          throw std::runtime_error("truncated direct replay frame");
        if (path==c.power_cycle_fault_restart_file && offset==c.power_cycle_fault_restart_offset_bytes)
          sequence=FeedbackSequence{};
        offset+=sizeof(host)+bytes.size();
        if (!sequence.accept(bytes))continue;
        const auto recorded=decodeFeedbackFrame(bytes,Clock::time_point{},0);
        if (first) {direct.initializeAtHome(recorded.q,c.home_tolerance);first=false;}
        else direct.advance(recorded.q);
      }
      if (audit.gcount()!=0)throw std::runtime_error("truncated direct replay timestamp");
    }
    direct.advance(snapshot.feedback.q);
    const auto expected=direct.metrics(),actual=snapshot.tracker.metrics();
    if (std::abs(expected.net-actual.net)>1e-10 || std::abs(expected.total-actual.total)>1e-10 ||
        std::abs(expected.energy-actual.energy)>1e-10 || expected.local!=actual.local ||
        direct.lastJoints()!=snapshot.tracker.lastJoints())
      throw std::runtime_error("restored history differs from independent direct tracker replay");
    std::cout<<"independent_replay_equal=true; twist_not_zeroed=true\n";
    std::cout<<std::setprecision(15)<<"reference_valid="<<snapshot.reference
             <<"; net="<<snapshot.tracker.metrics().net
             <<"; total="<<snapshot.tracker.metrics().total;
    const Eigen::Vector3d tcp=m->tcp(snapshot.feedback.q).translation();
    std::cout<<"; tcp_xyz_m=["<<tcp.x()<<','<<tcp.y()<<','<<tcp.z()<<"]\n";
    return 0;
  } catch (const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
