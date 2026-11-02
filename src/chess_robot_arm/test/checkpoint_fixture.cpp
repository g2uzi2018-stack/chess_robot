// Loopback/simulation-only bootstrap for node restart tests. Not a live initializer.
#include "chess_robot_arm/checkpoint.hpp"
#include "chess_robot_arm/driver.hpp"
#include <iostream>
#include <iomanip>
using namespace chess_robot_arm;
int main(int argc,char **argv) {
  try {
    if(argc!=6)throw std::runtime_error("fixture URDF SAVE DASHBOARD_PORT FEEDBACK_PORT clean|dirty");
    Config c;c.urdf=argv[1];c.checkpoint_file=argv[2];c.dashboard_port=std::stoi(argv[3]);c.feedback_port=std::stoi(argv[4]);
    c.test_fixture=true;c.controller_family="v4";c.controller_firmware="checkpoint-loopback-fixture";
    c.flange_tcp.translation().z()=.2;
    c.flange_tcp.linear()=(Eigen::AngleAxisd(-pi/2,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(pi/2,Eigen::Vector3d::UnitX())).toRotationMatrix();
    c.validate();auto model=std::make_shared<chess_robot_twist::Model>(c.urdf,"base_link","Link6",c.flange_tcp,c.twist_reference);
    chess_robot_twist::Tracker t(model);t.initializeAtHome(c.twist_reference,c.home_tolerance);
    if(std::string(argv[5])=="clean-nonzero") {
      for(int i=1;i<=40;++i) {auto q=c.twist_reference;q[5]+=.4*i/40;t.advance(q);}
    }
    Snapshot s(t);s.fresh=s.reference=true;s.feedback.mode=5;s.feedback.q=t.lastJoints();s.feedback.controller_stamp=1;
    CheckpointStore store(c);store.save(s);if(std::string(argv[5])=="dirty")store.markMotionPending();
    auto pose=toSdkPose(model->flange(t.lastJoints()));std::cout<<std::setprecision(17)<<"{\"fixture_only\":true,\"q_deg\":[";
    for(size_t i=0;i<6;++i)std::cout<<(i?",":"")<<t.lastJoints()[i]*180/pi;
    std::cout<<"],\"flange_mm_deg\":[";for(size_t i=0;i<6;++i)std::cout<<(i?",":"")<<pose[i];std::cout<<"]}"<<std::endl;
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}
