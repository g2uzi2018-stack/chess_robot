// Offline records only. This executable has no Driver/socket instance and cannot actuate.
#include "chess_robot_arm/driver.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
using namespace chess_robot_arm;
int main(int argc,char **argv) {
  try {
    if(argc!=2) throw std::runtime_error("usage: replay_feedback <feedback.bin>");
    std::ifstream in(argv[1],std::ios::binary);
    if(!in) throw std::runtime_error("cannot open feedback record");
    size_t frames=0,unique=0,duplicates=0;
    FeedbackSequence sequence;
    double peak=0,max_step=0;Joints previous{};bool have=false;
    int64_t host=0;
    while(in.read(reinterpret_cast<char*>(&host),sizeof(host))) {
      FeedbackFrame bytes;
      if(!in.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))
        throw std::runtime_error("truncated record");
      // Simulated ROS clock is deliberately unrelated to recorded UTC/controller time.
      const int64_t ros_ns=42000000000LL+int64_t(frames)*8000000;
      auto f=decodeFeedbackFrame(bytes,Clock::time_point{},ros_ns);
      if(f.ros_received_ns!=ros_ns) throw std::runtime_error("ROS stamp changed");
      // Independently compare all used fields to the previously validated offsets.
      if(f.controller_stamp!=feedback_field<uint64_t>(bytes,32) ||
         f.mode!=feedback_field<uint64_t>(bytes,24)) throw std::runtime_error("header mismatch");
      for(size_t j=0;j<6;++j) {
        if(f.q[j]!=feedback_field<double>(bytes,432+j*8)*pi/180 ||
           f.speed[j]!=feedback_field<double>(bytes,480+j*8)*pi/180)
          throw std::runtime_error("official/common subset mismatch");
        peak=std::max(peak,std::abs(f.speed[j]));
      }
      ++frames;
      if(!sequence.accept(bytes)) {++duplicates;continue;}
      ++unique;
      if(have)max_step=std::max(max_step,jointDistance(previous,f.q));
      previous=f.q;have=true;
    }
    if(in.gcount()!=0 || !frames) throw std::runtime_error("empty/truncated timestamp");
    std::cout<<std::setprecision(12)<<"{\"status\":\"PASS\",\"frames\":"<<frames
      <<",\"unique\":"<<unique<<",\"duplicates\":"<<duplicates
      <<",\"peak_speed_rad_s_assuming_deg_s\":"<<peak<<",\"max_joint_step_rad\":"<<max_step
      <<",\"motion_commands\":0,\"arrival_claimed\":false,\"all_used_fields_equal\":true}\n";
    return 0;
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
