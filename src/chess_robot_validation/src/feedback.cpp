#include "net.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

template<class T> T field(const std::array<unsigned char,1440>& b,size_t offset){T value;std::memcpy(&value,b.data()+offset,sizeof(T));return value;}
void validate(const std::array<unsigned char,1440>& b){
  const uint16_t endian=1;if(*reinterpret_cast<const unsigned char*>(&endian)!=1) throw std::runtime_error("little-endian host required");
  if(field<uint16_t>(b,0)!=1440 || field<uint64_t>(b,48)!=0x0123456789ABCDEFULL) throw std::runtime_error("feedback framing/profile mismatch");
  for(size_t off:{size_t(192),size_t(432),size_t(480),size_t(624)})for(size_t j=0;j<6;j++)if(!std::isfinite(field<double>(b,off+8*j))) throw std::runtime_error("nonfinite feedback");
}
int main(int argc,char** argv){
  try{
    if(argc==2 && std::string(argv[1])=="--self-test"){
      std::array<unsigned char,1440>b{};uint16_t n=1440;uint64_t magic=0x0123456789ABCDEFULL;
      std::memcpy(b.data(),&n,2);std::memcpy(b.data()+48,&magic,8);validate(b);
      b[0]=0;bool refused=false;try{validate(b);}catch(...){refused=true;}
      if(!refused) throw std::runtime_error("bad frame accepted");
      std::memcpy(b.data(),&n,2);b[48]=0;refused=false;try{validate(b);}catch(...){refused=true;}
      if(!refused) throw std::runtime_error("bad sentinel accepted");
      std::memcpy(b.data()+48,&magic,8);double bad=std::numeric_limits<double>::quiet_NaN();std::memcpy(b.data()+432,&bad,8);
      refused=false;try{validate(b);}catch(...){refused=true;}
      if(!refused) throw std::runtime_error("NaN accepted");
      std::cout<<"PASS offline frame fixtures: valid, bad length, bad sentinel, NaN; no ROS publication or socket\n";return 0;
    }
    if(argc!=6) throw std::runtime_error("usage: IPv4 v3|v4 seconds output_prefix --confirmed-profile-and-degrees");
    std::string profile=argv[2];if((profile!="v3" && profile!="v4") || std::string(argv[5])!="--confirmed-profile-and-degrees") throw std::runtime_error("explicit verified profile/units required");
    double seconds=std::stod(argv[3]);if(!(seconds>0 && seconds<=600)) throw std::runtime_error("duration must be 0..600 s");
    // Common fields only. Firmware-dependent tail is retained in raw packets, not inferred.
    Socket socket(argv[1],30004);std::ofstream raw(std::string(argv[4])+".bin",std::ios::binary), csv(std::string(argv[4])+".csv");
    if(!raw || !csv) throw std::runtime_error("cannot open evidence files");
    csv<<"receive_utc_ns,receive_steady_ns,controller_timestamp_raw,robot_mode,speed_scaling,user_index,tool_index";
    for(auto tag:{"q_target_deg","q_actual_deg","qd_actual_deg_s","tcp_actual_mm_deg"})for(int i=1;i<=6;i++)csv<<','<<tag<<i;
    csv<<'\n'<<std::setprecision(17);
    rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("nova5_readonly_feedback");
    auto pub=node->create_publisher<sensor_msgs::msg::JointState>("/nova5/real_joint_states",10);
    auto end=Steady::now()+std::chrono::duration<double>(seconds);uint64_t last=0;bool have_last=false;size_t count=0;
    while(rclcpp::ok() && Steady::now()<end){
      std::array<unsigned char,1440>b{};socket.exact(b.data(),b.size(),500);auto utc=utc_ns(), mono=steady_ns();
      raw.write(reinterpret_cast<const char*>(b.data()),b.size());validate(b);auto stamp=field<uint64_t>(b,32);
      if(have_last && stamp<=last) throw std::runtime_error("non-increasing controller timestamp; publication stopped");
      last=stamp;have_last=true;
      csv<<utc<<','<<mono<<','<<stamp<<','<<field<uint64_t>(b,24)<<','<<field<double>(b,64)<<','<<int(b[1012])<<','<<int(b[1013]);
      for(size_t off:{size_t(192),size_t(432),size_t(480),size_t(624)}) {
        for(size_t j=0;j<6;j++) csv<<','<<field<double>(b,off+8*j);
      }
      csv<<'\n';
      sensor_msgs::msg::JointState msg;msg.header.stamp=node->now();
      for(size_t j=0;j<6;j++){msg.name.push_back("joint"+std::to_string(j+1));msg.position.push_back(field<double>(b,432+8*j)*std::acos(-1)/180.0);msg.velocity.push_back(field<double>(b,480+8*j)*std::acos(-1)/180.0);}
      pub->publish(msg);rclcpp::spin_some(node);count++;
    }
    rclcpp::shutdown();std::cout<<"real_frames="<<count<<" profile="<<profile<<"; stamps are reception time; raw controller clock retained\n";
    return count?0:2;
  }catch(const std::exception& e){if(rclcpp::ok())rclcpp::shutdown();std::cerr<<e.what()<<'\n';return 2;}
}
