#include "net.hpp"
#include <iostream>
#include <regex>
#include <set>

// Exact whole-command allowlist. No raw arbitrary TCP command mode exists.
bool allowed(const std::string& profile,const std::string& cmd){
  static const std::set<std::string> common{"RobotMode()","GetAngle()","GetPose()","GetErrorID()"};
  if(common.count(cmd)) return true;
  const std::string number="[-+]?[0-9]+(?:\\.[0-9]+)?";
  std::string six=number;for(int i=1;i<6;i++) six+=","+number;
  if(profile=="v3") return std::regex_match(cmd,std::regex("PositiveSolution\\("+six+",[0-9],[0-9]\\)")) || std::regex_match(cmd,std::regex("InverseSolution\\("+six+",[0-9],[0-9]\\)"));
  if(profile=="v4") return std::regex_match(cmd,std::regex("GetPose\\(user=0,tool=0\\)")) ||
    std::regex_match(cmd,std::regex("PositiveKin\\("+six+",user=0,tool=0\\)")) ||
    std::regex_match(cmd,std::regex("InverseKin\\("+six+",useJointNear=1,jointNear=\\{"+six+"\\},user=0,tool=0\\)"));
  return false;
}
int main(int argc,char** argv){
  try{
    if(argc==2 && std::string(argv[1])=="--self-test"){
      for(auto p:{"v3","v4"}){
        for(auto s:{"EnableRobot()","MovJ(0,0,0,0,0,0)","SpeedFactor(101)","AccJ(200)","RobotMode();EnableRobot()","GetPose()\nMovL(1,2,3,0,0,0)","PositiveKin(nan,0,0,0,0,0,user=0,tool=0)"})
          if(allowed(p,s)) throw std::runtime_error("unsafe allowlist");
        if(!allowed(p,"GetAngle()")) throw std::runtime_error("query rejected");
      }
      if(!allowed("v3","PositiveSolution(0,0,-90,0,90,0,0,0)") || !allowed("v4","PositiveKin(0,0,-90,0,90,0,user=0,tool=0)")) throw std::runtime_error("FK format rejected");
      std::cout<<"PASS: 14 denied cases + 4 valid queries; offline, no socket opened\n";return 0;
    }
    if(argc==3 && std::string(argv[1])=="--probe"){
      std::cout<<"utc_ns,ip,port,connect_ms,status\n";
      for(int port:{29999,30003,30004}){
        auto t=steady_ns();try{Socket s(argv[2],port);std::cout<<utc_ns()<<','<<argv[2]<<','<<port<<','<<(steady_ns()-t)/1e6<<",connected_no_bytes_sent\n";}
        catch(const std::exception& e){std::cout<<utc_ns()<<','<<argv[2]<<','<<port<<','<<(steady_ns()-t)/1e6<<','<<e.what()<<'\n';}
      }return 0;
    }
    if(argc!=5 || std::string(argv[1])!="--query") throw std::runtime_error("usage: --probe IPv4 | --query IPv4 v3|v4 'allowed-command' | --self-test");
    std::string profile=argv[3],cmd=argv[4];
    if((profile!="v3" && profile!="v4") || !allowed(profile,cmd)) throw std::runtime_error("query refused before opening socket");
    Socket s(argv[2],29999);auto utc=utc_ns(), start=steady_ns();auto reply=s.query(cmd);
    std::cout<<"send_utc_ns="<<utc<<" receive_steady_ns="<<steady_ns()<<" rtt_ms="<<(steady_ns()-start)/1e6<<"\ncommand="<<cmd<<"\nreply="<<reply<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
