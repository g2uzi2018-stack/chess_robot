#include "net.hpp"
#include "dashboard_reply.hpp"
#include <iostream>
#include <regex>
#include <set>
#include <iomanip>

// Exact whole-command allowlist. No raw arbitrary TCP command mode exists.
bool allowed(const std::string& profile,const std::string& cmd){
  if(profile!="common" && profile!="v3" && profile!="v4") return false;
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
      for(auto cmd:{"RobotMode()","GetAngle()","GetPose()","GetErrorID()"})
        if(!allowed("common",cmd) || allowed("unknown",cmd)) throw std::runtime_error("profile guard failed");
      if(allowed("common","PositiveKin(0,0,-90,0,90,0,user=0,tool=0)")) throw std::runtime_error("common profile allowed FK");
      if(parse_dashboard_reply("0,{1,2,3,4,5,6},GetAngle();","GetAngle()").error!=0 ||
         parse_dashboard_reply("-10000,{},GetPose();","GetPose()").error!=-10000 ||
         parse_dashboard_reply("0,{[[1],[2]]},GetErrorID();","GetErrorID()").payload!="[[1],[2]]" ||
         parse_dashboard_reply(" 0, {{}}, robotmode();\r\n","RobotMode()").payload!="{}")
        throw std::runtime_error("valid reply rejected");
      for(auto reply:{"bad,{4},RobotMode();","0,{4},GetAngle();","0,{4,RobotMode();","0,{4},RobotMode();junk","0,{4};,RobotMode();","99999999999999999999999,{},RobotMode();"}){
        bool refused=false;try{parse_dashboard_reply(reply,"RobotMode()");}catch(const std::runtime_error&){refused=true;}
        if(!refused) throw std::runtime_error("malformed/mismatched reply accepted");
      }
      std::cout<<"PASS offline: command/profile guards and dashboard success/error/echo/framing fixtures; no socket opened\n";return 0;
    }
    if(argc==3 && std::string(argv[1])=="--probe"){
      std::cout<<"utc_ns,ip,port,connect_ms,status\n";
      for(int port:{29999,30003,30004}){
        auto t=steady_ns();try{Socket s(argv[2],port);std::cout<<utc_ns()<<','<<argv[2]<<','<<port<<','<<(steady_ns()-t)/1e6<<",connected_no_bytes_sent\n";}
        catch(const std::exception& e){std::cout<<utc_ns()<<','<<argv[2]<<','<<port<<','<<(steady_ns()-t)/1e6<<','<<e.what()<<'\n';}
      }return 0;
    }
    if(argc!=5 || std::string(argv[1])!="--query") throw std::runtime_error("usage: --probe IPv4 | --query IPv4 common|v3|v4 'allowed-command' | --self-test");
    std::string profile=argv[3],cmd=argv[4];
    if(!allowed(profile,cmd)) throw std::runtime_error("query refused before opening socket");
    Socket s(argv[2],29999);auto utc=utc_ns(), start=steady_ns();auto reply=s.query(cmd);
    std::cout<<"send_utc_ns="<<utc<<" receive_steady_ns="<<steady_ns()<<" rtt_ms="<<(steady_ns()-start)/1e6<<"\ncommand="<<cmd<<"\nreply="<<reply<<'\n';
    const auto parsed=parse_dashboard_reply(reply,cmd);
    std::cout<<"controller_error="<<parsed.error<<" payload="<<parsed.payload<<'\n';
    return parsed.error==0?0:3;
  }catch(const QueryFailure& e){
    std::cerr<<e.what()<<"\npartial_reply_bytes="<<e.partial_reply.size()<<"\npartial_reply_hex=";
    for(size_t i=0;i<std::min(size_t(256),e.partial_reply.size());++i)
      std::cerr<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<unsigned int>(static_cast<unsigned char>(e.partial_reply[i]));
    std::cerr<<'\n';return 2;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
