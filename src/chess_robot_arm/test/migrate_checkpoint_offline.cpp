// One-time migration only. Reads bounded SHA-bound originals; never creates a
// Driver, connects a socket, fabricates raw audit records, or sends a command.
#include "chess_robot_arm/checkpoint.hpp"
#include "chess_robot_arm/driver.hpp"
#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>

using namespace chess_robot_arm;
namespace {
void check(bool b,const std::string &why){if(!b)throw std::runtime_error(why);}
std::string sha(const std::string &path,uint64_t bytes=0) {
  std::ifstream f(path,std::ios::binary);check(bool(f),"missing source audit");
  if(!bytes)bytes=std::filesystem::file_size(path);
  auto ctx=std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  check(ctx && EVP_DigestInit_ex(ctx.get(),EVP_sha256(),nullptr)==1,"digest init");
  std::array<char,1024*1024> block{};
  while(bytes) {auto n=std::min<uint64_t>(bytes,block.size());f.read(block.data(),n);check(uint64_t(f.gcount())==n,"truncated prefix");
    check(EVP_DigestUpdate(ctx.get(),block.data(),n)==1,"digest update");bytes-=n;}
  unsigned char result[32];unsigned n=0;check(EVP_DigestFinal_ex(ctx.get(),result,&n)==1 && n==32,"digest finish");
  std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto b:result)out<<std::setw(2)<<unsigned(b);return out.str();
}
struct OfflinePump {
  std::atomic<bool> stop{false};std::thread worker;
  OfflinePump(State &s,Feedback f,double dwell) {
    s.update(f);worker=std::thread([this,&s,f]() mutable {
      while(!stop){std::this_thread::sleep_for(std::chrono::milliseconds(8));f.received=Clock::now();++f.controller_stamp;s.update(f);}
    });std::this_thread::sleep_for(std::chrono::duration<double>(dwell+.01));
  }
  ~OfflinePump(){stop=true;worker.join();}
};
double speed(const Feedback &f){double v=0;for(double x:f.speed)v=std::max(v,std::abs(x));return v;}
}
int main(int argc,char **argv) {
  try {
    check(argc==2,"usage: migrate_checkpoint_offline REVIEWED_PLAN.yaml");auto plan=YAML::LoadFile(argv[1]);
    check(plan["no_unobserved_motion"].as<bool>(),"operator no-movement policy required");
    auto p=YAML::LoadFile(plan["parameters"].as<std::string>())["chess_robot_arm"]["ros__parameters"];
    auto h=YAML::LoadFile(plan["home_parameters"].as<std::string>())["chess_robot_arm"]["ros__parameters"];
    Config c;c.urdf=p["urdf_file"].as<std::string>();c.ip=p["controller"]["ip"].as<std::string>();
    c.controller_family=p["controller"]["protocol_family"].as<std::string>();c.controller_firmware=p["controller"]["firmware_version"].as<std::string>();
    auto home=h["home"]["joint_positions_rad"].as<std::vector<double>>();check(home.size()==6,"Home dimension");std::copy(home.begin(),home.end(),c.home.begin());
    auto tr=p["tcp"]["translation_m"].as<std::vector<double>>(),rpy=p["tcp"]["rpy_rad"].as<std::vector<double>>();check(tr.size()==3 && rpy.size()==3,"TCP dimension");
    c.flange_tcp.translation()=Eigen::Vector3d(tr[0],tr[1],tr[2]);
    c.flange_tcp.linear()=(Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).toRotationMatrix();
    c.freshness=.2;c.stationary_speed=.002;c.dwell=.4;c.checkpoint_dwell=1;c.joint_tolerance=.0005;
    c.power_cycle_handoff=true;c.power_cycle_endpoint_tolerance=p["reference"]["power_cycle_endpoint_tolerance_rad"].as<double>();
    c.power_cycle_join_file=p["reference"]["power_cycle_join_file"].as<std::string>();c.power_cycle_stopped_join_file=p["reference"]["power_cycle_stopped_join_file"].as<std::string>();
    c.power_cycle_fault_restart_file=p["reference"]["power_cycle_fault_restart_file"].as<std::string>();
    c.power_cycle_fault_restart_offset_bytes=p["reference"]["power_cycle_fault_restart_offset_bytes"].as<int64_t>();
    c.power_cycle_fault_restart_sha256=p["reference"]["power_cycle_fault_restart_sha256"].as<std::string>();
    c.checkpoint_file=plan["output_checkpoint"].as<std::string>();check(!std::filesystem::exists(c.checkpoint_file),"migration never overwrites an existing checkpoint");c.validate();
    auto model=std::make_shared<chess_robot_twist::Model>(c.urdf,"base_link","Link6",c.flange_tcp,c.home);
    Feedback endpoint;std::ifstream snapshot(plan["old_endpoint_snapshot"].as<std::string>());
    for(double &q:endpoint.q)check(bool(snapshot>>q),"old endpoint q missing");
    check(bool(snapshot>>endpoint.mode>>endpoint.controller_stamp),"old endpoint mode/stamp missing");endpoint.received=Clock::now();
    std::vector<std::string> files;
    for(const auto &item:plan["old_audits"]) {
      auto path=item["path"].as<std::string>();check(sha(path)==item["sha256"].as<std::string>(),"old audit SHA mismatch: "+path);files.push_back(path);
    }
    State old(c,model);std::optional<Snapshot> base;
    {OfflinePump pump(old,endpoint,c.dwell);std::cout<<"offline historical replay; physical_commands=0"<<std::endl;
      std::cout<<old.restore(files,true)<<std::endl;base=old.snapshot();}
    auto tracker=base->tracker;
    auto continuation=plan["continuation"];auto path=continuation["path"].as<std::string>();
    auto bytes=continuation["prefix_bytes"].as<uint64_t>();auto expected=continuation["prefix_sha256"].as<std::string>();
    check(bytes>0 && bytes%1448==0 && bytes<=16ULL*1024*1024*1024,"bounded complete continuation prefix required");
    check(sha(path,bytes)==expected,"continuation prefix SHA mismatch");
    FeedbackSequence sequence;std::ifstream f(path,std::ios::binary);int64_t previous_host=0,last_accepted_host=0,idle_since=0,fault_anchor_host=0;
    Feedback previous;Joints idle_anchor{},fault_anchor{};bool fault=false,restarted=false,seen_enabled=false;
    std::optional<Snapshot> stopped_save;int64_t stopped_save_host=0;
    uint64_t restart_offset=0,fault_count=0;double fault_max=0;uint64_t accepted=0;
    for(uint64_t offset=0;offset<bytes;offset+=1448) {
      int64_t host=0;FeedbackFrame frame{};f.read(reinterpret_cast<char*>(&host),8);f.read(reinterpret_cast<char*>(frame.data()),frame.size());check(bool(f),"truncated continuation");
      auto row=decodeFeedbackFrame(frame,Clock::time_point{},0);check(model->withinJointBounds(row.q),"continuation joint bounds");
      if(previous_host) {
        const double dt=double(host-previous_host)*1e-9;check(dt>0,"continuation host regression");
        if(dt>c.freshness) {
          check(!restarted && fault && previous.mode==9 && row.mode==4 && speed(row)<=c.stationary_speed &&
                jointDistance(previous.q,row.q)<=.001,"unreviewed gap in continuation");
          restarted=true;restart_offset=offset;sequence=FeedbackSequence{};
        }
      }
      if(restarted)check(row.mode==4 && speed(row)<=c.stationary_speed && jointDistance(row.q,fault_anchor)<=.005,"raw post-restart suffix is not disabled/matching");
      if(!sequence.accept(frame)){check(row.mode!=9,"fault sample must advance controller stamp");previous_host=host;continue;}
      if(last_accepted_host && double(host-last_accepted_host)*1e-9>c.freshness)idle_since=0;
      last_accepted_host=host;
      ++accepted;
      if(!previous_host)check(row.mode==4 && speed(row)<=c.stationary_speed && jointDistance(row.q,tracker.lastJoints())<=.001,"continuation does not join old stopped endpoint");
      if(restarted)check(row.mode==4 && speed(row)<=c.stationary_speed && jointDistance(row.q,fault_anchor)<=.005,"post-restart suffix is not disabled/matching");
      else if(row.mode==9) {
        if(!fault) {
          check(previous.mode==5 && stopped_save && stopped_save_host>0 &&
                    double(host-stopped_save_host)*1e-9<=c.freshness,"fault not preceded by recent confirmed stopped save");
          fault=true;fault_anchor=stopped_save->feedback.q;fault_anchor_host=stopped_save_host;
        }
        check(double(host-fault_anchor_host)*1e-9<=c.freshness,"fault suffix duration");
        fault_max=std::max(fault_max,jointDistance(row.q,fault_anchor));check(fault_max<=.005,"fault suffix movement exceeds stopped bound");++fault_count;
      } else {
        check(!fault,"unexpected post-fault healthy data before declared restart");
        check(row.mode==4 || row.mode==5 || row.mode==7,"unsupported continuation mode");
        // EnableRobot's recorded brake transition can still report mode4 with
        // nonzero velocity. Replay every measured q; do not call it stopped dwell.
        if(row.mode==4)check(!seen_enabled && jointDistance(row.q,endpoint.q)<=.001,"unexpected disabled data before restart");
        if(row.mode==5 || row.mode==7)seen_enabled=true;
      }
      tracker.advance(row.q);check(chess_robot_twist::withinLimits(tracker.metrics(),c.twist_limits),"continuation twist limits");
      if(!fault && row.mode==5 && speed(row)<=c.stationary_speed) {
        if(!idle_since || jointDistance(row.q,idle_anchor)>.001){idle_since=host;idle_anchor=row.q;}
        if(double(host-idle_since)*1e-9>=c.checkpoint_dwell) {
          stopped_save.emplace(tracker);stopped_save->feedback=row;stopped_save->reference=stopped_save->fresh=true;stopped_save_host=host;
        }
      } else if(!fault)idle_since=0;
      if(!fault && (row.mode==7 || (stopped_save && jointDistance(row.q,stopped_save->feedback.q)>.005))) {
        stopped_save.reset();stopped_save_host=0;
      }
      previous=row;previous_host=host;
      if(accepted%1000000==0)std::cout<<"continuation accepted="<<accepted<<std::endl;
    }
    check(fault && restarted && stopped_save && fault_count>0,"required stopped fault/restart not found");
    auto direct=stopped_save->tracker;direct.advance(previous.q);
    const auto a=direct.metrics(),b=tracker.metrics();check(a.local.size()==b.local.size(),"direct comparison dimensions");
    for(size_t i=0;i<a.local.size();++i)check(std::abs(a.local[i]-b.local[i])<1e-8,"fault/suffix path differs from stopped endpoint join");
    check(sha(path,bytes)==expected,"continuation source changed during migration");
    CheckpointStore store(c);store.save(*stopped_save);auto restored=store.load();
    check(restored.local==stopped_save->tracker.metrics().local,"stored winding mismatch");
    std::cout<<std::setprecision(17)<<"migration_complete=true; physical_commands=0; accepted_continuation="<<accepted
      <<"; restart_offset="<<restart_offset<<"; fault_frames="<<fault_count<<"; fault_max_delta_rad="<<fault_max
      <<"; live_join_max_delta_rad="<<jointDistance(restored.joints,previous.q)
      <<"; saved_net="<<stopped_save->tracker.metrics().net<<"; saved_total="<<stopped_save->tracker.metrics().total
      <<"; model_identity="<<store.identity()<<std::endl;
    return 0;
  } catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}
