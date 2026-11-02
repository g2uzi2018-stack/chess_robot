#include "chess_robot_arm/checkpoint.hpp"
#include <yaml-cpp/yaml.h>
#include <openssl/evp.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cerrno>
#include <algorithm>
#include <locale>
#include <cstdlib>

namespace chess_robot_arm {
namespace {
void check(bool value,const std::string &why) { if (!value) throw Failure(REFERENCE_INVALID,why); }
std::string digest(const std::string &s) {
  unsigned char bytes[EVP_MAX_MD_SIZE];unsigned size=0;
  check(EVP_Digest(s.data(),s.size(),bytes,&size,EVP_sha256(),nullptr)==1 && size==32,"checkpoint digest failed");
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for (unsigned i=0;i<size;++i) out<<std::setw(2)<<unsigned(bytes[i]);
  return out.str();
}
std::string read(const std::string &path,size_t maximum) {
  std::ifstream f(path,std::ios::binary);check(bool(f),"cannot read checkpoint identity/file: "+path);
  f.seekg(0,std::ios::end);auto n=f.tellg();
  check(n>=0 && static_cast<uint64_t>(n)<=maximum,"checkpoint/identity file too large");
  f.seekg(0);std::string s(static_cast<size_t>(n),'\0');f.read(s.data(),s.size());
  check(bool(f),"incomplete checkpoint read");return s;
}
std::string fingerprint(const Config &c) {
  std::ostringstream s;s.imbue(std::locale::classic());s<<std::setprecision(17);
  s<<"chess-checkpoint-model-v1\n"<<digest(read(c.urdf,16*1024*1024))<<'\n'
   <<c.ip<<':'<<c.dashboard_port<<':'<<c.feedback_port<<'\n'
   <<c.controller_family<<'\n'<<c.controller_firmware<<'\n'<<c.test_fixture<<'\n';
  for (double q:c.home) s<<q<<',';
  for (int i=0;i<4;++i) for(int j=0;j<4;++j)s<<c.flange_tcp(i,j)<<',';
  s<<c.tracker_options.max_joint_step_rad<<','<<c.tracker_options.max_phase_step_rad;
  return digest(s.str());
}
}
CheckpointStore::CheckpointStore(const Config &c):config_(c),identity_(fingerprint(c)) {
  namespace fs=std::filesystem;
  check(fs::path(c.checkpoint_file).is_absolute(),"checkpoint file must be absolute");
  check(fs::is_directory(fs::path(c.checkpoint_file).parent_path()),"checkpoint parent directory must exist");
  const auto lock=c.checkpoint_file+".lock";
  lock_fd_=::open(lock.c_str(),O_RDWR|O_CREAT|O_NOFOLLOW|O_CLOEXEC,0600);
  check(lock_fd_>=0,"cannot open checkpoint writer lock");
  if (::flock(lock_fd_,LOCK_EX|LOCK_NB)!=0) {::close(lock_fd_);lock_fd_=-1;throw Failure(BUSY,"checkpoint already has a writer");}
}
CheckpointStore::~CheckpointStore() {if(lock_fd_>=0){::flock(lock_fd_,LOCK_UN);::close(lock_fd_);}}
void CheckpointStore::write(const std::string &payload) {
  const auto path=config_.checkpoint_file;
  // Random exclusive names survive reboot/PID reuse with old interrupted temps.
  // mkostemp creates mode0600 with O_EXCL; an existing file/symlink is not followed.
  const auto pattern=path+".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(),pattern.end());temporary.push_back('\0');
  int fd=::mkostemp(temporary.data(),O_CLOEXEC);
  const std::string temp(temporary.data());
  check(fd>=0,"cannot create checkpoint temp");
  bool renamed=false;
  try {
    const std::string content=digest(payload)+"\n"+payload;
    size_t offset=0;
    while(offset<content.size()) {
      auto n=::write(fd,content.data()+offset,content.size()-offset);
      if(n<0 && errno==EINTR)continue;
      check(n>0,"checkpoint write failed");offset+=size_t(n);
    }
    check(::fsync(fd)==0,"checkpoint fsync failed");
    const int close_result=::close(fd);fd=-1;check(close_result==0,"checkpoint close failed");
    check(::rename(temp.c_str(),path.c_str())==0,"checkpoint atomic rename failed");renamed=true;
    const auto parent=std::filesystem::path(path).parent_path().string();
    int dir=::open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);
    check(dir>=0,"checkpoint directory open failed");const int synced=::fsync(dir);::close(dir);
    check(synced==0,"checkpoint directory fsync failed");
  } catch (...) {if(fd>=0)::close(fd);if(!renamed)::unlink(temp.c_str());throw;}
}
void CheckpointStore::save(const Snapshot &s) {
  std::lock_guard<std::mutex> lock(mutex_);
  check(s.fresh && s.reference && (s.feedback.mode==4 || s.feedback.mode==5) &&
        std::all_of(s.feedback.speed.begin(),s.feedback.speed.end(),[this](double v){return std::isfinite(v) && std::abs(v)<=config_.stationary_speed;}),
        "cannot persist unsafe checkpoint");
  auto cp=s.tracker.checkpoint();
  check(jointDistance(cp.joints,s.feedback.q)<=1e-12,"checkpoint tracker/feedback mismatch");
  YAML::Emitter y;y.SetDoublePrecision(17);y<<YAML::BeginMap<<YAML::Key<<"schema"<<YAML::Value<<1
    <<YAML::Key<<"identity"<<YAML::Value<<identity_
    <<YAML::Key<<"restorable"<<YAML::Value<<true
    <<YAML::Key<<"saved_utc_ns"<<YAML::Value<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
    <<YAML::Key<<"controller_stamp"<<YAML::Value<<s.feedback.controller_stamp
    <<YAML::Key<<"joints"<<YAML::Value<<YAML::Flow<<std::vector<double>(cp.joints.begin(),cp.joints.end())
    <<YAML::Key<<"previous_phase"<<YAML::Value<<YAML::Flow<<cp.previous_phase
    <<YAML::Key<<"local"<<YAML::Value<<YAML::Flow<<cp.local<<YAML::EndMap;
  check(y.good(),"checkpoint encoding failed");write(y.c_str());
}
void CheckpointStore::markMotionPending() {
  std::lock_guard<std::mutex> lock(mutex_);
  YAML::Emitter y;y.SetDoublePrecision(17);y<<YAML::BeginMap<<YAML::Key<<"schema"<<YAML::Value<<1
    <<YAML::Key<<"identity"<<YAML::Value<<identity_<<YAML::Key<<"restorable"<<YAML::Value<<false
    <<YAML::Key<<"reason"<<YAML::Value<<"motion pending; wait for confirmed stationary checkpoint"<<YAML::EndMap;
  check(y.good(),"checkpoint marker encoding failed");write(y.c_str());
}
chess_robot_twist::TrackerCheckpoint CheckpointStore::load() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto content=read(config_.checkpoint_file,65536);const auto split=content.find('\n');
  check(split==64 && content.substr(0,64)==digest(content.substr(split+1)),"checkpoint checksum mismatch");
  auto y=YAML::Load(content.substr(split+1));
  check(y["schema"].as<int>()==1 && y["identity"].as<std::string>()==identity_,"checkpoint schema/model/controller mismatch");
  check(y["restorable"].as<bool>(),"checkpoint is dirty: prior motion did not reach a durable stopped save");
  check(y["saved_utc_ns"].as<int64_t>()>0 && y["controller_stamp"].as<uint64_t>()>0,"invalid checkpoint timestamps");
  auto joints=y["joints"].as<std::vector<double>>();check(joints.size()==6,"checkpoint joints dimension");
  chess_robot_twist::TrackerCheckpoint cp;std::copy(joints.begin(),joints.end(),cp.joints.begin());
  cp.previous_phase=y["previous_phase"].as<std::vector<double>>();cp.local=y["local"].as<std::vector<double>>();
  check(cp.previous_phase.size()<=32 && cp.local.size()<=32,"checkpoint phase size out of bounds");return cp;
}
} // namespace chess_robot_arm
