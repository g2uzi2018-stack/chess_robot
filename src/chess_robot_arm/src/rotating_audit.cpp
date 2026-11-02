#include "chess_robot_arm/rotating_audit.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <charconv>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include "chess_robot_arm/core.hpp"

namespace chess_robot_arm {
AuditDirectory::AuditDirectory(const std::string &path) {
  std::filesystem::create_directories(path);
  fd_=::open((path+"/.writer.lock").c_str(),O_RDWR|O_CREAT|O_NOFOLLOW|O_CLOEXEC,0600);
  if(fd_<0)throw Failure(INVALID_GOAL,"cannot open audit directory lock");
  if(::flock(fd_,LOCK_EX|LOCK_NB)!=0){::close(fd_);fd_=-1;throw Failure(BUSY,"audit directory already has a writer");}
}
AuditDirectory::~AuditDirectory(){if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);}}
RotatingAudit::RotatingAudit(std::string path,size_t maximum,size_t keep,std::string header)
    :path_(std::move(path)),header_(std::move(header)),maximum_(maximum),keep_(keep) {
  if(keep<1 || keep>128 || maximum<1448 || maximum>1024ULL*1024*1024 || header_.size()>=maximum)
    throw std::runtime_error("invalid rotating audit bounds");
  worker_=std::thread([this]{run();});
}
RotatingAudit::~RotatingAudit() {
  {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
  cv_.notify_one();if(worker_.joinable())worker_.join();
}
bool RotatingAudit::write(std::string record) noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopping_ || !error_.empty() || record.empty() || record.size()+header_.size()>maximum_ ||
        queued_bytes_+record.size()>8*1024*1024 || queue_.size()>=8192) {++dropped_;return false;}
    queued_bytes_+=record.size();queue_.push_back(std::move(record));cv_.notify_one();return true;
  } catch (...) {++dropped_;return false;}
}
std::string RotatingAudit::error() {std::lock_guard<std::mutex> lock(mutex_);return error_;}
void RotatingAudit::run() {
  namespace fs=std::filesystem;
  std::ofstream out;
  size_t bytes=0;
  auto status=[&] {
    std::ofstream meta(path_+".status",std::ios::trunc);
    meta<<"generation="<<generation_<<"\nwritten_records="<<written_<<"\ndropped_records="<<dropped_
        <<"\nmax_file_bytes="<<maximum_<<"\nkeep_files="<<keep_<<"\nerror="<<error()<<'\n';
  };
  auto open=[&] {
    out.open(path_,std::ios::binary|std::ios::trunc);
    if(!out)throw std::runtime_error("cannot open rotating audit");
    out.write(header_.data(),header_.size());bytes=header_.size();
  };
  auto rotate=[&] {
    if(keep_==1)fs::remove(path_);
    else {
      fs::remove(path_+"."+std::to_string(keep_-1));
      for(size_t i=keep_-1;i>1;--i) {
        auto from=path_+"."+std::to_string(i-1);
        if(fs::exists(from))fs::rename(from,path_+"."+std::to_string(i));
      }
      fs::rename(path_,path_+".1");
    }
    ++generation_;
  };
  try {
    // Retention may have been reduced since the previous boot. Only canonical
    // numbered regular-file archives belong to this stream; preserve other files.
    auto parent=fs::path(path_).parent_path();if(parent.empty())parent=".";
    const auto prefix=fs::path(path_).filename().string()+".";
    for(const auto &entry:fs::directory_iterator(parent)) {
      if(!fs::is_regular_file(entry.symlink_status()))continue;
      const auto name=entry.path().filename().string();
      if(name.compare(0,prefix.size(),prefix)!=0)continue;
      const auto suffix=name.substr(prefix.size());uint64_t index=0;
      auto parsed=std::from_chars(suffix.data(),suffix.data()+suffix.size(),index);
      if(parsed.ec==std::errc{} && parsed.ptr==suffix.data()+suffix.size() && index>=keep_ &&
          suffix==std::to_string(index))fs::remove(entry.path());
    }
    if(fs::exists(path_))rotate();
    open();
    for(;;) {
      std::string record;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock,std::chrono::milliseconds(250),[&]{return stopping_ || !queue_.empty();});
        if(queue_.empty()) {
          if(stopping_)break;
          lock.unlock();out.flush();status();continue;
        }
        record=std::move(queue_.front());queue_.pop_front();queued_bytes_-=record.size();
      }
      if(bytes+record.size()>maximum_) {
        out.flush();out.close();
        rotate();open();status();
      }
      out.write(record.data(),record.size());
      if(!out)throw std::runtime_error("rotating audit write failed");
      bytes+=record.size();++written_;
    }
    out.flush();if(!out)throw std::runtime_error("rotating audit flush failed");
  } catch(const std::exception &e) {
    std::lock_guard<std::mutex> lock(mutex_);error_=e.what();dropped_+=queue_.size()+1;queue_.clear();queued_bytes_=0;
  }
  status();
}
} // namespace chess_robot_arm
