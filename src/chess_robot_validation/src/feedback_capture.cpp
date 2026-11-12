#include "net.hpp"
#include "feedback_frame.hpp"
#include <array>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

class EvidenceFile {
  int fd_ = -1;
public:
  explicit EvidenceFile(const std::string& path) {
    fd_ = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if(fd_ < 0) throw std::runtime_error("cannot create new evidence file: " + path + ": " + std::strerror(errno));
  }
  EvidenceFile(const EvidenceFile&) = delete;
  EvidenceFile& operator=(const EvidenceFile&) = delete;
  ~EvidenceFile() { if(fd_ >= 0) close(fd_); }
  void write(const void* data, size_t n) {
    size_t done = 0;
    while(done < n) {
      const auto result = ::write(fd_, static_cast<const char*>(data) + done, n - done);
      if(result < 0 && errno == EINTR) continue;
      if(result <= 0) throw std::runtime_error("evidence write failed");
      done += static_cast<size_t>(result);
    }
  }
  void write(const std::string& data) { write(data.data(), data.size()); }
  void sync() { if(fsync(fd_) != 0) throw std::runtime_error("evidence fsync failed"); }
};

FeedbackFrame fixture_frame(uint64_t stamp) {
  FeedbackFrame b{};
  const uint16_t len=1440;
  const uint64_t magic=0x0123456789ABCDEFULL;
  std::memcpy(b.data(), &len, sizeof(len));
  std::memcpy(b.data()+48, &magic, sizeof(magic));
  std::memcpy(b.data()+32, &stamp, sizeof(stamp));
  return b;
}

void self_test() {
  FeedbackSequence seq;
  seq.accept(fixture_frame(0));
  seq.accept(fixture_frame(10));
  if(seq.accept(fixture_frame(10))) throw std::runtime_error("identical duplicate not identified");
  for(const uint64_t stamp:{uint64_t(9)}) {
    bool failed=false;try{seq.accept(fixture_frame(stamp));}catch(const std::runtime_error&){failed=true;}
    if(!failed) throw std::runtime_error("duplicate/backwards timestamp accepted");
  }
  auto changed=fixture_frame(10);changed[432]=1;
  bool changed_refused=false;try{seq.accept(changed);}catch(const std::runtime_error&){changed_refused=true;}
  if(!changed_refused) throw std::runtime_error("changed frame with repeated timestamp accepted");
  for(const size_t offset:{size_t(64),size_t(192),size_t(432),size_t(480),size_t(624)}) {
    auto b=fixture_frame(11);const auto bad=std::numeric_limits<double>::infinity();
    std::memcpy(b.data()+offset,&bad,sizeof(bad));
    bool failed=false;try{validate_feedback(b);}catch(const std::runtime_error&){failed=true;}
    if(!failed) throw std::runtime_error("nonfinite field accepted");
  }
  bool failed=false;try{feedback_field<double>(fixture_frame(11),1439);}catch(const std::runtime_error&){failed=true;}
  if(!failed) throw std::runtime_error("out of bounds field accepted");
  std::cout<<"PASS offline: timestamp sequence, nonfinite common fields and bounds; no socket or ROS publication\n";
}

int main(int argc,char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--self-test") { self_test();return 0; }
    if(argc==3 && std::string(argv[1])=="--inspect-raw") {
      std::ifstream input(argv[2],std::ios::binary);
      if(!input) throw std::runtime_error("cannot open raw capture");
      FeedbackSequence sequence;FeedbackFrame frame{};size_t unique=0,duplicates=0;
      while(input.read(reinterpret_cast<char*>(frame.data()),frame.size())) {
        if(sequence.accept(frame)) ++unique; else ++duplicates;
      }
      if(input.bad() || input.gcount()!=0) throw std::runtime_error("raw capture ends with incomplete frame or read error");
      if(unique==0) throw std::runtime_error("empty raw capture");
      std::cout<<"raw_unique_frames="<<unique<<" identical_duplicate_frames="<<duplicates<<"; offline, no socket or ROS publication\n";
      return 0;
    }
    if(argc!=4) throw std::runtime_error("usage: IPv4 seconds output_prefix | --inspect-raw file.bin | --self-test");
    size_t parsed=0;
    const double seconds=std::stod(argv[2],&parsed);
    if(parsed!=std::string(argv[2]).size() || !std::isfinite(seconds) || seconds<=0 || seconds>120)
      throw std::runtime_error("duration must be a finite number in (0,120] seconds");
    const std::string prefix=argv[3];
    EvidenceFile raw(prefix+".bin"),csv(prefix+".csv"),summary(prefix+".summary.txt");
    size_t frames=0,bytes=0,duplicates=0;
    std::vector<double> intervals;
    std::array<double,6> q_min{},q_max{},q_last{};
    uint64_t first_stamp=0,last_stamp=0;
    long long previous_receive=0;
    double max_speed=0;
    const auto started=Steady::now();
    std::string failure;
    try {
      csv.write("receive_utc_ns,receive_steady_ns,controller_timestamp_raw,robot_mode,speed_scaling,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,qd1_deg_s,qd2_deg_s,qd3_deg_s,qd4_deg_s,qd5_deg_s,qd6_deg_s,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg\n");
      Socket socket(argv[1],30004);
      const auto end=Steady::now()+std::chrono::duration<double>(seconds);
      FeedbackSequence sequence;
      while(Steady::now()<end) {
        FeedbackFrame b{};
        const auto frame_deadline=Steady::now()+std::chrono::milliseconds(500);
        size_t received=0;
        while(received<b.size()) {
          const auto remaining=std::chrono::ceil<std::chrono::milliseconds>(frame_deadline-Steady::now()).count();
          if(remaining<=0) throw std::runtime_error("complete frame deadline exceeded");
          const auto n=socket.receive_some(b.data()+received,b.size()-received,static_cast<int>(remaining));
          raw.write(b.data()+received,n);bytes+=n;received+=n;
        }
        const auto utc=utc_ns(),mono=steady_ns();
        if(!sequence.accept(b)) {
          ++duplicates;
          if(mono-previous_receive>500000000LL)
            throw std::runtime_error("only identical duplicate feedback for more than 500 ms");
          if(bytes>32*1024*1024) throw std::runtime_error("capture byte budget exceeded");
          continue;
        }
        const auto stamp=feedback_field<uint64_t>(b,32);
        if(frames==0) first_stamp=stamp;
        if(frames>0) intervals.push_back((mono-previous_receive)/1e6);
        previous_receive=mono;last_stamp=stamp;
        std::ostringstream row;
        row<<std::setprecision(17)<<utc<<','<<mono<<','<<stamp<<','<<feedback_field<uint64_t>(b,24)<<','<<feedback_field<double>(b,64);
        for(size_t j=0;j<6;++j) {
          const auto q=feedback_field<double>(b,432+8*j);
          q_last[j]=q;
          if(frames==0) q_min[j]=q_max[j]=q;
          q_min[j]=std::min(q_min[j],q);q_max[j]=std::max(q_max[j],q);
          row<<','<<q;
        }
        for(size_t j=0;j<6;++j) {
          const auto speed=feedback_field<double>(b,480+8*j);
          max_speed=std::max(max_speed,std::abs(speed));row<<','<<speed;
        }
        for(size_t j=0;j<6;++j) row<<','<<feedback_field<double>(b,624+8*j);
        row<<'\n';csv.write(row.str());++frames;
        if(bytes>32*1024*1024) throw std::runtime_error("capture byte budget exceeded");
      }
      if(frames==0) throw std::runtime_error("no complete frames received");
    } catch(const std::exception& e) { failure=e.what(); }
    std::sort(intervals.begin(),intervals.end());
    std::ostringstream report;
    report<<std::setprecision(17)<<"status="<<(failure.empty()?"CAPTURED":"FAILED")
      <<"\nlayout=vendor_common_1440\nendpoint="<<argv[1]<<":30004\nframes="<<frames<<"\nraw_bytes="<<bytes
      <<"\nidentical_duplicate_frames="<<duplicates
      <<"\nelapsed_s="<<std::chrono::duration<double>(Steady::now()-started).count()
      <<"\ncontroller_timestamp_first_raw="<<first_stamp<<"\ncontroller_timestamp_last_raw="<<last_stamp
      <<"\nmax_abs_joint_speed_deg_s="<<max_speed<<'\n';
    if(!intervals.empty()) {
      const auto p95=static_cast<size_t>(std::ceil(intervals.size()*0.95))-1;
      report<<"receive_dt_median_ms="<<intervals[intervals.size()/2]
        <<"\nreceive_dt_p95_ms="<<intervals[p95]<<"\nreceive_dt_max_ms="<<intervals.back()<<'\n';
    }
    for(size_t j=0;j<6;++j)
      report<<"q"<<j+1<<"_last_deg="<<q_last[j]<<"\nq"<<j+1<<"_span_deg="<<q_max[j]-q_min[j]<<'\n';
    report<<"application_bytes_sent=0\nros_publications=0\n";
    if(!failure.empty()) report<<"error="<<failure<<'\n';
    summary.write(report.str());raw.sync();csv.sync();summary.sync();
    std::cout<<report.str();
    return failure.empty()?0:2;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 2; }
}
