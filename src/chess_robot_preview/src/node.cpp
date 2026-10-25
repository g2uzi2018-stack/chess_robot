#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/asio.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <sys/socket.h>
#include <poll.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cerrno>
#include <condition_variable>

namespace net=boost::asio;
namespace http=boost::beast::http;
using tcp=net::ip::tcp;
using Clock=std::chrono::steady_clock;
using Bytes=std::vector<uint8_t>;
using Response=http::response<http::vector_body<uint8_t>>;
static double seconds(Clock::time_point a,Clock::time_point b) { return std::chrono::duration<double>(a-b).count(); }
static std::string json_quote(const std::string &s) {
  std::ostringstream o; o<<'"';
  for(unsigned char c:s) {
    if(c=='"'||c=='\\') o<<'\\'<<c;
    else if(c<32) o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else o<<c;
  }
  return o.str()+'"';
}
static std::map<std::string,std::string> query(const std::string &target) {
  std::map<std::string,std::string> out;
  auto p=target.find('?'); if(p==std::string::npos) return out;
  std::istringstream in(target.substr(p+1)); std::string part;
  while(std::getline(in,part,'&')) { auto eq=part.find('='); if(eq!=std::string::npos) out[part.substr(0,eq)]=part.substr(eq+1); }
  return out;
}
struct Channel {
  std::string id,title,description,topic,type,mode,error;
  rclcpp::SubscriptionBase::SharedPtr subscription;
  Bytes jpeg; Clock::time_point received{},next_encode{};
  uint64_t sequence{0},received_count{0},encoded_count{0},sent_bytes{0};
  int width{0},height{0}; bool encoding{false};
  double source_fps{0};
};
struct Viewer { std::string channel; Clock::time_point touched; };

class PreviewNode : public rclcpp::Node {
 public:
  PreviewNode():Node("preview_node"),workers_(8) {
    const auto share=ament_index_cpp::get_package_share_directory("chess_robot_preview");
    const auto config_file=declare_parameter<std::string>("config_file",share+"/config/channels.yaml");
    const auto config=YAML::LoadFile(config_file),p=config["preview"];
    bind_=p["bind_address"].as<std::string>(); port_=p["port"].as<int>();
    fps_=p["fps"].as<double>(); max_width_=p["max_width"].as<int>(); quality_=p["jpeg_quality"].as<int>();
    idle_s_=p["idle_timeout_s"].as<double>(); stale_s_=p["stale_timeout_s"].as<double>();
    min_depth_=p["depth_min_m"].as<double>(); max_depth_=p["depth_max_m"].as<double>();
    if(port_<1||port_>65535||!std::isfinite(fps_)||fps_<0.5||fps_>30||max_width_<160||max_width_>1920||
       quality_<20||quality_>95||!std::isfinite(idle_s_)||idle_s_<0.5||idle_s_>10||
       !std::isfinite(stale_s_)||stale_s_<0.1||!std::isfinite(min_depth_)||!std::isfinite(max_depth_)||
       min_depth_<=0||max_depth_<=min_depth_) throw std::runtime_error("invalid preview configuration");
    for(const auto &entry:config["channels"]) {
      auto c=std::make_shared<Channel>();
      c->id=entry["id"].as<std::string>(); c->title=entry["title"].as<std::string>();
      c->description=entry["description"].as<std::string>(""); c->topic=entry["topic"].as<std::string>();
      c->type=entry["type"].as<std::string>(); c->mode=entry["mode"].as<std::string>("color");
      c->source_fps=entry["source_fps"].as<double>(0.0);
      if(!std::regex_match(c->id,std::regex("[a-zA-Z0-9_-]{1,48}"))||by_id_.count(c->id)||
         (c->type!="image"&&c->type!="compressed")||(c->mode!="color"&&c->mode!="depth")||
         (c->type=="compressed"&&c->mode=="depth")||c->topic.empty()||
         !std::isfinite(c->source_fps)||c->source_fps<0||c->source_fps>240)
        throw std::runtime_error("invalid or duplicate channel");
      channels_.push_back(c); by_id_[c->id]=c;
    }
    if(channels_.empty()||channels_.size()>64) throw std::runtime_error("need 1-64 channels");
    for(const auto &file:{"index.html","app.js","style.css"}) {
      std::ifstream in(share+"/web/"+file,std::ios::binary);
      if(!in) throw std::runtime_error("missing web asset");
      assets_[std::string(file)]=Bytes(std::istreambuf_iterator<char>(in),{});
    }
    cv::setNumThreads(1);
    parameter_callback_=add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult r; r.successful=false; r.reason="restart to change preview configuration"; return r;
    });
    acceptor_=std::make_unique<tcp::acceptor>(io_);
    const tcp::endpoint endpoint(net::ip::make_address(bind_),static_cast<unsigned short>(port_));
    acceptor_->open(endpoint.protocol()); acceptor_->set_option(tcp::acceptor::reuse_address(true));
    acceptor_->bind(endpoint); acceptor_->listen(16); acceptor_->non_blocking(true);
    timer_=create_wall_timer(std::chrono::milliseconds(200),[this] {
      std::lock_guard<std::mutex> lock(mutex_); reconcile();
    });
    server_=std::thread([this] { serve(); });
    RCLCPP_INFO(get_logger(),"Preview http://%s:%d : %.1f fps, max width %d; no image subscriptions until selected",bind_.c_str(),port_,fps_,max_width_);
  }
  ~PreviewNode() override {
    stopping_=true;
    frame_ready_.notify_all();
    if(server_.joinable()) server_.join();
    boost::system::error_code ec; if(acceptor_) acceptor_->close(ec);
    workers_.join();
    for(auto &c:channels_) c->subscription.reset();
  }
 private:
  void reconcile() { // mutex held; at most one selected channel per browser token.
    const auto now=Clock::now();
    for(auto it=viewers_.begin();it!=viewers_.end();) {
      if(seconds(now,it->second.touched)>idle_s_) it=viewers_.erase(it); else ++it;
    }
    for(auto &c:channels_) {
      const bool wanted=std::any_of(viewers_.begin(),viewers_.end(),[&](const auto &v) { return v.second.channel==c->id; });
      if(wanted&&!c->subscription) {
        auto qos=rclcpp::SensorDataQoS().keep_last(2);
        if(c->type=="compressed") c->subscription=create_subscription<sensor_msgs::msg::CompressedImage>(c->topic,qos,
          [this,c](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {
            if(!begin_frame(c)) return;
            try {
              if(m->data.size()>16*1024*1024) throw std::runtime_error("compressed frame too large");
              finish_frame(c,cv::imdecode(m->data,cv::IMREAD_COLOR),m->header.stamp);
            } catch(const std::exception &e) { frame_error(c,e.what()); }
          });
        else c->subscription=create_subscription<sensor_msgs::msg::Image>(c->topic,qos,
          [this,c](sensor_msgs::msg::Image::ConstSharedPtr m) {
            if(!begin_frame(c)) return;
            try {
              if(m->width>8192||m->height>8192||m->data.size()>128*1024*1024) throw std::runtime_error("image too large");
              if(c->mode=="depth") {
                if(m->encoding!="32FC1") throw std::runtime_error("depth channel requires 32FC1 metres");
                const cv::Mat depth=cv_bridge::toCvShare(m,"32FC1")->image;
                cv::Mat gray,color,valid=(depth>=min_depth_)&(depth<=max_depth_);
                depth.convertTo(gray,CV_8U,-255.0/(max_depth_-min_depth_),255.0*max_depth_/(max_depth_-min_depth_));
                cv::applyColorMap(gray,color,cv::COLORMAP_TURBO); color.setTo(cv::Scalar(0,0,0),~valid);
                finish_frame(c,color,m->header.stamp);
              } else finish_frame(c,cv_bridge::toCvShare(m,"bgr8")->image,m->header.stamp);
            } catch(const std::exception &e) { frame_error(c,e.what()); }
          });
      } else if(!wanted&&c->subscription) {
        c->subscription.reset(); c->jpeg.clear(); c->error.clear(); c->received={}; c->next_encode={};
      }
    }
  }
  bool begin_frame(const std::shared_ptr<Channel> &c) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++c->received_count;
    if(!c->subscription||c->encoding) return false;
    // A configured source already at or below the preview rate must not be throttled twice.
    // Its callback delivery may arrive in bursts even though the camera captures at 30 Hz.
    if(c->source_fps>0 && c->source_fps<=fps_) { c->encoding=true; return true; }
    const auto current=Clock::now();
    const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/fps_));
    // Keep a phase-based deadline: small source jitter must not halve a 30 Hz stream.
    if(c->next_encode!=Clock::time_point{} && current+period/3<c->next_encode) return false;
    if(c->next_encode==Clock::time_point{} || current-c->next_encode>period) c->next_encode=current+period;
    else c->next_encode+=period;
    c->encoding=true; return true;
  }
  void finish_frame(const std::shared_ptr<Channel> &c,cv::Mat image,const builtin_interfaces::msg::Time &stamp) {
    if(image.empty()) throw std::runtime_error("empty image");
    if(stamp.sec!=0||stamp.nanosec!=0) {
      const double age=(now().nanoseconds()-rclcpp::Time(stamp).nanoseconds())*1e-9;
      if(age>stale_s_||age< -0.5) throw std::runtime_error("source timestamp stale or unsynchronized");
    }
    const bool resize=image.cols>max_width_;
    if(resize) cv::resize(image,image,{max_width_,std::max(1,image.rows*max_width_/image.cols)},0,0,cv::INTER_AREA);
    Bytes jpeg;
    if(!cv::imencode(".jpg",image,jpeg,{cv::IMWRITE_JPEG_QUALITY,quality_})) throw std::runtime_error("JPEG encode failed");
    std::lock_guard<std::mutex> lock(mutex_);
    c->encoding=false;
    if(!c->subscription) return;
    c->jpeg=std::move(jpeg); c->received=Clock::now(); ++c->sequence; ++c->encoded_count;
    c->width=image.cols; c->height=image.rows; c->error.clear();
    frame_ready_.notify_all();
  }
  void frame_error(const std::shared_ptr<Channel> &c,const std::string &error) {
    std::lock_guard<std::mutex> lock(mutex_); c->encoding=false; c->error=error; c->jpeg.clear(); frame_ready_.notify_all();
  }
  std::string metadata() {
    std::lock_guard<std::mutex> lock(mutex_); reconcile();
    std::ostringstream o; o<<"{\"fps\":"<<fps_<<",\"max_width\":"<<max_width_<<",\"viewers\":"<<viewers_.size()<<",\"channels\":[";
    bool first=true;
    for(const auto &c:channels_) {
      if(!first) o<<',';
      first=false;
      o<<"{\"id\":"<<json_quote(c->id)<<",\"title\":"<<json_quote(c->title)<<",\"description\":"<<json_quote(c->description)
       <<",\"subscribed\":"<<(c->subscription?"true":"false")
       <<",\"has_publisher\":"<<(count_publishers(c->topic)>0?"true":"false")
       <<",\"received_frames\":"<<c->received_count<<",\"encoded_frames\":"<<c->encoded_count
       <<",\"sent_bytes\":"<<c->sent_bytes<<",\"width\":"<<c->width<<",\"height\":"<<c->height<<'}';
    }
    return o.str()+"]}";
  }
  Response response(http::status status,const std::string &type,Bytes data={}) {
    Response r{status,11}; r.set(http::field::content_type,type); r.set(http::field::cache_control,"no-store");
    r.set("X-Content-Type-Options","nosniff"); r.set("Referrer-Policy","no-referrer");
    r.set("Content-Security-Policy","default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; frame-ancestors 'none'");
    r.keep_alive(false); r.body()=std::move(data); r.prepare_payload(); return r;
  }
  Response text_response(http::status s,const std::string &text) { return response(s,"application/json; charset=utf-8",Bytes(text.begin(),text.end())); }
  Response route(const http::request<http::string_body> &request) {
    const std::string target(request.target()); const auto args=query(target);
    const auto path=target.substr(0,target.find('?'));
    if(request.method()==http::verb::get) {
      const auto asset=path=="/"?"index.html":path.substr(1);
      auto found=assets_.find(asset);
      if(found!=assets_.end()) return response(http::status::ok,asset=="index.html"?"text/html; charset=utf-8":asset=="app.js"?"text/javascript; charset=utf-8":"text/css; charset=utf-8",found->second);
      if(path=="/api/channels") return text_response(http::status::ok,metadata());
      if(path.rfind("/api/frame/",0)==0) {
        const auto token=args.find("client"); const auto id=path.substr(11);
        if(token==args.end()||!std::regex_match(token->second,std::regex("[a-zA-Z0-9_-]{1,64}"))) return text_response(http::status::bad_request,"{\"error\":\"invalid client\"}");
        std::unique_lock<std::mutex> lock(mutex_); reconcile();
        auto found_channel=by_id_.find(id);
        if(found_channel==by_id_.end()) return text_response(http::status::not_found,"{}");
        if(!viewers_.count(token->second)&&viewers_.size()>=16) return text_response(http::status::too_many_requests,"{}");
        viewers_[token->second]={id,Clock::now()}; reconcile(); auto &c=found_channel->second;
        auto after=args.find("after");
        if(after!=args.end() && after->second==std::to_string(c->sequence) && !c->jpeg.empty()) {
          // Wait briefly for a NEW frame instead of sampling the same cache on two independent clocks.
          // The lock is released during this wait so ROS callbacks can publish the next JPEG.
          frame_ready_.wait_for(lock,std::chrono::milliseconds(50),[&] {
            return stopping_||!c->subscription||c->jpeg.empty()||after->second!=std::to_string(c->sequence);
          });
        }
        if(!c->subscription||c->jpeg.empty()||seconds(Clock::now(),c->received)>stale_s_)
          return text_response(http::status::accepted,"{\"status\":\"no_data\"}");
        if(after!=args.end()&&after->second==std::to_string(c->sequence)) return response(http::status::no_content,"image/jpeg");
        auto r=response(http::status::ok,"image/jpeg",c->jpeg);
        r.set("X-Frame-Sequence",std::to_string(c->sequence));
        c->sent_bytes+=c->jpeg.size(); return r;
      }
    }
    if(path=="/api/release"&&request.method()==http::verb::post) {
      auto token=args.find("client");
      std::lock_guard<std::mutex> lock(mutex_); if(token!=args.end()) viewers_.erase(token->second); reconcile();
      return text_response(http::status::ok,"{}");
    }
    return text_response(http::status::not_found,"{}");
  }
  void serve() {
    while(!stopping_) {
      tcp::socket socket(io_); boost::system::error_code ec; acceptor_->accept(socket,ec);
      if(ec) {
        pollfd incoming{acceptor_->native_handle(),POLLIN,0};
        ::poll(&incoming,1,100);
        continue;
      }
      if(active_requests_.fetch_add(1)>=8) { --active_requests_; socket.close(ec); continue; }
      net::post(workers_,[this,socket=std::move(socket)]() mutable {
        try {
          timeval timeout{2,0};
          setsockopt(socket.native_handle(),SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
          setsockopt(socket.native_handle(),SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
          http::request_parser<http::string_body> parser; parser.header_limit(8192); parser.body_limit(256);
          parser.skip(true);
          std::string header; char chunk[2048]; const auto began=Clock::now();
          while(header.find("\r\n\r\n")==std::string::npos) {
            if(seconds(Clock::now(),began)>2) throw std::runtime_error("HTTP header timeout");
            const auto n=::recv(socket.native_handle(),chunk,sizeof(chunk),0);
            if(n<=0) throw std::runtime_error("HTTP connection closed or timed out");
            header.append(chunk,n); if(header.size()>8192) throw std::runtime_error("HTTP header too large");
          }
          boost::system::error_code parse_error;
          parser.put(net::buffer(header),parse_error);
          if(parse_error||!parser.is_header_done()) throw std::runtime_error("invalid HTTP header");
          auto reply=route(parser.get()); std::ostringstream wire; wire<<reply; const auto payload=wire.str();
          size_t sent=0; const auto send_started=Clock::now();
          while(sent<payload.size()&&seconds(Clock::now(),send_started)<2) {
            const auto n=::send(socket.native_handle(),payload.data()+sent,payload.size()-sent,MSG_NOSIGNAL);
            if(n<0&&errno==EINTR) continue;
            if(n<=0) break;
            sent+=static_cast<size_t>(n);
          }
        } catch(const std::exception &) { /* disconnected/invalid HTTP client; no node termination */ }
        boost::system::error_code ec; socket.shutdown(tcp::socket::shutdown_both,ec); socket.close(ec); --active_requests_;
      });
    }
  }
  net::io_context io_; net::thread_pool workers_; std::unique_ptr<tcp::acceptor> acceptor_; std::thread server_;
  std::atomic<bool> stopping_{false}; std::atomic<int> active_requests_{0};
  std::mutex mutex_; std::condition_variable frame_ready_; std::vector<std::shared_ptr<Channel>> channels_;
  std::map<std::string,std::shared_ptr<Channel>> by_id_; std::map<std::string,Viewer> viewers_;
  std::map<std::string,Bytes> assets_;
  std::string bind_; int port_,max_width_,quality_; double fps_,idle_s_,stale_s_,min_depth_,max_depth_;
  rclcpp::TimerBase::SharedPtr timer_; rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<PreviewNode>()); }
  catch(const std::exception &e) { std::cerr<<"preview_node: "<<e.what()<<std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
