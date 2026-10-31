#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

struct Entry {int id;std::string name;double score,tilt;cv::Point center;std::vector<cv::Point> outline;std::vector<double> xyz;};
class TrackingPreview final:public rclcpp::Node {
 public:
  TrackingPreview():Node("xiangqi_tracking_preview") {
    cv::setNumThreads(1);auto qos=rclcpp::SensorDataQoS().keep_last(1);
    images_=create_publisher<sensor_msgs::msg::Image>("/pose/debug/image",qos);
    metrics_=create_publisher<std_msgs::msg::String>("/pose/xiangqi/preview_metrics",10);
    data_=create_subscription<std_msgs::msg::String>("/pose/xiangqi/detections",10,[this](std_msgs::msg::String::ConstSharedPtr msg){data(msg->data);});
    joints_=create_subscription<sensor_msgs::msg::JointState>("/arm/joint_states",rclcpp::QoS(5),[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){joint_=*msg;});
    source_=create_subscription<sensor_msgs::msg::Image>("/vision/color/image_rect",qos,[this](sensor_msgs::msg::Image::ConstSharedPtr msg){image(msg);});
  }
 private:
  void data(const std::string &s) {
    try {
      const auto d=YAML::Load(s);const int64_t next=d["stamp_ns"].as<int64_t>();
      if(next<=data_stamp_) return;
      data_stamp_=next;entries_.clear();source_q_.clear();
      if(!d["source_joint_position_rad"] || !d["base_transform_valid"].as<bool>(false)) return;
      source_q_=d["source_joint_position_rad"].as<std::vector<double>>();
      if(source_q_.size()!=6) {source_q_.clear();return;}
      for(const auto &v:d["detections"]) {
        if(!v["valid"].as<bool>(false)||!v["track_confirmed"].as<bool>(false)||!v["complete_center_target"].as<bool>(false))continue;
        Entry e;e.id=v["track_id"].as<int>();e.name=v["track_class_name"].as<std::string>();e.score=v["track_class_vote_fraction"].as<double>();e.tilt=v["surface_tilt_deg"].as<double>();
        auto uv=v["center_pixel"].as<std::vector<double>>();if(uv.size()!=2)continue;e.center={int(std::lround(uv[0])),int(std::lround(uv[1]))};
        e.xyz=v["base_surface_center_m"].as<std::vector<double>>();if(e.xyz.size()!=3)continue;
        for(const auto &point:v["mask_outline_xy"]) {auto xy=point.as<std::vector<int>>();if(xy.size()==2)e.outline.emplace_back(xy[0],xy[1]);}
        entries_.push_back(std::move(e));
      }
      std::sort(entries_.begin(),entries_.end(),[](const Entry&a,const Entry&b){return a.id<b.id;});
      if(!entries_.empty())++rate_data_frames_;
    } catch(const std::exception &e) {entries_.clear();source_q_.clear();RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Track JSON rejected: %s",e.what());}
  }
  void image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();const int64_t current=now().nanoseconds();
    if(msg->encoding!="bgr8"||msg->header.frame_id!="d405_color_optical_frame"||current<stamp||(current-stamp)*1e-9>.3)return;
    if(rate_window_start_==0)rate_window_start_=current;
    ++rate_video_frames_;
    last_image_=stamp;
    auto image=cv_bridge::toCvCopy(msg,"bgr8");auto &bgr=image->image;
    const double age=(current-data_stamp_)*1e-9;
    bool valid=data_stamp_>0&&age>=0&&age<=.5&&source_q_.size()==6&&joint_.position.size()==6&&joint_.velocity.size()==6;
    if(valid) {
      double joint_age=(current-rclcpp::Time(joint_.header.stamp).nanoseconds())*1e-9;
      valid=joint_age>=0&&joint_age<=.25;
      for(size_t i=0;i<6;++i)valid=valid&&std::isfinite(joint_.position[i])&&std::isfinite(joint_.velocity[i])&&std::abs(joint_.velocity[i])<=.002&&std::abs(joint_.position[i]-source_q_[i])<=.001;
    }
    cv::rectangle(bgr,{0,0},{bgr.cols,49},{0,0,0},-1);
    std::ostringstream title;title<<std::fixed<<std::setprecision(1)<<"VIDEO "<<video_hz_<<" Hz / "<<bgr.cols<<"x"<<bgr.rows<<" | recognition "<<data_hz_<<" Hz";
    cv::putText(bgr,title.str(),{10,20},cv::FONT_HERSHEY_SIMPLEX,.52,{0,255,255},1);
    std::ostringstream description;description<<"Persistent piece IDs | surface XYZ + tilt | yaw unobserved | ";
    if(valid)description<<"measurement age "<<std::fixed<<std::setprecision(2)<<age<<" s";else description<<"waiting for fresh stopped observation";
    cv::putText(bgr,description.str(),{10,41},cv::FONT_HERSHEY_SIMPLEX,.44,{0,255,255},1);
    if(valid) {
      static const cv::Scalar colors[]={{0,255,0},{255,200,0},{0,220,255},{255,0,200},{200,255,200},{255,150,100}};
      int row=0;
      for(const auto &e:entries_) {
        auto color=colors[(e.id-1)%6];
        if(e.outline.size()>2)cv::polylines(bgr,std::vector<std::vector<cv::Point>>{e.outline},true,color,2);
        cv::drawMarker(bgr,e.center,color,cv::MARKER_CROSS,17,2);
        const std::string id="#"+std::to_string(e.id);cv::Point at=e.center+cv::Point(12,-12);
        cv::putText(bgr,id,at,cv::FONT_HERSHEY_SIMPLEX,.8,{0,0,0},5);cv::putText(bgr,id,at,cv::FONT_HERSHEY_SIMPLEX,.8,color,2);
        if(row<12) {
          std::ostringstream text;text<<id<<" pred:"<<e.name<<"  XYZ(mm) "<<std::fixed<<std::setprecision(1)<<e.xyz[0]*1000<<", "<<e.xyz[1]*1000<<", "<<e.xyz[2]*1000<<"  tilt_est "<<e.tilt<<" deg";
          cv::Point text_at(10,72+row*25);cv::putText(bgr,text.str(),text_at,cv::FONT_HERSHEY_SIMPLEX,.42,{0,0,0},4);cv::putText(bgr,text.str(),text_at,cv::FONT_HERSHEY_SIMPLEX,.42,color,1);++row;
        }
      }
    }
    images_->publish(*image->toImageMsg());++published_;
    if(current-last_metrics_>2000000000LL) {
      const double duration=(current-rate_window_start_)*1e-9;
      if(duration>=1.) {video_hz_=rate_video_frames_/duration;data_hz_=rate_data_frames_/duration;rate_window_start_=current;rate_video_frames_=0;rate_data_frames_=0;}
      std::ostringstream json;json<<"{\"stamp_ns\":"<<stamp<<",\"video_hz\":"<<video_hz_<<",\"recognition_hz\":"<<data_hz_<<",\"published_frames\":"<<published_<<",\"overlay_valid\":"<<(valid?"true":"false")<<",\"visible_tracks\":"<<(valid?entries_.size():0)<<"}";
      std_msgs::msg::String value;value.data=json.str();metrics_->publish(value);last_metrics_=current;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr source_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_;
  sensor_msgs::msg::JointState joint_;std::vector<double> source_q_;std::vector<Entry> entries_;
  int64_t data_stamp_{0},last_image_{0},last_metrics_{0},rate_window_start_{0};uint64_t published_{0},rate_video_frames_{0},rate_data_frames_{0};double video_hz_{0},data_hz_{0};
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<TrackingPreview>());rclcpp::shutdown();}
