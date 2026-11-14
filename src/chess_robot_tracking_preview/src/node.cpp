#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/freetype.hpp>
#include <json/json.h>
#include <memory>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>
#include <map>

struct Entry {int id;std::string name,reason;double score,tilt;cv::Point center;std::vector<cv::Point> outline;std::vector<double> xyz;bool base=false,classified=false,camera_geometry=false,base_geometry=false,complete=false,confirmed=false,edge=false;int hits=0;std::vector<double> camera;};
class TrackingPreview final:public rclcpp::Node {
 public:
  TrackingPreview():Node("xiangqi_tracking_preview") {
    cv::setNumThreads(1);
    font_=cv::freetype::createFreeType2();
    font_->loadFontData(declare_parameter<std::string>("font_path","/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),0);
    const std::map<std::string,std::string> names={
      {"red_pawn","兵"},{"red_king","帅"},{"red_advisor","仕"},{"red_elephant","相"},
      {"red_rook","车"},{"red_horse","马"},{"red_cannon","炮"},
      {"black_pawn","卒"},{"black_king","将"},{"black_advisor","士"},{"black_elephant","象"},
      {"black_rook","车"},{"black_horse","马"},{"black_cannon","炮"}};
    for(const auto &kv:names) {
      cv::Mat tile(44,44,CV_8UC3,cv::Scalar(245,245,245));
      font_->putText(tile,kv.second,{6,36},32,kv.first.rfind("red_",0)==0?cv::Scalar(30,30,220):cv::Scalar(25,25,25),-1,cv::LINE_AA,true);
      badges_[kv.first]=tile;
    }
    cv::Mat unknown(44,64,CV_8UC3,cv::Scalar(30,30,30));
    font_->putText(unknown,"待认",{4,34},27,{0,220,255},-1,cv::LINE_AA,true);
    badges_["unknown"]=unknown;
    for(const auto &text:{"深度待定","不完整"}) {
      cv::Mat tile(25,80,CV_8UC3,cv::Scalar(30,30,30));
      font_->putText(tile,text,{2,21},18,{0,220,255},-1,cv::LINE_AA,true);
      badges_[text]=tile;
    }
    auto qos=rclcpp::SensorDataQoS().keep_last(1);
    images_=create_publisher<sensor_msgs::msg::Image>("/pose/debug/image",qos);
    metrics_=create_publisher<std_msgs::msg::String>("/pose/xiangqi/preview_metrics",10);
    data_=create_subscription<std_msgs::msg::String>("/pose/xiangqi/detections",10,[this](std_msgs::msg::String::ConstSharedPtr msg){data(msg->data);});
    joints_=create_subscription<sensor_msgs::msg::JointState>("/arm/joint_states",rclcpp::QoS(5),[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){joint_=*msg;});
    source_=create_subscription<sensor_msgs::msg::Image>("/vision/color/image_rect",qos,[this](sensor_msgs::msg::Image::ConstSharedPtr msg){image(msg);});
    auto durable=rclcpp::QoS(1).transient_local();
    status_=create_subscription<std_msgs::msg::String>("/pose/xiangqi/observation/status",durable,[this](std_msgs::msg::String::ConstSharedPtr msg){
      try {auto s=YAML::Load(msg->data);observation_state_=s["state"].as<std::string>();observation_id_=s["snapshot_id"].IsNull()?"":s["snapshot_id"].as<std::string>();observation_elapsed_=s["sample_elapsed_s"].as<double>(0);observation_eligible_=s["observation_eligible"].as<bool>(false);status_received_=now().nanoseconds();}catch(const std::exception&){observation_state_="waiting";}
    });
    snapshot_=create_subscription<std_msgs::msg::String>("/pose/xiangqi/observation/snapshot",durable,[this](std_msgs::msg::String::ConstSharedPtr msg){
      try {auto s=YAML::Load(msg->data);snapshot_id_=s["snapshot_id"].as<std::string>();locked_xyz_.clear();for(const auto &t:s["targets"])locked_xyz_[t["track_id"].as<int>()]=t["base_surface_center_m"].as<std::vector<double>>();}catch(const std::exception&){locked_xyz_.clear();}
    });
  }
 private:
  void data(const std::string &s) {
    try {
      Json::CharReaderBuilder builder;
      std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
      Json::Value d;std::string error;
      if(!reader->parse(s.data(),s.data()+s.size(),&d,&error))throw std::runtime_error(error);
      const int64_t next=d["stamp_ns"].asInt64();
      if(next<=data_stamp_)return;
      data_stamp_=next;entries_.clear();source_q_.clear();source_names_.clear();
      auto doubles=[](const Json::Value &a){std::vector<double> out;for(const auto &v:a)out.push_back(v.asDouble());return out;};
      if(d["base_transform_valid"].asBool())source_q_=doubles(d["source_joint_position_rad"]);
      if(source_q_.size()!=6)source_q_.clear();
      if(d.isMember("source_joint_names"))for(const auto &v:d["source_joint_names"])source_names_.push_back(v.asString());
      else source_names_={"joint1","joint2","joint3","joint4","joint5","joint6"};
      for(const auto &v:d["detections"]) {
        if(!v.get("detection_valid",v.get("valid",false)).asBool())continue;
        Entry e;e.base=v["valid"].asBool()&&v["track_confirmed"].asBool()&&v["complete_center_target"].asBool();
        e.classified=v["classification_valid"].asBool();e.camera_geometry=v["camera_geometry_valid"].asBool();
        e.base_geometry=v["base_geometry_valid"].asBool();e.complete=v["complete_center_target"].asBool();
        e.confirmed=v["track_confirmed"].asBool();e.hits=v["track_hits"].asInt();
        e.edge=v["touches_image_edge"].asBool()||v["touches_processing_edge"].asBool();
        e.id=e.base?v["track_id"].asInt():0;
        e.name=v.get("class_name",v.get("track_class_name","unknown")).asString();
        e.score=v.get("confidence",v.get("track_class_vote_fraction",0)).asDouble();e.tilt=v["surface_tilt_deg"].asDouble();
        std::vector<double> uv;
        if(v.isMember("center_pixel"))uv=doubles(v["center_pixel"]);
        else if(v.isMember("box_xyxy")){auto b=doubles(v["box_xyxy"]);if(b.size()==4)uv={(b[0]+b[2])/2,(b[1]+b[3])/2};}
        if(uv.size()!=2||!std::isfinite(uv[0])||!std::isfinite(uv[1]))continue;
        e.center={int(std::lround(uv[0])),int(std::lround(uv[1]))};
        if(e.base)e.xyz=doubles(v["base_surface_center_m"]);
        if(e.xyz.size()!=3)e.base=false;
        if(e.camera_geometry)e.camera=doubles(v["camera_surface_center_m"]);
        for(const auto &point:v["mask_outline_xy"])if(point.size()==2)e.outline.emplace_back(point[0].asInt(),point[1].asInt());
        entries_.push_back(std::move(e));
      }
      std::sort(entries_.begin(),entries_.end(),[](const Entry&a,const Entry&b){return a.id<b.id;});
      ++rate_data_frames_;
    } catch(const std::exception &e) {entries_.clear();source_q_.clear();source_names_.clear();RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Track JSON rejected: %s",e.what());}
  }
  void image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();const int64_t current=now().nanoseconds();
    if(msg->encoding!="bgr8"||msg->header.frame_id!="d405_color_optical_frame"||current<stamp||(current-stamp)*1e-9>.3)return;
    if(rate_window_start_==0)rate_window_start_=current;
    ++rate_video_frames_;
    last_image_=stamp;
    auto image=cv_bridge::toCvCopy(msg,"bgr8");auto &bgr=image->image;
    const double age=(current-data_stamp_)*1e-9;
    const bool camera_valid=data_stamp_>0&&age>=0&&age<=.5&&stamp>=data_stamp_&&(stamp-data_stamp_)*1e-9<=.5;
    bool valid=camera_valid&&source_q_.size()==6&&joint_.position.size()==6&&joint_.velocity.size()==6;
    std::string render_reason=!camera_valid?"image stale":(source_q_.size()!=6?"base unavailable":"joint unavailable");
    if(valid) {
      double joint_age=(current-rclcpp::Time(joint_.header.stamp).nanoseconds())*1e-9;
      valid=joint_age>=0&&joint_age<=.25&&joint_.name==source_names_;
      render_reason=joint_age<0||joint_age>.25?"joint stale":(joint_.name!=source_names_?"joint order":"ok");
      for(size_t i=0;i<6;++i) {
        if(!std::isfinite(joint_.position[i])||!std::isfinite(joint_.velocity[i])) {valid=false;render_reason="joint invalid";}
        else if(std::abs(joint_.velocity[i])>.002) {valid=false;render_reason="moving";}
        else if(std::abs(joint_.position[i]-source_q_[i])>.001) {valid=false;render_reason="pose changed";}
      }
    }
    size_t classified_count=0,camera_count=0,base_count=0;
    if(camera_valid) {
      for(const auto &e:entries_) {
        classified_count+=e.classified;camera_count+=e.camera_geometry;
        const bool show_base=valid&&e.base&&e.classified&&!e.edge;
        if(!show_base) {
          ++pred_observations_;
          if(e.edge)++pred_edge_;
          else if(!e.classified)++pred_class_;
          else if(!e.camera_geometry)++pred_geometry_;
          else if(!e.base_geometry||!e.complete)++pred_base_;
          else if(!e.confirmed)++pred_track_;
          else ++pred_render_;
        } else {++number_observations_;++base_count;}
        const cv::Scalar color=!e.classified?cv::Scalar(0,220,255):
          (e.name.rfind("red_",0)==0?cv::Scalar(50,60,245):cv::Scalar(255,210,80));
        if(e.outline.size()>2)cv::polylines(bgr,std::vector<std::vector<cv::Point>>{e.outline},true,color,2);
        cv::drawMarker(bgr,e.center,color,cv::MARKER_CROSS,11,1);
        const auto found=badges_.find(e.classified?e.name:"unknown");
        const cv::Mat &tile=found!=badges_.end()?found->second:badges_.at("unknown");
        const int x=std::clamp(e.center.x+18,0,bgr.cols-80);
        const int y=std::clamp(e.center.y-35,80,bgr.rows-70);
        tile.copyTo(bgr(cv::Rect(x,y,tile.cols,tile.rows)));
        if(e.edge||!e.camera_geometry) {
          const auto &warning=badges_.at(e.edge?"不完整":"深度待定");
          warning.copyTo(bgr(cv::Rect(x,y+44,warning.cols,warning.rows)));
        } else if(show_base) {
          cv::putText(bgr,"#"+std::to_string(e.id),{x,y+60},cv::FONT_HERSHEY_SIMPLEX,.48,{0,0,0},3);
          cv::putText(bgr,"#"+std::to_string(e.id),{x,y+60},cv::FONT_HERSHEY_SIMPLEX,.48,color,1);
        }
      }
    }
    cv::rectangle(bgr,{0,0},{bgr.cols,77},{20,20,20},-1);
    std::ostringstream title;
    if(camera_valid)title<<"识别 "<<classified_count<<"/"<<entries_.size()<<"   深度 "<<camera_count<<"/"<<entries_.size();
    else title<<"等待实时识别";
    title<<"   |   "<<bgr.cols<<"x"<<bgr.rows<<"   画面 "<<std::fixed<<std::setprecision(1)<<video_hz_<<" Hz   识别 "<<data_hz_<<" Hz";
    font_->putText(bgr,title.str(),{12,31},25,{245,245,245},-1,cv::LINE_AA,true);
    std::ostringstream status;
    if(!camera_valid)status<<"检测结果已过期，等待新帧";
    else if(!valid)status<<"基座坐标暂不可用";
    else status<<"基座坐标 "<<base_count<<"/"<<entries_.size();
    status<<"   |   仅显示测量，不代表可抓取";
    font_->putText(bgr,status.str(),{12,65},23,valid?cv::Scalar(180,230,180):cv::Scalar(0,200,255),-1,cv::LINE_AA,true);
    images_->publish(*image->toImageMsg());++published_;
    if(current-last_metrics_>2000000000LL) {
      const double duration=(current-rate_window_start_)*1e-9;
      if(duration>=1.) {video_hz_=rate_video_frames_/duration;data_hz_=rate_data_frames_/duration;rate_window_start_=current;rate_video_frames_=0;rate_data_frames_=0;}
      std::ostringstream json;json<<"{\"stamp_ns\":"<<stamp<<",\"video_hz\":"<<video_hz_<<",\"recognition_hz\":"<<data_hz_<<",\"published_frames\":"<<published_<<",\"overlay_valid\":"<<(valid?"true":"false")<<",\"visible_tracks\":"<<(camera_valid?entries_.size():0)
          <<",\"number_observations\":"<<number_observations_<<",\"pred_observations\":"<<pred_observations_
          <<",\"pred_edge\":"<<pred_edge_<<",\"pred_classification\":"<<pred_class_<<",\"pred_camera_geometry\":"<<pred_geometry_
          <<",\"pred_base_geometry\":"<<pred_base_<<",\"pred_track_confirmation\":"<<pred_track_<<",\"pred_render_feedback\":"<<pred_render_<<"}";
      std::string metrics_json=json.str();metrics_json.pop_back();
      std::ostringstream extra;extra<<",\"recognition_overlay_valid\":"<<(camera_valid?"true":"false")
        <<",\"base_overlay_valid\":"<<(valid?"true":"false")
        <<",\"classified_count\":"<<classified_count<<",\"camera_geometry_count\":"<<camera_count
        <<",\"base_display_count\":"<<base_count<<"}";
      std_msgs::msg::String value;value.data=metrics_json+extra.str();metrics_->publish(value);last_metrics_=current;
      number_observations_=pred_observations_=pred_edge_=pred_class_=pred_geometry_=pred_base_=pred_track_=pred_render_=0;
    }
  }
  cv::Ptr<cv::freetype::FreeType2> font_;std::map<std::string,cv::Mat> badges_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr source_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_,snapshot_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_;
  sensor_msgs::msg::JointState joint_;std::vector<double> source_q_;std::vector<std::string> source_names_;std::vector<Entry> entries_;
  std::map<int,std::vector<double>> locked_xyz_;std::string observation_state_="waiting",observation_id_,snapshot_id_;double observation_elapsed_=0;bool observation_eligible_=false;int64_t status_received_=0;
  int64_t data_stamp_{0},last_image_{0},last_metrics_{0},rate_window_start_{0};uint64_t published_{0},rate_video_frames_{0},rate_data_frames_{0};double video_hz_{0},data_hz_{0};
  uint64_t number_observations_{0},pred_observations_{0},pred_edge_{0},pred_class_{0},pred_geometry_{0},pred_base_{0},pred_track_{0},pred_render_{0};
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<TrackingPreview>());rclcpp::shutdown();}
