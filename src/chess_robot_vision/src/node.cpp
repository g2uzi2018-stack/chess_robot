#include "chess_robot_vision/calibration.hpp"
#include "chess_robot_vision/msg/frame_info.hpp"
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
namespace vision = chess_robot_vision;
class VisionNode final : public rclcpp::Node {
 public:
  VisionNode() : Node("vision_node"), align_(RS2_STREAM_COLOR) {
    serial_ = declare_parameter<std::string>("serial", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "d405_color_optical_frame");
    evidence_dir_ = declare_parameter<std::string>("evidence_dir", "");
    width_ = declare_parameter<int>("width",640); height_ = declare_parameter<int>("height",480);
    fps_ = declare_parameter<int>("fps",30); max_frames_ = declare_parameter<int>("max_frames",0);
    const int opencv_threads=declare_parameter<int>("opencv_threads",2);
    publish_raw_ = declare_parameter<bool>("publish_raw",true);
    stale_s_ = declare_parameter<double>("stale_timeout_s",0.5);
    preview_min_depth_ = declare_parameter<double>("preview_min_depth_m",0.07);
    preview_max_depth_ = declare_parameter<double>("preview_max_depth_m",0.5);
    if (width_<=0 || height_<=0 || fps_<=0 || max_frames_<0 || opencv_threads<1 || frame_id_.empty() ||
        !std::isfinite(preview_min_depth_) || !std::isfinite(preview_max_depth_) ||
        preview_min_depth_<=0 || preview_max_depth_<=preview_min_depth_ ||
        !std::isfinite(stale_s_) || stale_s_ < 0.1) throw std::invalid_argument("invalid stream parameters");
    cv::setNumThreads(opencv_threads);
    // Parameters are startup-only; reject silent runtime edits to camera/calibration settings.
    parameter_callback_ = add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult r; r.successful=false;
      r.reason="vision settings are startup-only; restart with the new YAML"; return r;
    });
    const auto qos = rclcpp::SensorDataQoS();
    frame_pub_ = create_publisher<vision::msg::FrameInfo>("/vision/frame_info",qos);
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/color/image_rect",qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/aligned_depth/image_rect",qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/vision/color/camera_info",qos);
    color_preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/vision/color/image_rect/compressed",qos);
    depth_preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/vision/aligned_depth/preview/compressed",qos);
    start();
    last_frame_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(5ms,[this] { tick(); });
  }
  ~VisionNode() override { try { if (running_) pipeline_.stop(); } catch (...) {} }
 private:
  void start() {
    rs2::context context;
    std::vector<std::string> devices;
    for (auto device : context.query_devices()) {
      const std::string name=device.get_info(RS2_CAMERA_INFO_NAME);
      if (name.find("405")!=std::string::npos) devices.emplace_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }
    if (serial_.empty()) {
      if (devices.size()!=1) throw std::runtime_error("need exactly one D405 or set serial explicitly");
      serial_=devices.front();
    } else if (std::find(devices.begin(),devices.end(),serial_)==devices.end()) {
      throw std::runtime_error("configured serial is not a connected D405");
    }
    rs2::config config;
    config.enable_device(serial_);
    config.enable_stream(RS2_STREAM_DEPTH,width_,height_,RS2_FORMAT_Z16,fps_);
    config.enable_stream(RS2_STREAM_COLOR,width_,height_,RS2_FORMAT_BGR8,fps_);
    profile_=pipeline_.start(config); running_=true;
    scale_=profile_.get_device().first<rs2::depth_sensor>().get_depth_scale();
    source_intrinsics_=profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    vision::rectification_maps(source_intrinsics_,rectify_x_,rectify_y_);
    RCLCPP_INFO(get_logger(),"rectifying SDK color model %s to pinhole output",rs2_distortion_to_string(source_intrinsics_.model));
    RCLCPP_INFO(get_logger(),"D405 serial=%s firmware=%s USB=%s depth_scale=%.9f m/unit",
      serial_.c_str(),profile_.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
      profile_.get_device().get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR),scale_);
  }
  void invalid(const std::string &why) {
    vision::msg::FrameInfo msg;
    msg.header.stamp=now(); msg.header.frame_id=frame_id_;
    msg.stream_ok=false; msg.diagnostic=why;
    msg.depth_scale_m=scale_;
    frame_pub_->publish(msg);
  }
  void publish_preview(const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &publisher,
                       const std_msgs::msg::Header &header,const cv::Mat &image) {
    sensor_msgs::msg::CompressedImage message;
    message.header=header; message.format="bgr8; jpeg compressed bgr8";
    cv::imencode(".jpg",image,message.data,{cv::IMWRITE_JPEG_QUALITY,75});
    publisher->publish(message);
  }
  void tick() {
    const auto steady_now=std::chrono::steady_clock::now();
    try {
      rs2::frameset frames;
      if (!pipeline_.poll_for_frames(&frames)) {
        if (std::chrono::duration<double>(steady_now-last_frame_).count()>stale_s_ &&
            steady_now-last_warning_>200ms) {
          invalid("no fresh RGB-D frames"); last_warning_=steady_now;
        }
        return;
      }
      // Stamp host receipt before alignment/rectification, not processing completion.
      const auto received=now();
      frames=align_.process(frames).as<rs2::frameset>();
      auto color=frames.get_color_frame(); auto depth=frames.get_depth_frame();
      if (!color || !depth || color.get_width()!=depth.get_width() || color.get_height()!=depth.get_height())
        throw std::runtime_error("missing or mismatched aligned RGB-D pair");
      if (have_frame_ && (color.get_frame_number()<=last_color_number_ || depth.get_frame_number()<=last_depth_number_))
        throw std::runtime_error("repeated or out-of-order RGB-D frame; discarded");
      if (color.get_frame_timestamp_domain()!=depth.get_frame_timestamp_domain() ||
          std::abs(color.get_timestamp()-depth.get_timestamp())>1500.0/fps_)
        throw std::runtime_error("RGB-D timestamp domains or capture times disagree");
      const auto source_intrinsics=depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
      const auto intrinsics=vision::pinhole_intrinsics(source_intrinsics);
      cv::Mat source_bgr(height_,width_,CV_8UC3,const_cast<void *>(color.get_data()),color.get_stride_in_bytes());
      cv::Mat source_z16(height_,width_,CV_16UC1,const_cast<void *>(depth.get_data()),depth.get_stride_in_bytes());
      cv::Mat bgr,z16;
      cv::remap(source_bgr,bgr,rectify_x_,rectify_y_,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
      // Nearest-neighbour preserves Z16 units/holes, avoiding depth blending across object edges.
      cv::remap(source_z16,z16,rectify_x_,rectify_y_,cv::INTER_NEAREST,cv::BORDER_CONSTANT);
      std_msgs::msg::Header header; header.stamp=received; header.frame_id=frame_id_;
      vision::msg::FrameInfo result;
      result.header=header; result.color_frame_number=color.get_frame_number();
      result.depth_frame_number=depth.get_frame_number();
      result.device_timestamp_ms=color.get_timestamp();
      result.timestamp_domain=rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain());
      result.depth_scale_m=scale_;
      result.stream_ok=true;
      result.diagnostic="synchronized rectified RGB-D; no detection or pose estimation";
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-steady_now).count()>stale_s_) {
        invalid("processing exceeded stale_timeout_s; frame discarded"); return;
      }
      sensor_msgs::msg::CameraInfo info;
      info.header=header; info.width=width_; info.height=height_;
      info.distortion_model="plumb_bob";
      info.d.assign(intrinsics.coeffs,intrinsics.coeffs+5);
      info.k={intrinsics.fx,0,intrinsics.ppx,0,intrinsics.fy,intrinsics.ppy,0,0,1};
      info.r={1,0,0,0,1,0,0,0,1};
      info.p={intrinsics.fx,0,intrinsics.ppx,0,0,intrinsics.fy,intrinsics.ppy,0,0,0,1,0};
      // CameraInfo describes the aligned image plane. Distortion model is preserved below.
      if (intrinsics.model==RS2_DISTORTION_NONE) info.d.assign(5,0.0);
      else if (intrinsics.model!=RS2_DISTORTION_BROWN_CONRADY) {
        throw std::runtime_error("unsupported ROS CameraInfo distortion model");
      }
      last_frame_=std::chrono::steady_clock::now();
      have_frame_=true; last_color_number_=color.get_frame_number(); last_depth_number_=depth.get_frame_number();
      frame_pub_->publish(result);
      info_pub_->publish(info);
      if (publish_raw_ && color_pub_->get_subscription_count()>0) {
        color_pub_->publish(*cv_bridge::CvImage(header,"bgr8",bgr).toImageMsg());
      }
      if (publish_raw_ && depth_pub_->get_subscription_count()>0) {
        // Convert SDK Z16 units to metres; 0 denotes missing depth.
        cv::Mat metres; z16.convertTo(metres,CV_32F,scale_);
        metres.setTo(std::numeric_limits<float>::quiet_NaN(),z16==0);
        depth_pub_->publish(*cv_bridge::CvImage(header,"32FC1",metres).toImageMsg());
      }
      if(color_preview_pub_->get_subscription_count()>0) publish_preview(color_preview_pub_,header,bgr);
      if(depth_preview_pub_->get_subscription_count()>0) {
        cv::Mat metres,gray,visual;
        z16.convertTo(metres,CV_32F,scale_);
        const cv::Mat valid=(metres>=preview_min_depth_)&(metres<=preview_max_depth_);
        metres.convertTo(gray,CV_8U,-255.0/(preview_max_depth_-preview_min_depth_),
          255.0*preview_max_depth_/(preview_max_depth_-preview_min_depth_));
        cv::applyColorMap(gray,visual,cv::COLORMAP_TURBO);
        visual.setTo(cv::Scalar(0,0,0),~valid);
        publish_preview(depth_preview_pub_,header,visual);
      }
      ++count_;
      if (!evidence_dir_.empty() && (count_==30 || (max_frames_>0 && count_==max_frames_)))
        save(bgr,z16,info,result,intrinsics);
      if (max_frames_>0 && count_>=max_frames_) {
        RCLCPP_INFO(get_logger(),"completed %d frames",count_); rclcpp::shutdown();
      }
    } catch (const std::exception &e) {
      invalid(e.what());
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"vision: %s",e.what());
    }
  }
  void save(const cv::Mat &color,const cv::Mat &depth,
            const sensor_msgs::msg::CameraInfo &info,const vision::msg::FrameInfo &result,
            const rs2_intrinsics &intrinsics) {
    std::filesystem::create_directories(evidence_dir_);
    cv::imwrite(evidence_dir_+"/color.png",color);
    cv::imwrite(evidence_dir_+"/depth_z16.png",depth);
    cv::FileStorage out(evidence_dir_+"/capture.json",cv::FileStorage::WRITE|cv::FileStorage::FORMAT_JSON);
    out << "serial" << serial_ << "firmware" << profile_.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
        << "depth_scale_m" << scale_ << "frame_id" << frame_id_ << "color_frame_number" << double(result.color_frame_number)
        << "depth_frame_number" << double(result.depth_frame_number)
        << "device_timestamp_ms" << result.device_timestamp_ms << "timestamp_domain" << result.timestamp_domain
        << "stamp_sec" << info.header.stamp.sec << "stamp_nanosec" << double(info.header.stamp.nanosec)
        << "width" << width_ << "height" << height_ << "fx" << intrinsics.fx << "fy" << intrinsics.fy
        << "ppx" << intrinsics.ppx << "ppy" << intrinsics.ppy << "distortion" << rs2_distortion_to_string(intrinsics.model)
        << "coeffs" << "[";
    for (float c : intrinsics.coeffs) out << c;
    out << "]" << "source_distortion" << rs2_distortion_to_string(source_intrinsics_.model)
        << "source_coeffs" << "[";
    for(float c:source_intrinsics_.coeffs) out << c;
    out << "]";
  }
  rs2::pipeline pipeline_; rs2::pipeline_profile profile_; rs2::align align_;
  bool running_{false},publish_raw_{true}; double scale_{0},stale_s_{0.5};
  double preview_min_depth_{0.07},preview_max_depth_{0.5};
  int width_,height_,fps_,max_frames_,count_{0};
  bool have_frame_{false}; uint64_t last_color_number_{0},last_depth_number_{0};
  std::string serial_,frame_id_,evidence_dir_;
  rs2_intrinsics source_intrinsics_{}; cv::Mat rectify_x_,rectify_y_;
  std::chrono::steady_clock::time_point last_frame_,last_warning_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<vision::msg::FrameInfo>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_,depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_preview_pub_,depth_preview_pub_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<VisionNode>()); }
  catch (const std::exception &e) { std::cerr<<"vision_node: "<<e.what()<<std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
