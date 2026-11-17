// D405 采集节点：对齐、校正彩色与深度图，并发布 ROS 数据。
// 识别和位姿计算由下游节点完成。

#include "chess_robot_vision/calibration.hpp"
#include "chess_robot_vision/msg/frame_info.hpp"
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
    // 参数说明见 config/d405.yaml。
    serial_ = declare_parameter<std::string>("serial", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "d405_color_optical_frame");
    evidence_dir_ = declare_parameter<std::string>("evidence_dir", "");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<int>("fps", 30);
    max_frames_ = declare_parameter<int>("max_frames", 0);
    const int opencv_threads = declare_parameter<int>("opencv_threads", 2);
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    stale_s_ = declare_parameter<double>("stale_timeout_s", 0.5);
    exposure_mode_ = declare_parameter<std::string>("exposure_mode", "keep");
    exposure_us_ = declare_parameter<double>("exposure_us", 10000.0);
    gain_ = declare_parameter<double>("gain", 16.0);

    // 模式不支持、曝光时间无效或不大于 0、增益无效或小于 0 时，拒绝启动。
    // 无效数值指 NaN 或无穷大；keep、auto 模式也检查这些数值。
    if ((exposure_mode_ != "keep" && exposure_mode_ != "auto" && exposure_mode_ != "manual") ||
        !std::isfinite(exposure_us_) || exposure_us_ <= 0 || !std::isfinite(gain_) || gain_ < 0)
      throw std::invalid_argument("invalid exposure_mode/exposure_us/gain");
    // 宽高、帧率和线程数须大于 0，帧数上限须不小于 0，坐标系名不能为空。
    // 超时须为有效数值且至少 0.1 秒；任一项不满足则拒绝启动。下限依据待确认。
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0 || max_frames_ < 0 || opencv_threads < 1 ||
        frame_id_.empty() || !std::isfinite(stale_s_) || stale_s_ < 0.1)
      throw std::invalid_argument("invalid stream parameters");
    cv::setNumThreads(opencv_threads);

    // 参数只在启动时生效，修改后需重启节点。
    parameter_callback_ = add_on_set_parameters_callback([](const auto &) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = false;
      r.reason = "vision settings are startup-only; restart with the new YAML";
      return r;
    });

    const auto qos = rclcpp::SensorDataQoS();
    frame_pub_ = create_publisher<vision::msg::FrameInfo>("/vision/frame_info", qos);
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/color/image_rect", qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/aligned_depth/image_rect", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/vision/color/camera_info", qos);

    start();
    last_frame_ = std::chrono::steady_clock::now();
    // 每 5 ms 检查新帧；相机帧率由 fps 决定。
    timer_ = create_wall_timer(5ms, [this] { tick(); });
  }

  ~VisionNode() override {
    try {
      // 相机已启动时，退出前停止采集。
      if (running_)
        pipeline_.stop();
    } catch (...) {
    }
  }

 private:
  // 打开相机，设置曝光并准备校正映射。
  void start() {
    rs2::context context;
    std::vector<std::string> devices;
    for (auto device : context.query_devices()) {
      const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
      // 只收集名称含“405”的设备序列号。
      if (name.find("405") != std::string::npos)
        devices.emplace_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    }
    // 未指定序列号时，自动选择唯一一台候选相机。
    if (serial_.empty()) {
      // 没找到相机或找到多台时，拒绝启动。
      if (devices.size() != 1)
        throw std::runtime_error("need exactly one D405 or set serial explicitly");
      serial_ = devices.front();
    } else if (std::find(devices.begin(), devices.end(), serial_) == devices.end()) {
      // 找不到指定序列号对应的相机，拒绝启动。
      throw std::runtime_error("configured serial is not a connected D405");
    }

    rs2::config config;
    config.enable_device(serial_);
    config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    config.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
    profile_ = pipeline_.start(config);
    running_ = true;

    // D405 的曝光设置同时影响彩色与深度。
    auto sensor = profile_.get_device().first<rs2::depth_sensor>();
    auto checked_set = [&sensor](rs2_option option, double value) {
      // 相机不支持此设置项时，停止启动。
      if (!sensor.supports(option))
        throw std::runtime_error("unsupported camera exposure option");
      auto range = sensor.get_option_range(option);
      // 设置值超出 SDK 给出的范围时，停止启动。
      if (value < range.min || value > range.max)
        throw std::invalid_argument("camera exposure option outside SDK range");
      sensor.set_option(option, static_cast<float>(value));
    };

    // keep 不修改曝光设置；auto 开启自动曝光。
    if (exposure_mode_ == "auto")
      checked_set(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    // manual 关闭自动曝光，再写入增益和曝光时间。
    if (exposure_mode_ == "manual") {
      checked_set(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
      checked_set(RS2_OPTION_GAIN, gain_);
      checked_set(RS2_OPTION_EXPOSURE, exposure_us_);
    }
    RCLCPP_INFO(
        get_logger(),
        "D405 exposure_mode=%s auto=%.0f exposure_us=%.0f gain=%.0f actual_color_profile=%dx%d@%d",
        exposure_mode_.c_str(), sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE),
        sensor.get_option(RS2_OPTION_EXPOSURE), sensor.get_option(RS2_OPTION_GAIN),
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().width(),
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().height(),
        profile_.get_stream(RS2_STREAM_COLOR).fps());

    scale_ = profile_.get_device().first<rs2::depth_sensor>().get_depth_scale();
    source_intrinsics_ =
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    // 根据相机内参，生成校正图像到原图的采样位置表。
    vision::rectification_maps(source_intrinsics_, rectify_x_, rectify_y_);

    // 将采样位置表转换为 OpenCV 校正用的格式，彩色和深度各一份，逐帧复用。
    cv::convertMaps(rectify_x_, rectify_y_, linear_map_, linear_weights_, CV_16SC2, false);
    cv::Mat unused;
    cv::convertMaps(rectify_x_, rectify_y_, nearest_map_, unused, CV_16SC2, true);
    rectify_x_.release();
    rectify_y_.release();

    RCLCPP_INFO(get_logger(), "rectifying SDK color model %s to pinhole output",
                rs2_distortion_to_string(source_intrinsics_.model));
    RCLCPP_INFO(get_logger(), "D405 serial=%s firmware=%s USB=%s depth_scale=%.9f m/unit",
                serial_.c_str(), profile_.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION),
                profile_.get_device().get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR), scale_);
  }

  // 以下状态仅用于汇总日志，不参与帧有效性判断。
  std::string frame_log_reason_;
  size_t frame_log_count_{0};
  double frame_log_first_{0}, frame_log_last_{0};
  std::chrono::steady_clock::time_point frame_log_reported_{};

  // 汇总重复异常，并在恢复或原因改变时记录。
  void logFrameFault(const std::string &reason, bool recovered = false) {
    // 当前没有待汇总的异常时，忽略恢复通知。
    if (recovered && frame_log_count_ == 0)
      return;
    const auto summary = [&](const char *stage, bool success) {
      const std::string object = serial_;
      // 恢复消息记为 INFO，异常消息记为 ERROR。
      if (success) {
        RCLCPP_INFO(
            get_logger(),
            "camera_stream stage=%s object=%s count=%zu first_unix_s=%.6f last_unix_s=%.6f previous_reason=%s",
            stage, object.c_str(), frame_log_count_, frame_log_first_, frame_log_last_,
            frame_log_reason_.c_str());
      } else {
        RCLCPP_ERROR(
            get_logger(),
            "camera_stream stage=%s object=%s impact=frame_unavailable count=%zu first_unix_s=%.6f last_unix_s=%.6f reason=%s",
            stage, object.c_str(), frame_log_count_, frame_log_first_, frame_log_last_,
            frame_log_reason_.c_str());
      }
    };

    // 收到恢复通知时，记录并清空本轮异常。
    if (recovered) {
      summary("recovered", true);
      frame_log_count_ = 0;
      frame_log_reason_.clear();
      return;
    }
    // 已有异常且原因改变时，先汇总旧异常，再开始新一轮统计。
    if (frame_log_count_ && reason != frame_log_reason_) {
      summary("reason_changed", false);
      frame_log_count_ = 0;
    }

    const auto current = std::chrono::steady_clock::now();
    frame_log_last_ =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    // 本轮首次异常时，保存原因和首次发生时间。
    if (frame_log_count_ == 0) {
      frame_log_reason_ = reason;
      frame_log_first_ = frame_log_last_;
    }
    ++frame_log_count_;
    // 本轮首次异常立即报告；持续异常距上次报告满 2 秒时再汇总。
    if (frame_log_count_ == 1 || current - frame_log_reported_ >= std::chrono::seconds(2)) {
      summary(frame_log_count_ == 1 ? "first" : "ongoing", false);
      frame_log_reported_ = current;
    }
  }

  // 发布无效帧状态，并记录原因。
  void invalid(const std::string &why) {
    vision::msg::FrameInfo msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.stream_ok = false;
    msg.diagnostic = why;
    msg.depth_scale_m = scale_;
    frame_pub_->publish(msg);
    logFrameFault(why);
  }

  // 处理一组新帧；没有新帧时返回。
  void tick() {
    const auto steady_now = std::chrono::steady_clock::now();
    try {
      rs2::frameset frames;
      // 本次未取到新帧时，检查是否超时，然后返回。
      if (!pipeline_.poll_for_frames(&frames)) {
        // 未得到有效帧已超时，且距上次无帧报告超过 200 ms 时，报告异常。
        if (std::chrono::duration<double>(steady_now - last_frame_).count() > stale_s_ &&
            steady_now - last_warning_ > 200ms) {
          invalid("no fresh RGB-D frames");
          last_warning_ = steady_now;
        }
        return;
      }

      // 使用主机收到帧的时间，供图像和配套消息共用。
      const auto received = now();
      // 将深度图对齐到彩色图的像素坐标。
      frames = align_.process(frames).as<rs2::frameset>();
      auto color = frames.get_color_frame();
      auto depth = frames.get_depth_frame();

      // 缺少彩色或深度帧，或两图尺寸不同，丢弃这一组。
      if (!color || !depth || color.get_width() != depth.get_width() ||
          color.get_height() != depth.get_height())
        throw std::runtime_error("missing or mismatched aligned RGB-D pair");
      // 已有有效帧后，任一图像的帧号未递增，就丢弃这一组。
      if (have_frame_ && (color.get_frame_number() <= last_color_number_ ||
                          depth.get_frame_number() <= last_depth_number_))
        throw std::runtime_error("repeated or out-of-order RGB-D frame; discarded");
      // 两图的时间戳来源不同，或采集时间相差超过 1.5 个帧周期时，丢弃这一组。
      // 时间戳单位为毫秒；1.5 个帧周期的取值依据待确认。
      if (color.get_frame_timestamp_domain() != depth.get_frame_timestamp_domain() ||
          std::abs(color.get_timestamp() - depth.get_timestamp()) > 1500.0 / fps_)
        throw std::runtime_error("RGB-D timestamp domains or capture times disagree");

      const auto source_intrinsics =
          depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
      // 复制内参并清零畸变项，用于描述校正后的图像。
      const auto intrinsics = vision::pinhole_intrinsics(source_intrinsics);
      cv::Mat source_bgr(height_, width_, CV_8UC3, const_cast<void *>(color.get_data()),
                         color.get_stride_in_bytes());
      cv::Mat source_z16(height_, width_, CV_16UC1, const_cast<void *>(depth.get_data()),
                         depth.get_stride_in_bytes());
      cv::Mat bgr, z16;
      // 按映射表重新采样，得到校正后的彩色图。
      cv::remap(source_bgr, bgr, linear_map_, linear_weights_, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
      // 深度用最近邻采样，避免混合边缘深度，也保留无效值 0。
      cv::remap(source_z16, z16, nearest_map_, cv::Mat(), cv::INTER_NEAREST, cv::BORDER_CONSTANT);

      std_msgs::msg::Header header;
      header.stamp = received;
      header.frame_id = frame_id_;

      vision::msg::FrameInfo result;
      result.header = header;
      result.color_frame_number = color.get_frame_number();
      result.depth_frame_number = depth.get_frame_number();
      result.device_timestamp_ms = color.get_timestamp();
      result.timestamp_domain = rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain());
      result.depth_scale_m = scale_;
      result.stream_ok = true;
      result.diagnostic = "synchronized rectified RGB-D; no detection or pose estimation";
      // 当前彩色帧提供曝光数据时，附上实际曝光时间。
      if (color.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE))
        result.diagnostic +=
            "; exposure_us=" +
            std::to_string(color.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
      // 当前彩色帧提供增益数据时，附上实际增益。
      if (color.supports_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL))
        result.diagnostic +=
            "; gain=" + std::to_string(color.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL));

      // 本次处理已超过配置的超时时间时，丢弃本帧并报告异常。
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - steady_now).count() >
          stale_s_) {
        invalid("processing exceeded stale_timeout_s; frame discarded");
        return;
      }

      // 发布校正后图像的内参，畸变系数为 0。
      sensor_msgs::msg::CameraInfo info;
      info.header = header;
      info.width = width_;
      info.height = height_;
      info.distortion_model = "plumb_bob";
      info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
      info.k = {
          intrinsics.fx, 0,             intrinsics.ppx,
          0,             intrinsics.fy, intrinsics.ppy,
          0,             0,             1};
      info.r = {
          1, 0, 0,
          0, 1, 0,
          0, 0, 1};
      info.p = {
          intrinsics.fx, 0,             intrinsics.ppx, 0,
          0,             intrinsics.fy, intrinsics.ppy, 0,
          0,             0,             1,             0};
      // 无畸变模型填零系数；其余模型只有 Brown-Conrady 可以直接填入此消息。
      if (intrinsics.model == RS2_DISTORTION_NONE)
        info.d.assign(5, 0.0);
      else if (intrinsics.model != RS2_DISTORTION_BROWN_CONRADY) {
        throw std::runtime_error("unsupported ROS CameraInfo distortion model");
      }

      last_frame_ = std::chrono::steady_clock::now();
      have_frame_ = true;
      last_color_number_ = color.get_frame_number();
      last_depth_number_ = depth.get_frame_number();

      frame_pub_->publish(result);
      info_pub_->publish(info);
      // 允许发布图像且有彩色图订阅者时，发布校正后的彩色图。
      if (publish_raw_ && color_pub_->get_subscription_count() > 0) {
        color_pub_->publish(*cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg());
      }
      // 允许发布图像且有深度图订阅者时，发布校正后的深度图。
      if (publish_raw_ && depth_pub_->get_subscription_count() > 0) {
        // 深度换算为米；原始值 0 表示缺失，发布时改为 NaN。
        cv::Mat metres;
        z16.convertTo(metres, CV_32F, scale_);
        metres.setTo(std::numeric_limits<float>::quiet_NaN(), z16 == 0);
        depth_pub_->publish(*cv_bridge::CvImage(header, "32FC1", metres).toImageMsg());
      }

      ++count_;
      // 已设置保存目录，且到第 30 帧或指定末帧时，保存图像和参数。
      if (!evidence_dir_.empty() && (count_ == 30 || (max_frames_ > 0 && count_ == max_frames_)))
        save(bgr, z16, info, result, intrinsics);
      // 本帧处理成功后，记录此前异常的恢复情况。
      logFrameFault("", true);
      // 已设置帧数上限且成功处理帧数达到上限时，退出节点；上限为 0 时持续运行。
      if (max_frames_ > 0 && count_ >= max_frames_) {
        RCLCPP_INFO(get_logger(), "completed %d frames", count_);
        rclcpp::shutdown();
      }
    } catch (const std::exception &e) {
      invalid(e.what());
    }
  }

  // 保存图像和参数；同目录下再次保存会覆盖同名文件。
  void save(const cv::Mat &color, const cv::Mat &depth, const sensor_msgs::msg::CameraInfo &info,
            const vision::msg::FrameInfo &result, const rs2_intrinsics &intrinsics) {
    std::filesystem::create_directories(evidence_dir_);
    cv::imwrite(evidence_dir_ + "/color.png", color);
    cv::imwrite(evidence_dir_ + "/depth_z16.png", depth);

    cv::FileStorage out(evidence_dir_ + "/capture.json",
                        cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    out << "serial" << serial_
        << "firmware" << profile_.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
        << "depth_scale_m" << scale_
        << "frame_id" << frame_id_
        << "color_frame_number" << double(result.color_frame_number)
        << "depth_frame_number" << double(result.depth_frame_number)
        << "device_timestamp_ms" << result.device_timestamp_ms
        << "timestamp_domain" << result.timestamp_domain
        << "stamp_sec" << info.header.stamp.sec
        << "stamp_nanosec" << double(info.header.stamp.nanosec)
        << "width" << width_
        << "height" << height_
        << "fx" << intrinsics.fx
        << "fy" << intrinsics.fy
        << "ppx" << intrinsics.ppx
        << "ppy" << intrinsics.ppy
        << "distortion" << rs2_distortion_to_string(intrinsics.model)
        << "coeffs" << "[";
    for (float c : intrinsics.coeffs)
      out << c;
    out << "]" << "source_distortion" << rs2_distortion_to_string(source_intrinsics_.model)
        << "source_coeffs" << "[";
    for (float c : source_intrinsics_.coeffs)
      out << c;
    out << "]";
  }

  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  rs2::align align_;

  bool running_{false}, publish_raw_{true};
  double scale_{0}, stale_s_{0.5};
  int width_, height_, fps_, max_frames_, count_{0};
  bool have_frame_{false};
  uint64_t last_color_number_{0}, last_depth_number_{0};
  std::string serial_, frame_id_, evidence_dir_;
  std::string exposure_mode_;
  double exposure_us_{10000.0}, gain_{16.0};

  rs2_intrinsics source_intrinsics_{};
  cv::Mat rectify_x_, rectify_y_;
  cv::Mat linear_map_, linear_weights_, nearest_map_;

  std::chrono::steady_clock::time_point last_frame_, last_warning_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  rclcpp::Publisher<vision::msg::FrameInfo>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VisionNode>());
  } catch (const std::exception &e) {
    std::cerr << "vision_node: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
