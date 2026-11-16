#pragma once

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <dlfcn.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

class HardwareJpegEncoder {
 public:
  static void initialize() {
    gst_init(nullptr, nullptr);
    GstPlugin *registered = gst_registry_find_plugin(gst_registry_get(), "nvjpeg");
    if (!registered) throw std::runtime_error("nvjpegenc plugin unavailable");
    const std::string filename = gst_plugin_get_filename(registered);
    gst_object_unref(registered);
    // NVIDIA and OpenCV expose incompatible libjpeg ABIs under the same symbols.
    // Resolve this plugin against its own dependencies before GStreamer loads it.
    void *library = dlopen(filename.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
    if (!library) throw std::runtime_error(std::string("hardware JPEG library: ") + dlerror());
    GError *error = nullptr;
    GstPlugin *loaded = gst_plugin_load_file(filename.c_str(), &error);
    dlclose(library);
    if (error) {
      const std::string detail = error->message;
      g_error_free(error);
      throw std::runtime_error("hardware JPEG plugin: " + detail);
    }
    gst_object_unref(loaded);
  }

  HardwareJpegEncoder(int width, int height, int quality)
      : width_(width), height_(height), luma_lookup_(1, 256, CV_8U),
        chroma_lookup_(1, 256, CV_8U) {
    for (int value = 0; value < 256; ++value) {
      luma_lookup_.at<uint8_t>(value) = cv::saturate_cast<uint8_t>(
          std::round((value - 16) * 255.0 / 219.0));
      chroma_lookup_.at<uint8_t>(value) = cv::saturate_cast<uint8_t>(
          std::round((value - 128) * 255.0 / 224.0 + 128));
    }
    const std::string description =
        "appsrc name=input format=time caps=video/x-raw,format=I420,width=" +
        std::to_string(width) + ",height=" + std::to_string(height) +
        ",framerate=30/1 ! nvjpegenc quality=" + std::to_string(quality) +
        " ! appsink name=output sync=false max-buffers=1";
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error) {
      const std::string detail = error->message;
      g_error_free(error);
      release();
      throw std::runtime_error("hardware JPEG pipeline: " + detail);
    }
    source_ = gst_bin_get_by_name(GST_BIN(pipeline_), "input");
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "output");
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      release();
      throw std::runtime_error("hardware JPEG pipeline failed to start");
    }
  }

  ~HardwareJpegEncoder() { release(); }
  HardwareJpegEncoder(const HardwareJpegEncoder &) = delete;
  HardwareJpegEncoder &operator=(const HardwareJpegEncoder &) = delete;

  bool matches(const cv::Mat &image) const {
    return image.cols == width_ && image.rows == height_;
  }

  void encode(const cv::Mat &image, std::vector<uint8_t> &jpeg) {
    cv::cvtColor(image, yuv_, cv::COLOR_BGR2YUV_I420);
    // OpenCV I420 uses studio range; the JPEG hardware consumes full-range YUV.
    auto luma = yuv_.rowRange(0, height_);
    auto chroma = yuv_.rowRange(height_, yuv_.rows);
    cv::LUT(luma, luma_lookup_, luma);
    cv::LUT(chroma, chroma_lookup_, chroma);
    const size_t size = yuv_.total();
    GstBuffer *input = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(input, 0, yuv_.data, size);
    GST_BUFFER_PTS(input) = sequence_++ * GST_SECOND / 30;
    GST_BUFFER_DURATION(input) = GST_SECOND / 30;
    if (gst_app_src_push_buffer(GST_APP_SRC(source_), input) != GST_FLOW_OK)
      throw std::runtime_error("hardware JPEG input rejected");
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), GST_SECOND / 2);
    if (!sample) throw std::runtime_error("hardware JPEG output deadline exceeded");
    GstBuffer *output = gst_sample_get_buffer(sample);
    jpeg.resize(gst_buffer_get_size(output));
    gst_buffer_extract(output, 0, jpeg.data(), jpeg.size());
    gst_sample_unref(sample);
  }

 private:
  void release() {
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (source_) gst_object_unref(source_);
    if (sink_) gst_object_unref(sink_);
    if (pipeline_) gst_object_unref(pipeline_);
    source_ = sink_ = pipeline_ = nullptr;
  }

  int width_, height_;
  cv::Mat yuv_, luma_lookup_, chroma_lookup_;
  GstElement *pipeline_{nullptr}, *source_{nullptr}, *sink_{nullptr};
  uint64_t sequence_{0};
};
