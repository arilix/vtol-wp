#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <hailo/hailort.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>

#if defined(__unix__)
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace {

constexpr int kInputSize = 640;
constexpr int kBboxParams = 5;
constexpr int kWaitTimeoutMs = 10000;

struct PreparedFrame {
  cv::Mat image;
  float scale_x;
  float scale_y;
  int pad_x;
  int pad_y;
};

struct Detection {
  int class_id;
  float confidence;
  cv::Rect box;
};

struct NmsStats {
  std::vector<int> raw_counts;
  std::vector<float> max_scores;
};

std::string class_name(int class_id, int basket_class_id) {
  return class_id == basket_class_id ? "basket_box" : "class_" + std::to_string(class_id);
}

std::shared_ptr<uint8_t> page_aligned_alloc(size_t size) {
#if defined(__unix__)
  void *addr = mmap(nullptr, size, PROT_WRITE | PROT_READ,
                    MAP_ANONYMOUS | MAP_PRIVATE, -1, 0);
  if (MAP_FAILED == addr) {
    throw std::bad_alloc();
  }
  return std::shared_ptr<uint8_t>(
      reinterpret_cast<uint8_t *>(addr),
      [size](uint8_t *ptr) { munmap(ptr, size); });
#else
  return std::shared_ptr<uint8_t>(new uint8_t[size],
                                  std::default_delete<uint8_t[]>());
#endif
}

PreparedFrame prepare_frame(const cv::Mat &frame, const std::string &preprocess_mode,
                            const std::string &input_color_order) {
  const bool use_rgb = input_color_order != "bgr";
  auto convert_color = [use_rgb](const cv::Mat &src) {
    cv::Mat out;
    if (use_rgb) {
      cv::cvtColor(src, out, cv::COLOR_BGR2RGB);
    } else {
      out = src.clone();
    }
    return out;
  };

  if (preprocess_mode == "stretch") {
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(kInputSize, kInputSize));
    return {convert_color(resized),
            static_cast<float>(kInputSize) / frame.cols,
            static_cast<float>(kInputSize) / frame.rows,
            0, 0};
  }

  const float scale = std::min(static_cast<float>(kInputSize) / frame.cols,
                               static_cast<float>(kInputSize) / frame.rows);
  const int width = std::lround(frame.cols * scale);
  const int height = std::lround(frame.rows * scale);
  const int pad_x = (kInputSize - width) / 2;
  const int pad_y = (kInputSize - height) / 2;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(width, height));
  cv::Mat converted = convert_color(resized);

  cv::Mat result(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
  converted.copyTo(result(cv::Rect(pad_x, pad_y, width, height)));
  return {result, scale, scale, pad_x, pad_y};
}

float normalize_coord(float value) {
  return value > 1.5F ? value / static_cast<float>(kInputSize) : value;
}

float sigmoid(float x) {
  return 1.0F / (1.0F + std::exp(-x));
}

float score_value(float value) {
  return (value >= 0.0F && value <= 1.0F) ? value : sigmoid(value);
}

float iou(const cv::Rect &a, const cv::Rect &b) {
  const int area_intersection = (a & b).area();
  const int area_union = a.area() + b.area() - area_intersection;
  return area_union > 0 ? static_cast<float>(area_intersection) / area_union : 0.0F;
}

std::vector<Detection> suppress_overlaps(std::vector<Detection> detections,
                                         float iou_threshold) {
  std::sort(detections.begin(), detections.end(),
            [](const Detection &a, const Detection &b) {
              return a.confidence > b.confidence;
            });
  std::vector<Detection> kept;
  for (const auto &candidate : detections) {
    bool overlaps = false;
    for (const auto &existing : kept) {
      if (iou(candidate.box, existing.box) > iou_threshold) {
        overlaps = true;
        break;
      }
    }
    if (!overlaps) {
      kept.push_back(candidate);
    }
  }
  return kept;
}

std::vector<Detection> parse_hailo_nms(const std::vector<uint8_t> &output,
                                       const hailo_nms_shape_t &nms_shape,
                                       const PreparedFrame &prepared,
                                       const cv::Size &frame_size,
                                       float confidence, int target_class_id,
                                       NmsStats *stats) {
  const auto *data = reinterpret_cast<const float *>(output.data());
  std::vector<Detection> detections;
  uint32_t class_offset = 0;
  if (stats != nullptr) {
    stats->raw_counts.assign(nms_shape.number_of_classes, 0);
    stats->max_scores.assign(nms_shape.number_of_classes, 0.0F);
  }

  for (uint32_t cls = 0; cls < nms_shape.number_of_classes; ++cls) {
    int count = static_cast<int>(std::lround(data[class_offset]));
    count = std::clamp(count, 0, static_cast<int>(nms_shape.max_bboxes_per_class));
    if (stats != nullptr) {
      stats->raw_counts[cls] = count;
    }

    for (int i = 0; i < count; ++i) {
      const uint32_t base = class_offset + 1 + (i * kBboxParams);
      const float y_min = normalize_coord(data[base + 0]);
      const float x_min = normalize_coord(data[base + 1]);
      const float y_max = normalize_coord(data[base + 2]);
      const float x_max = normalize_coord(data[base + 3]);
      const float score = data[base + 4];
      if (stats != nullptr) {
        stats->max_scores[cls] = std::max(stats->max_scores[cls], score);
      }

      if (score < confidence) {
        continue;
      }
      if (target_class_id >= 0 && static_cast<int>(cls) != target_class_id) {
        continue;
      }

      const int x1 = std::clamp(
          static_cast<int>(((x_min * kInputSize) - prepared.pad_x) / prepared.scale_x),
          0, frame_size.width - 1);
      const int y1 = std::clamp(
          static_cast<int>(((y_min * kInputSize) - prepared.pad_y) / prepared.scale_y),
          0, frame_size.height - 1);
      const int x2 = std::clamp(
          static_cast<int>(((x_max * kInputSize) - prepared.pad_x) / prepared.scale_x),
          0, frame_size.width - 1);
      const int y2 = std::clamp(
          static_cast<int>(((y_max * kInputSize) - prepared.pad_y) / prepared.scale_y),
          0, frame_size.height - 1);

      if (x2 > x1 && y2 > y1) {
        detections.push_back({static_cast<int>(cls), score, cv::Rect(x1, y1, x2 - x1, y2 - y1)});
      }
    }
    class_offset += 1 + (kBboxParams * count);
  }
  return detections;
}

std::vector<Detection> parse_yolov8_seg_raw(const std::vector<uint8_t> &boxes_output,
                                            const std::vector<uint8_t> &scores_output,
                                            hailo_quant_info_t boxes_quant,
                                            hailo_quant_info_t scores_quant,
                                            const PreparedFrame &prepared,
                                            const cv::Size &frame_size,
                                            float confidence,
                                            int target_class_id,
                                            NmsStats *stats) {
  constexpr int kAnchors = 8400;
  const int boxes_values = static_cast<int>(boxes_output.size());
  const int score_values = static_cast<int>(scores_output.size());
  const int classes = score_values / kAnchors;
  std::vector<Detection> detections;

  auto dequant = [](uint8_t value, const hailo_quant_info_t &qi) {
    return (static_cast<float>(value) - qi.qp_zp) * qi.qp_scale;
  };

  if (stats != nullptr) {
    stats->raw_counts.assign(classes, 0);
    stats->max_scores.assign(classes, 0.0F);
  }

  if (classes <= 0 || boxes_values < kAnchors * 4) {
    return detections;
  }

  const int class_begin = target_class_id >= 0 ? target_class_id : 0;
  const int class_end = target_class_id >= 0 ? target_class_id + 1 : classes;
  for (int anchor = 0; anchor < kAnchors; ++anchor) {
    for (int cls = class_begin; cls < class_end && cls < classes; ++cls) {
      const float score = score_value(dequant(scores_output[(anchor * classes) + cls], scores_quant));
      if (stats != nullptr) {
        stats->max_scores[cls] = std::max(stats->max_scores[cls], score);
        if (score >= confidence) {
          ++stats->raw_counts[cls];
        }
      }
      if (score < confidence) {
        continue;
      }

      const float cx = dequant(boxes_output[(anchor * 4) + 0], boxes_quant);
      const float cy = dequant(boxes_output[(anchor * 4) + 1], boxes_quant);
      const float w = dequant(boxes_output[(anchor * 4) + 2], boxes_quant);
      const float h = dequant(boxes_output[(anchor * 4) + 3], boxes_quant);
      const float x_min = cx - (w * 0.5F);
      const float y_min = cy - (h * 0.5F);
      const float x_max = cx + (w * 0.5F);
      const float y_max = cy + (h * 0.5F);

      const int x1 = std::clamp(
          static_cast<int>((x_min - prepared.pad_x) / prepared.scale_x),
          0, frame_size.width - 1);
      const int y1 = std::clamp(
          static_cast<int>((y_min - prepared.pad_y) / prepared.scale_y),
          0, frame_size.height - 1);
      const int x2 = std::clamp(
          static_cast<int>((x_max - prepared.pad_x) / prepared.scale_x),
          0, frame_size.width - 1);
      const int y2 = std::clamp(
          static_cast<int>((y_max - prepared.pad_y) / prepared.scale_y),
          0, frame_size.height - 1);

      if (x2 > x1 && y2 > y1) {
        detections.push_back({cls, score, cv::Rect(x1, y1, x2 - x1, y2 - y1)});
      }
    }
  }
  return suppress_overlaps(std::move(detections), 0.45F);
}

void annotate(cv::Mat &frame, const std::vector<Detection> &detections,
              int basket_class_id, double infer_ms, double fps) {
  const cv::Scalar color(0, 255, 0);
  for (const auto &detection : detections) {
    cv::rectangle(frame, detection.box, color, 2);
    const std::string label = class_name(detection.class_id, basket_class_id)
                            + " " + cv::format("%.2f", detection.confidence);
    cv::putText(frame, label,
                cv::Point(detection.box.x, std::max(20, detection.box.y - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
  }
  cv::putText(frame,
              cv::format("Hailo AI HAT | FPS %.2f | inference %.1f ms", fps, infer_ms),
              cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 255), 2);
}

class HailoYolo {
 public:
  HailoYolo(const std::string &hef_path, float nms_score_threshold,
            std::string preprocess_mode, std::string input_color_order)
      : preprocess_mode_(std::move(preprocess_mode)),
        input_color_order_(std::move(input_color_order)) {
    hailo_vdevice_params_t params;
    auto status = hailo_init_vdevice_params(&params);
    if (HAILO_SUCCESS != status) {
      throw hailort::hailort_error(status, "Failed to initialize Hailo vdevice params");
    }
    params.group_id = "SHARED";

    vdevice_ = hailort::VDevice::create(params).expect("Failed to create Hailo vdevice");
    infer_model_ = vdevice_->create_infer_model(hef_path).expect("Failed to create Hailo infer model");

    auto input = infer_model_->input().expect("Failed to get Hailo input stream");
    input_name_ = input.name();
    input_frame_size_ = input.get_frame_size();
    output_names_ = infer_model_->get_output_names();

    if (output_names_.size() == 1) {
      auto output = infer_model_->output().expect("Failed to get Hailo output stream");
      if (output.is_nms()) {
        output.set_nms_score_threshold(nms_score_threshold);
        nms_score_threshold_ = nms_score_threshold;
        nms_shape_ = output.get_nms_shape().expect("Hailo output is not NMS");
        mode_ = Mode::NMS;
      } else {
        throw std::runtime_error("Single-output HEF is not an NMS model");
      }
    } else {
      mode_ = Mode::RAW_YOLOV8_SEG;
    }

    configured_ = infer_model_->configure().expect("Failed to configure Hailo infer model");
    bindings_ = configured_.create_bindings().expect("Failed to create Hailo bindings");

    input_buffer_ = page_aligned_alloc(input_frame_size_);
    status = bindings_.input(input_name_)->set_buffer(
        hailort::MemoryView(input_buffer_.get(), input_frame_size_));
    if (HAILO_SUCCESS != status) {
      throw hailort::hailort_error(status, "Failed to bind Hailo input buffer");
    }

    if (mode_ == Mode::NMS) {
      const auto output = infer_model_->output().expect("Failed to get Hailo output stream");
      output_frame_size_ = output.get_frame_size();
      output_buffer_ = page_aligned_alloc(output_frame_size_);
      status = bindings_.output()->set_buffer(
          hailort::MemoryView(output_buffer_.get(), output_frame_size_));
      if (HAILO_SUCCESS != status) {
        throw hailort::hailort_error(status, "Failed to bind Hailo output buffer");
      }
    } else {
      for (const auto &name : output_names_) {
        auto output = infer_model_->output(name).expect("Failed to get Hailo output stream");
        const size_t frame_size = output.get_frame_size();
        output_frame_sizes_[name] = frame_size;
        const auto quant_infos = output.get_quant_infos();
        if (!quant_infos.empty()) {
          output_quant_infos_[name] = quant_infos.front();
        }
        output_buffers_[name] = page_aligned_alloc(frame_size);
        status = bindings_.output(name)->set_buffer(
            hailort::MemoryView(output_buffers_[name].get(), frame_size));
        if (HAILO_SUCCESS != status) {
          throw hailort::hailort_error(status, "Failed to bind Hailo output buffer");
        }
        if (name.find("activation1") != std::string::npos) {
          raw_boxes_name_ = name;
        } else if (name.find("concat14") != std::string::npos) {
          raw_scores_name_ = name;
        }
      }
      if (raw_boxes_name_.empty() || raw_scores_name_.empty()) {
        throw std::runtime_error("Raw YOLOv8-seg outputs not found: need activation1 boxes and concat14 scores");
      }
    }
  }

  uint32_t class_count() const {
    if (mode_ == Mode::NMS) return nms_shape_.number_of_classes;
    const auto it = output_frame_sizes_.find(raw_scores_name_);
    if (it == output_frame_sizes_.end()) return 0;
    constexpr uint32_t kAnchors = 8400;
    return static_cast<uint32_t>(it->second / kAnchors);
  }
  uint32_t max_bboxes_per_class() const {
    return mode_ == Mode::NMS ? nms_shape_.max_bboxes_per_class : 8400;
  }
  float nms_score_threshold() const { return nms_score_threshold_; }
  bool raw_segmentation() const { return mode_ == Mode::RAW_YOLOV8_SEG; }
  const std::string &preprocess_mode() const { return preprocess_mode_; }
  const std::string &input_color_order() const { return input_color_order_; }

  std::vector<Detection> infer(const cv::Mat &frame, float confidence,
                               int target_class_id, double &infer_ms,
                               NmsStats *stats) {
    const auto prepared = prepare_frame(frame, preprocess_mode_, input_color_order_);
    const size_t bytes = prepared.image.total() * prepared.image.elemSize();
    if (bytes != input_frame_size_) {
      throw std::runtime_error("Ukuran input Hailo tidak cocok dengan frame 640x640x3");
    }
    std::copy(prepared.image.data, prepared.image.data + bytes, input_buffer_.get());

    if (mode_ == Mode::NMS) {
      std::fill(output_buffer_.get(), output_buffer_.get() + output_frame_size_, 0);
    } else {
      for (const auto &kv : output_buffers_) {
        std::fill(kv.second.get(), kv.second.get() + output_frame_sizes_.at(kv.first), 0);
      }
    }

    const auto started = std::chrono::steady_clock::now();
    auto status = configured_.run(bindings_, std::chrono::milliseconds(kWaitTimeoutMs));
    infer_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (HAILO_SUCCESS != status) {
      throw hailort::hailort_error(status, "Hailo inference failed");
    }

    if (mode_ == Mode::NMS) {
      std::vector<uint8_t> output(output_buffer_.get(),
                                  output_buffer_.get() + output_frame_size_);
      return parse_hailo_nms(output, nms_shape_, prepared, frame.size(), confidence,
                             target_class_id, stats);
    }

    const auto boxes_it = output_buffers_.find(raw_boxes_name_);
    const auto scores_it = output_buffers_.find(raw_scores_name_);
    std::vector<uint8_t> boxes(boxes_it->second.get(),
                               boxes_it->second.get() + output_frame_sizes_.at(raw_boxes_name_));
    std::vector<uint8_t> scores(scores_it->second.get(),
                                scores_it->second.get() + output_frame_sizes_.at(raw_scores_name_));
    return parse_yolov8_seg_raw(boxes, scores,
                                output_quant_infos_.at(raw_boxes_name_),
                                output_quant_infos_.at(raw_scores_name_),
                                prepared, frame.size(), confidence,
                                target_class_id, stats);
  }

 private:
  enum class Mode { NMS, RAW_YOLOV8_SEG };

  Mode mode_{Mode::NMS};
  std::unique_ptr<hailort::VDevice> vdevice_;
  std::shared_ptr<hailort::InferModel> infer_model_;
  hailort::ConfiguredInferModel configured_;
  hailort::ConfiguredInferModel::Bindings bindings_;
  std::string input_name_;
  std::shared_ptr<uint8_t> input_buffer_;
  std::shared_ptr<uint8_t> output_buffer_;
  std::unordered_map<std::string, std::shared_ptr<uint8_t>> output_buffers_;
  std::unordered_map<std::string, size_t> output_frame_sizes_;
  std::unordered_map<std::string, hailo_quant_info_t> output_quant_infos_;
  std::vector<std::string> output_names_;
  std::string raw_boxes_name_;
  std::string raw_scores_name_;
  size_t input_frame_size_ = 0;
  size_t output_frame_size_ = 0;
  hailo_nms_shape_t nms_shape_{};
  float nms_score_threshold_{0.0F};
  std::string preprocess_mode_;
  std::string input_color_order_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("general_box_yolo_camera");

  // Node ini TIDAK membuka device kamera sendiri — subscribe ke topic
  // image yang sudah dipublish node lain (capture_node/realsense2_camera_node),
  // persis pola aruco_node di package fiducial_detector. Ini penting
  // karena kamera nadir dipakai gantian antara aruco_node dan node ini
  // (lihat PROGRAM_OVERVIEW.md) — kalau dua node sama-sama buka device
  // V4L2 sendiri-sendiri, keduanya akan rebutan device. Dengan subscribe
  // ke topic yang sama, tidak ada konflik device sama sekali.
  node->declare_parameter("camera_topic", "/camera/image_raw");
  node->declare_parameter("model_path", "/home/vtol/wayfix_ws/src/yolobox/models/model.hef");
  node->declare_parameter("output", "/home/vtol/wayfix_ws/src/yolobox/general_box_ros_camera.mp4");
  node->declare_parameter("confidence", 0.25);
  node->declare_parameter("max_frames", 0);
  node->declare_parameter("save_video", false);
  node->declare_parameter("publish_topic", "/general_box/image_annotated");
  node->declare_parameter("count_topic", "/general_box/detection_count");
  node->declare_parameter("boxes_topic", "/general_box/detections");
  node->declare_parameter("center_topic", "/general_box/target_center");
  node->declare_parameter("frame_id", "general_box_camera");
  node->declare_parameter("target_class_id", -2);
  node->declare_parameter("preprocess_mode", "letterbox");
  node->declare_parameter("input_color_order", "rgb");

  const std::string camera_topic = node->get_parameter("camera_topic").as_string();
  const std::string model = node->get_parameter("model_path").as_string();
  const std::string output = node->get_parameter("output").as_string();
  const float confidence = static_cast<float>(node->get_parameter("confidence").as_double());
  const int max_frames = static_cast<int>(node->get_parameter("max_frames").as_int());
  const bool save_video = node->get_parameter("save_video").as_bool();
  const std::string publish_topic = node->get_parameter("publish_topic").as_string();
  const std::string count_topic = node->get_parameter("count_topic").as_string();
  const std::string boxes_topic = node->get_parameter("boxes_topic").as_string();
  const std::string center_topic = node->get_parameter("center_topic").as_string();
  const std::string frame_id = node->get_parameter("frame_id").as_string();
  const int requested_target_class_id = static_cast<int>(node->get_parameter("target_class_id").as_int());
  std::string preprocess_mode = node->get_parameter("preprocess_mode").as_string();
  std::string input_color_order = node->get_parameter("input_color_order").as_string();
  std::transform(preprocess_mode.begin(), preprocess_mode.end(), preprocess_mode.begin(), ::tolower);
  std::transform(input_color_order.begin(), input_color_order.end(), input_color_order.begin(), ::tolower);

  if (confidence <= 0.0F || confidence > 1.0F) {
    RCLCPP_ERROR(node->get_logger(), "confidence harus dalam rentang (0, 1]");
    rclcpp::shutdown();
    return 2;
  }
  if (max_frames < 0) {
    RCLCPP_ERROR(node->get_logger(), "max_frames tidak boleh negatif");
    rclcpp::shutdown();
    return 2;
  }
  if (preprocess_mode != "letterbox" && preprocess_mode != "stretch") {
    RCLCPP_ERROR(node->get_logger(), "preprocess_mode harus letterbox atau stretch");
    rclcpp::shutdown();
    return 2;
  }
  if (input_color_order != "rgb" && input_color_order != "bgr") {
    RCLCPP_ERROR(node->get_logger(), "input_color_order harus rgb atau bgr");
    rclcpp::shutdown();
    return 2;
  }

  try {
    cv::setNumThreads(2);
    HailoYolo detector(model, confidence, preprocess_mode, input_color_order);
    const int basket_class_id = requested_target_class_id == -2
        ? (detector.class_count() == 1 ? 0 : 3)
        : requested_target_class_id;
    RCLCPP_INFO(node->get_logger(), "MODEL_CLASSES=%u MAX_BBOXES_PER_CLASS=%u HAILO_NMS_THRESHOLD=%.3f",
                detector.class_count(), detector.max_bboxes_per_class(),
                detector.nms_score_threshold());
    RCLCPP_INFO(node->get_logger(), "MODEL_MODE=%s",
                detector.raw_segmentation() ? "RAW_YOLOV8_SEG" : "HAILO_NMS");
    RCLCPP_INFO(node->get_logger(), "PREPROCESS_MODE=%s INPUT_COLOR_ORDER=%s",
                detector.preprocess_mode().c_str(), detector.input_color_order().c_str());

    auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(publish_topic, 10);
    auto count_pub = node->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    auto boxes_pub =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(boxes_topic, 10);
    auto center_pub =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(center_topic, 10);

    cv::VideoWriter writer;
    int frames = 0;
    RCLCPP_INFO(node->get_logger(), "CAMERA_TOPIC=%s (subscribe; node ini tidak buka device sendiri)",
                camera_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "IMAGE_TOPIC=%s", publish_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "COUNT_TOPIC=%s", count_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "BOXES_TOPIC=%s FORMAT=[class_id,confidence,x,y,width,height,...]",
                boxes_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "CENTER_TOPIC=%s FORMAT=[frame_width,frame_height,cx,cy,confidence] "
                "(cx/cy/confidence omitted when detection count != 1 - ambiguous multi-box frames rejected)",
                center_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "BACKEND=Hailo AI HAT");
    RCLCPP_INFO(node->get_logger(),
                "TARGET_CLASS_ID=%d LABEL=basket_box (-2 auto: 0 for single-class, 3 for multi-class)",
                basket_class_id);

    // Skip-inferensi (bukan start/stop proses) berdasar sinyal dari
    // mission_manager (px4) di /mission/vision_source_active ("ARUCO"/
    // "YOLO"/"NONE") — hemat CPU/NPU pas bukan giliran WP2 tanpa risiko
    // lifecycle proses di tengah misi. Kalau sinyal belum pernah datang
    // atau sudah basi >1s (mission_manager belum/tidak jalan, mis. saat
    // tes standalone seperti ini), DEFAULT TETAP INFERENSI — supaya node
    // ini tetap bisa dites sendirian tanpa px4 seperti biasa.
    bool active_signal_seen = false;
    std::string active_vision_source;
    rclcpp::Time last_active_signal_time;
    auto vision_source_sub = node->create_subscription<std_msgs::msg::String>(
        "/mission/vision_source_active", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
          active_vision_source = msg->data;
          last_active_signal_time = node->now();
          active_signal_seen = true;
        });

    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, rclcpp::SensorDataQoS(),
        [&](const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
      cv::Mat frame;
      try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      } catch (const cv_bridge::Exception &error) {
        RCLCPP_WARN(node->get_logger(), "cv_bridge gagal konversi frame dari %s: %s",
                    camera_topic.c_str(), error.what());
        return;
      }
      if (frame.empty()) {
        return;
      }

      // Log ini sengaja DEBUG (bukan INFO) — saat giliran ArUco (WP lain
      // selain WP1), pesan "bukan giliran YOLO" ini akan terus berulang
      // tiap 2 detik dan cuma jadi noise; log YOLO di terminal ini
      // seharusnya cuma muncul saat YOLO memang sedang dipakai (WP1).
      // Naikkan log level ke debug kalau perlu menelusuri kenapa node ini
      // idle.
      const bool signal_fresh = active_signal_seen &&
          (node->now() - last_active_signal_time).seconds() <= 1.0;
      if (signal_fresh && active_vision_source != "YOLO") {
        RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
            "SKIP inferensi - vision_source_active=%s (bukan giliran YOLO)",
            active_vision_source.c_str());
        return;
      }

      const auto loop_started = std::chrono::steady_clock::now();
      double infer_ms = 0.0;
      NmsStats nms_stats;
      const auto detections = detector.infer(frame, confidence, basket_class_id, infer_ms,
                                             &nms_stats);
      const double loop_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - loop_started).count();
      const double fps = 1000.0 / std::max(loop_ms, 0.001);
      annotate(frame, detections, basket_class_id, infer_ms, fps);

      std_msgs::msg::UInt32 count_msg;
      count_msg.data = static_cast<uint32_t>(detections.size());
      count_pub->publish(count_msg);

      std_msgs::msg::Header header;
      header.stamp = node->now();
      header.frame_id = frame_id;
      auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      image_pub->publish(*image_msg);

      std_msgs::msg::Float32MultiArray boxes_msg;
      boxes_msg.data.reserve(detections.size() * 6);
      for (const auto &detection : detections) {
        boxes_msg.data.push_back(static_cast<float>(detection.class_id));
        boxes_msg.data.push_back(detection.confidence);
        boxes_msg.data.push_back(static_cast<float>(detection.box.x));
        boxes_msg.data.push_back(static_cast<float>(detection.box.y));
        boxes_msg.data.push_back(static_cast<float>(detection.box.width));
        boxes_msg.data.push_back(static_cast<float>(detection.box.height));
      }
      boxes_pub->publish(boxes_msg);

      // target_center cuma dianggap valid kalau PERSIS SATU box terdeteksi.
      // Box (beda dari marker ArUco) tidak punya ID unik — kalau lebih dari
      // satu box kelihatan, "pilih confidence tertinggi" bisa gonta-ganti
      // box fisik antar-frame (flicker) dan GroundLock/mission_manager di
      // sisi px4 akan mengejar target yang meloncat-loncat. Daripada
      // menebak, treat sebagai "tidak ada target" — mission_manager akan
      // menahan target centering terakhir (lihat CENTER_MARKER) sampai
      // frame berikutnya kembali jelas cuma satu box.
      if (detections.size() > 1) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "DETECTIONS=%zu (>1) - target_center diabaikan, box ambigu.",
            detections.size());
      }

      std_msgs::msg::Float32MultiArray center_msg;
      center_msg.data.push_back(static_cast<float>(frame.cols));
      center_msg.data.push_back(static_cast<float>(frame.rows));
      if (detections.size() == 1) {
        const auto &only_detection = detections.front();
        center_msg.data.push_back(static_cast<float>(only_detection.box.x) +
                                   static_cast<float>(only_detection.box.width) * 0.5F);
        center_msg.data.push_back(static_cast<float>(only_detection.box.y) +
                                   static_cast<float>(only_detection.box.height) * 0.5F);
        center_msg.data.push_back(only_detection.confidence);
      }
      center_pub->publish(center_msg);

      if (save_video) {
        if (!writer.isOpened()) {
          // fps sumber tidak lagi diketahui node ini (kamera dibuka
          // capture_node/realsense2_camera_node, bukan di sini) — pakai
          // nilai tetap, cukup buat debug/rekaman, bukan buat timing presisi.
          writer.open(output, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                      20.0, frame.size());
          if (!writer.isOpened()) {
            RCLCPP_ERROR(node->get_logger(), "Output video gagal dibuka: %s", output.c_str());
          }
        }
        if (writer.isOpened()) {
          writer.write(frame);
        }
      }

      ++frames;
      RCLCPP_INFO(node->get_logger(), "FRAME=%d DETECTIONS=%zu INFERENCE_MS=%.3f FPS=%.3f",
                  frames, detections.size(), infer_ms, fps);
      if (frames == 1 || frames % 30 == 0) {
        std::ostringstream stats_line;
        stats_line << "NMS_STATS";
        for (size_t i = 0; i < nms_stats.raw_counts.size(); ++i) {
          stats_line << " cls" << i << " count=" << nms_stats.raw_counts[i]
                     << " max=" << cv::format("%.3f", nms_stats.max_scores[i]);
        }
        RCLCPP_INFO(node->get_logger(), "%s", stats_line.str().c_str());
      }

      if (max_frames > 0 && frames >= max_frames) {
        rclcpp::shutdown();
      }
    });

    rclcpp::spin(node);
  } catch (const hailort::hailort_error &error) {
    RCLCPP_ERROR(node->get_logger(), "HAILO_ERROR status=%d message=%s",
                 error.status(), error.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(node->get_logger(), "ERROR: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
