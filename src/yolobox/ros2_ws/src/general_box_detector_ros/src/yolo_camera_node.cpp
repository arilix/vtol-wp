#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hailo/hailort.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

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

struct CameraModel {
  bool valid{false};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string frame_id;
};

struct BboxOrientation {
  bool valid{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
  double yaw_deg{0.0};
};

struct NmsStats {
  std::vector<int> raw_counts;
  std::vector<float> max_scores;
};

std::string class_name(int class_id, int basket_class_id) {
  return class_id == basket_class_id ? "basket_box" : "class_" + std::to_string(class_id);
}

double normalize_degrees(double degrees) {
  double normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

BboxOrientation yaw_orientation(double yaw_deg) {
  const double yaw_rad = yaw_deg * M_PI / 180.0;
  const double half = yaw_rad * 0.5;
  BboxOrientation orientation;
  orientation.valid = true;
  orientation.z = std::sin(half);
  orientation.w = std::cos(half);
  orientation.yaw_deg = yaw_deg;
  return orientation;
}

double angle_delta_180(double current_deg, double reference_deg) {
  double delta = std::fmod(current_deg - reference_deg + 90.0, 180.0);
  if (delta < 0.0) {
    delta += 180.0;
  }
  return delta - 90.0;
}

bool estimate_visual_angle_deg(const cv::Mat &frame, const Detection &detection,
                               double &angle_deg) {
  const cv::Rect bounds(0, 0, frame.cols, frame.rows);
  const cv::Rect box = detection.box & bounds;
  if (box.width < 20 || box.height < 20) {
    return false;
  }

  cv::Mat crop = frame(box);
  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  cv::Mat red_low;
  cv::Mat red_high;
  cv::inRange(hsv, cv::Scalar(0, 45, 40), cv::Scalar(12, 255, 255), red_low);
  cv::inRange(hsv, cv::Scalar(168, 45, 40), cv::Scalar(180, 255, 255), red_high);
  cv::Mat mask = red_low | red_high;
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    return false;
  }

  const auto largest = std::max_element(
      contours.begin(), contours.end(),
      [](const auto &a, const auto &b) { return cv::contourArea(a) < cv::contourArea(b); });
  const double area = cv::contourArea(*largest);
  if (area < static_cast<double>(box.area()) * 0.04) {
    return false;
  }

  const cv::RotatedRect rect = cv::minAreaRect(*largest);
  double angle = rect.angle;
  if (rect.size.width < rect.size.height) {
    angle += 90.0;
  }
  angle_deg = normalize_degrees(angle);
  return true;
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

void draw_bbox_axis_2d(cv::Mat &frame, const cv::Rect &box,
                       const BboxOrientation &orientation) {
  const cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
  const int axis_len = std::clamp(std::min(box.width, box.height) / 4, 24, 70);
  const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

  const cv::Point x_tip(
      center.x + static_cast<int>(std::round(axis_len * std::cos(yaw))),
      center.y + static_cast<int>(std::round(axis_len * std::sin(yaw))));
  const cv::Point y_tip(
      center.x + static_cast<int>(std::round(axis_len * std::cos(yaw + M_PI / 2.0))),
      center.y + static_cast<int>(std::round(axis_len * std::sin(yaw + M_PI / 2.0))));
  const cv::Point z_tip(center.x, center.y - axis_len);

  auto arrow = [](cv::Mat &img, cv::Point from, cv::Point to, cv::Scalar color) {
    cv::arrowedLine(img, from, to, color, 3, cv::LINE_AA, 0, 0.18);
  };
  arrow(frame, center, x_tip, cv::Scalar(0, 0, 255));
  arrow(frame, center, y_tip, cv::Scalar(0, 255, 0));
  arrow(frame, center, z_tip, cv::Scalar(255, 0, 0));
  cv::putText(frame, "X", x_tip + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX,
              0.45, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  cv::putText(frame, "Y", y_tip + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX,
              0.45, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::putText(frame, "Z", z_tip + cv::Point(4, -4), cv::FONT_HERSHEY_SIMPLEX,
              0.45, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
}

void annotate(cv::Mat &frame, const std::vector<Detection> &detections,
              int basket_class_id, double infer_ms, double fps,
              const BboxOrientation &orientation) {
  const cv::Scalar color(0, 255, 0);
  for (const auto &detection : detections) {
    cv::rectangle(frame, detection.box, color, 2);
    draw_bbox_axis_2d(frame, detection.box, orientation);
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

geometry_msgs::msg::TransformStamped make_bbox_transform(
    const rclcpp::Time &stamp,
    const std::string &parent_frame,
    const std::string &child_frame,
    const Detection &detection,
    const CameraModel &camera_model,
    const cv::Size &frame_size,
    double depth_m,
    const BboxOrientation &orientation) {
  const double u = static_cast<double>(detection.box.x) +
                   static_cast<double>(detection.box.width) * 0.5;
  const double v = static_cast<double>(detection.box.y) +
                   static_cast<double>(detection.box.height) * 0.5;

  const double fx = camera_model.valid ? camera_model.fx : static_cast<double>(frame_size.width);
  const double fy = camera_model.valid ? camera_model.fy : static_cast<double>(frame_size.width);
  const double cx = camera_model.valid ? camera_model.cx : static_cast<double>(frame_size.width) * 0.5;
  const double cy = camera_model.valid ? camera_model.cy : static_cast<double>(frame_size.height) * 0.5;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;
  tf.transform.translation.x = ((u - cx) / fx) * depth_m;
  tf.transform.translation.y = ((v - cy) / fy) * depth_m;
  tf.transform.translation.z = depth_m;
  tf.transform.rotation.x = orientation.x;
  tf.transform.rotation.y = orientation.y;
  tf.transform.rotation.z = orientation.z;
  tf.transform.rotation.w = orientation.w;
  return tf;
}

visualization_msgs::msg::MarkerArray make_bbox_markers(
    const rclcpp::Time &stamp,
    const std::string &frame_id,
    const std::vector<Detection> &detections,
    const CameraModel &camera_model,
    const cv::Size &frame_size,
    double depth_m,
    int basket_class_id,
    const BboxOrientation &orientation) {
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear;
  clear.header.stamp = stamp;
  clear.header.frame_id = frame_id;
  clear.ns = "general_box";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  int marker_id = 1;
  for (const auto &detection : detections) {
    const auto tf = make_bbox_transform(
        stamp, frame_id, "general_box_marker_tmp", detection,
        camera_model, frame_size, depth_m, orientation);

    const double box_scale = std::clamp(
        static_cast<double>(std::max(detection.box.width, detection.box.height)) /
            static_cast<double>(std::max(frame_size.width, 1)) * depth_m,
        0.08, 0.45);

    visualization_msgs::msg::Marker cube;
    cube.header.stamp = stamp;
    cube.header.frame_id = frame_id;
    cube.ns = "general_box_pose";
    cube.id = marker_id++;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.position.x = tf.transform.translation.x;
    cube.pose.position.y = tf.transform.translation.y;
    cube.pose.position.z = tf.transform.translation.z;
    cube.pose.orientation.x = orientation.x;
    cube.pose.orientation.y = orientation.y;
    cube.pose.orientation.z = orientation.z;
    cube.pose.orientation.w = orientation.w;
    cube.scale.x = box_scale;
    cube.scale.y = box_scale;
    cube.scale.z = 0.03;
    cube.color.r = 1.0f;
    cube.color.g = 0.85f;
    cube.color.b = 0.0f;
    cube.color.a = 0.85f;
    cube.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(cube);

    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = frame_id;
    text.ns = "general_box_label";
    text.id = marker_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = tf.transform.translation.x;
    text.pose.position.y = tf.transform.translation.y;
    text.pose.position.z = tf.transform.translation.z + std::max(0.10, box_scale * 0.8);
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.08;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.text = class_name(detection.class_id, basket_class_id) + " " +
                cv::format("%.2f", detection.confidence);
    text.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(text);
  }

  return marker_array;
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
  node->declare_parameter("publish_image", false);
  node->declare_parameter("publish_topic", "/general_box/image_annotated");
  node->declare_parameter("count_topic", "/general_box/detection_count");
  node->declare_parameter("boxes_topic", "/general_box/detections");
  node->declare_parameter("boxes_with_yaw_topic", "/general_box/detections_with_yaw");
  node->declare_parameter("center_topic", "/general_box/target_center");
  node->declare_parameter("markers_topic", "/general_box/markers");
  node->declare_parameter("frame_id", "general_box_camera");
  node->declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");
  node->declare_parameter("imu_topic", "/camera/camera/imu");
  node->declare_parameter("publish_tf", true);
  node->declare_parameter("tf_parent_frame", "");
  node->declare_parameter("tf_child_frame", "yolo_bbox_target");
  node->declare_parameter("tf_depth_m", 1.0);
  node->declare_parameter("tf_use_imu_yaw", true);
  node->declare_parameter("tf_heading_offset_deg", 0.0);
  node->declare_parameter("tf_camera_mount_yaw_offset_deg", -90.0);
  node->declare_parameter("target_class_id", -2);
  node->declare_parameter("preprocess_mode", "letterbox");
  node->declare_parameter("input_color_order", "rgb");

  const std::string camera_topic = node->get_parameter("camera_topic").as_string();
  const std::string model = node->get_parameter("model_path").as_string();
  const std::string output = node->get_parameter("output").as_string();
  const float confidence = static_cast<float>(node->get_parameter("confidence").as_double());
  const int max_frames = static_cast<int>(node->get_parameter("max_frames").as_int());
  const bool save_video = node->get_parameter("save_video").as_bool();
  const bool publish_image = node->get_parameter("publish_image").as_bool();
  const std::string publish_topic = node->get_parameter("publish_topic").as_string();
  const std::string count_topic = node->get_parameter("count_topic").as_string();
  const std::string boxes_topic = node->get_parameter("boxes_topic").as_string();
  const std::string boxes_with_yaw_topic = node->get_parameter("boxes_with_yaw_topic").as_string();
  const std::string center_topic = node->get_parameter("center_topic").as_string();
  const std::string markers_topic = node->get_parameter("markers_topic").as_string();
  const std::string frame_id = node->get_parameter("frame_id").as_string();
  const std::string camera_info_topic = node->get_parameter("camera_info_topic").as_string();
  const std::string imu_topic = node->get_parameter("imu_topic").as_string();
  const bool publish_tf = node->get_parameter("publish_tf").as_bool();
  const std::string tf_parent_frame_param = node->get_parameter("tf_parent_frame").as_string();
  const std::string tf_child_frame = node->get_parameter("tf_child_frame").as_string();
  const double tf_depth_m = node->get_parameter("tf_depth_m").as_double();
  const bool tf_use_imu_yaw = node->get_parameter("tf_use_imu_yaw").as_bool();
  const double tf_heading_offset_deg = node->get_parameter("tf_heading_offset_deg").as_double();
  const double tf_camera_mount_yaw_offset_deg =
      node->get_parameter("tf_camera_mount_yaw_offset_deg").as_double();
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
  if (tf_depth_m <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "tf_depth_m harus > 0");
    rclcpp::shutdown();
    return 2;
  }
  if (publish_tf && tf_child_frame.empty()) {
    RCLCPP_ERROR(node->get_logger(), "tf_child_frame tidak boleh kosong saat publish_tf=true");
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

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto image_pub = publish_image
        ? node->create_publisher<sensor_msgs::msg::Image>(publish_topic, image_qos)
        : rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr{};
    auto count_pub = node->create_publisher<std_msgs::msg::UInt32>(count_topic, 10);
    auto boxes_pub =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(boxes_topic, 10);
    auto boxes_with_yaw_pub =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(boxes_with_yaw_topic, 10);
    auto center_pub =
        node->create_publisher<std_msgs::msg::Float32MultiArray>(center_topic, 10);
    auto markers_pub =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic, 10);
    auto tf_broadcaster = publish_tf
        ? std::make_unique<tf2_ros::TransformBroadcaster>(*node)
        : std::unique_ptr<tf2_ros::TransformBroadcaster>{};

    CameraModel camera_model;
    std::mutex imu_mutex;
    double imu_heading_deg = 0.0;
    rclcpp::Time imu_last_stamp;
    bool imu_initialized = false;
    bool imu_valid = false;
    bool object_yaw_latched = false;
    double latched_imu_yaw_camera_deg = 0.0;
    double latched_visual_angle_deg = 0.0;
    double last_object_yaw_camera_deg = 0.0;

    auto camera_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS().keep_last(1).best_effort(),
        [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
            return;
          }
          camera_model.valid = true;
          camera_model.fx = msg->k[0];
          camera_model.fy = msg->k[4];
          camera_model.cx = msg->k[2];
          camera_model.cy = msg->k[5];
          camera_model.frame_id = msg->header.frame_id;
        });
    auto imu_sub = tf_use_imu_yaw
        ? node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
              const rclcpp::Time stamp(msg->header.stamp);
              std::lock_guard<std::mutex> lock(imu_mutex);
              if (!imu_initialized) {
                imu_last_stamp = stamp;
                imu_initialized = true;
                imu_valid = true;
                return;
              }
              const double dt = (stamp - imu_last_stamp).seconds();
              imu_last_stamp = stamp;
              if (dt > 0.0 && dt < 0.25) {
                // RealSense IMU is in camera_imu_optical_frame. With the
                // camera facing down and image top mounted toward drone
                // front: X_ned/front=-Y_optical, Y_ned/right=X_optical,
                // Z_ned/down=Z_optical. Yaw rate uses optical gyro Z.
                imu_heading_deg = normalize_degrees(
                    imu_heading_deg + msg->angular_velocity.z * dt * 180.0 / M_PI);
                imu_valid = true;
              }
            })
        : rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr{};

    cv::VideoWriter writer;
    int frames = 0;
    RCLCPP_INFO(node->get_logger(), "CAMERA_TOPIC=%s (subscribe; node ini tidak buka device sendiri)",
                camera_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "IMAGE_TOPIC=%s (%s)", publish_topic.c_str(),
                publish_image ? "enabled" : "disabled");
    RCLCPP_INFO(node->get_logger(), "COUNT_TOPIC=%s", count_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "BOXES_TOPIC=%s FORMAT=[class_id,confidence,x,y,width,height,...]",
                boxes_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "BOXES_WITH_YAW_TOPIC=%s FORMAT=[class_id,confidence,x,y,width,height,yaw_screen_deg,...]",
                boxes_with_yaw_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "CENTER_TOPIC=%s FORMAT=[frame_width,frame_height,cx,cy,confidence] "
                "(cx/cy/confidence omitted when detection count != 1 - ambiguous multi-box frames rejected)",
                center_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "MARKERS_TOPIC=%s (RViz MarkerArray bbox pose)", markers_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "BBOX_TF=%s child=%s parent=%s depth=%.2fm camera_info=%s",
                publish_tf ? "enabled" : "disabled",
                tf_child_frame.c_str(),
                tf_parent_frame_param.empty() ? "<image_header_or_camera_info>" : tf_parent_frame_param.c_str(),
                tf_depth_m,
                camera_info_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
                "BBOX_TF_IMU_YAW=%s imu_topic=%s heading_offset=%.1fdeg mount_yaw_offset=%.1fdeg",
                tf_use_imu_yaw ? "enabled" : "disabled",
                imu_topic.c_str(),
                tf_heading_offset_deg,
                tf_camera_mount_yaw_offset_deg);
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
        camera_topic, rclcpp::SensorDataQoS().keep_last(1).best_effort(),
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
      BboxOrientation bbox_orientation;
      if (tf_use_imu_yaw) {
        double imu_yaw_camera_deg = 0.0;
        bool imu_fresh = false;
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_fresh = imu_initialized && imu_valid &&
            (node->now() - imu_last_stamp).seconds() <= 1.0;
        if (imu_fresh) {
          imu_yaw_camera_deg = normalize_degrees(
              imu_heading_deg + tf_heading_offset_deg + tf_camera_mount_yaw_offset_deg);
        }

        double visual_angle_deg = 0.0;
        const bool visual_valid = detections.size() == 1 &&
            estimate_visual_angle_deg(frame, detections.front(), visual_angle_deg);

        if (!object_yaw_latched && imu_fresh && visual_valid) {
          latched_imu_yaw_camera_deg = imu_yaw_camera_deg;
          latched_visual_angle_deg = visual_angle_deg;
          last_object_yaw_camera_deg = latched_imu_yaw_camera_deg;
          object_yaw_latched = true;
          RCLCPP_INFO(node->get_logger(),
                      "BBOX_YAW_LATCH imu_camera=%.1fdeg visual=%.1fdeg",
                      latched_imu_yaw_camera_deg, latched_visual_angle_deg);
        }

        if (object_yaw_latched && visual_valid) {
          last_object_yaw_camera_deg = normalize_degrees(
              latched_imu_yaw_camera_deg +
              angle_delta_180(visual_angle_deg, latched_visual_angle_deg));
          bbox_orientation = yaw_orientation(last_object_yaw_camera_deg);
        } else if (object_yaw_latched) {
          bbox_orientation = yaw_orientation(last_object_yaw_camera_deg);
        } else if (imu_fresh) {
          bbox_orientation = yaw_orientation(imu_yaw_camera_deg);
        }
      }
      if (publish_image || save_video) {
        annotate(frame, detections, basket_class_id, infer_ms, fps, bbox_orientation);
      }

      std_msgs::msg::UInt32 count_msg;
      count_msg.data = static_cast<uint32_t>(detections.size());
      count_pub->publish(count_msg);

      if (publish_image && image_pub) {
        std_msgs::msg::Header header;
        header.stamp = msg->header.stamp;
        header.frame_id = msg->header.frame_id.empty() ? frame_id : msg->header.frame_id;
        auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub->publish(*image_msg);
      }

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

      std_msgs::msg::Float32MultiArray boxes_with_yaw_msg;
      boxes_with_yaw_msg.data.reserve(detections.size() * 7);
      for (const auto &detection : detections) {
        boxes_with_yaw_msg.data.push_back(static_cast<float>(detection.class_id));
        boxes_with_yaw_msg.data.push_back(detection.confidence);
        boxes_with_yaw_msg.data.push_back(static_cast<float>(detection.box.x));
        boxes_with_yaw_msg.data.push_back(static_cast<float>(detection.box.y));
        boxes_with_yaw_msg.data.push_back(static_cast<float>(detection.box.width));
        boxes_with_yaw_msg.data.push_back(static_cast<float>(detection.box.height));
        boxes_with_yaw_msg.data.push_back(static_cast<float>(bbox_orientation.yaw_deg));
      }
      boxes_with_yaw_pub->publish(boxes_with_yaw_msg);

      std::string marker_frame = tf_parent_frame_param;
      if (marker_frame.empty()) {
        marker_frame = msg->header.frame_id.empty() ? camera_model.frame_id : msg->header.frame_id;
      }
      if (marker_frame.empty()) {
        marker_frame = frame_id;
      }
      markers_pub->publish(make_bbox_markers(
          msg->header.stamp, marker_frame, detections,
          camera_model, frame.size(), tf_depth_m, basket_class_id,
          bbox_orientation));

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

        if (publish_tf && tf_broadcaster) {
          tf_broadcaster->sendTransform(make_bbox_transform(
              msg->header.stamp, marker_frame, tf_child_frame, only_detection,
              camera_model, frame.size(), tf_depth_m, bbox_orientation));
        }
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
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                  "FRAME=%d DETECTIONS=%zu INFERENCE_MS=%.3f FPS=%.3f",
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
