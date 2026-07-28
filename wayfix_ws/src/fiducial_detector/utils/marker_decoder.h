#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
#include <map>
namespace fiducial_detector {
enum class RejectionReason : uint8_t {
  NONE               = 0,
  BAD_CONTOUR        = 1,
  LOW_CONFIDENCE     = 2,
  BORDER_FAILURE     = 3,
  HAMMING_ERROR      = 4,
  PERSPECTIVE_FAILURE= 5,
  SIZE_FILTER        = 6,
  BORDER_TOO_CLOSE   = 7,
};
inline std::string rejectionReasonString(RejectionReason r) {
  switch (r) {
    case RejectionReason::BAD_CONTOUR:         return "BAD_CONTOUR";
    case RejectionReason::LOW_CONFIDENCE:      return "LOW_CONF";
    case RejectionReason::BORDER_FAILURE:      return "BORDER_FAIL";
    case RejectionReason::HAMMING_ERROR:       return "HAMMING_ERR";
    case RejectionReason::PERSPECTIVE_FAILURE: return "PERSP_FAIL";
    case RejectionReason::SIZE_FILTER:         return "SIZE_FILT";
    case RejectionReason::BORDER_TOO_CLOSE:    return "BORDER_CLOSE";
    default:                                   return "UNKNOWN";
  }
}
struct RejectedCandidate {
  std::vector<cv::Point2f> corners;
  RejectionReason          reason{RejectionReason::BAD_CONTOUR};
  float                    confidence{0.f};
  int                      hamming_distance{-1};
};
struct MarkerDebugImages {
  cv::Mat threshold_image;
  cv::Mat contour_image;
  cv::Mat perspective_image;
  cv::Mat cell_grid_image;
  cv::Mat bit_matrix_image;
  cv::Mat rejected_image;
  bool    valid{false};
};
struct MarkerDecodeDetail {
  int  id{-1};
  int  marker_bits{4};
  std::vector<uint8_t> bits;
  std::vector<uint8_t> border_bits;
  int  hamming_distance{-1};
  bool border_valid{false};
  cv::Mat perspective_image;
};
class MarkerDecoder {
public:
  explicit MarkerDecoder(
    cv::Ptr<cv::aruco::Dictionary> dict,
    int    cell_pixels = 8,
    int    border_bits = 1,
    double margin_frac = 0.13);
  void setDictionary(cv::Ptr<cv::aruco::Dictionary> dict) { dict_ = dict; }
  cv::Mat applyAdaptiveThreshold(
    const cv::Mat& gray,
    int win_size_min  = 3,
    int win_size_max  = 23,
    int win_size_step = 10,
    double constant   = 7.0) const;
  std::vector<std::vector<cv::Point2f>> extractCandidates(
    const cv::Mat& threshold,
    double min_perimeter_rate = 0.03,
    double max_perimeter_rate = 4.0,
    double approx_accuracy    = 0.03,
    int    min_dist_to_border = 3) const;
  cv::Mat perspectiveWarp(
    const cv::Mat&                   gray,
    const std::vector<cv::Point2f>&  corners) const;
  std::vector<std::vector<cv::Mat>> segmentCells(
    const cv::Mat& normalized) const;
  std::vector<uint8_t> extractBits(
    const std::vector<std::vector<cv::Mat>>& cells) const;
  bool validateBorderBits(
    const std::vector<uint8_t>& bits,
    int total_side,
    double max_error_rate = 0.35) const;
  bool matchDictionary(
    const std::vector<uint8_t>& data_bits,
    int&  id,
    int&  hamming,
    double error_rate = 0.6) const;
  MarkerDecodeDetail decodeMarker(
    const cv::Mat&                  gray,
    const std::vector<cv::Point2f>& corners) const;
  RejectedCandidate analyseRejected(
    const cv::Mat&                  gray,
    const std::vector<cv::Point2f>& corners,
    double max_border_error = 0.35,
    double error_rate       = 0.6) const;
  cv::Mat visualizeCellGrid(
    const cv::Mat&              normalized,
    const std::vector<uint8_t>& bits,
    cv::Size out_size = {460, 460}) const;
  cv::Mat visualizeBitMatrix(
    const std::vector<uint8_t>& bits,
    int id,
    int hamming_dist,
    cv::Size out_size = {300, 300}) const;
  MarkerDebugImages generateDebugImages(
    const cv::Mat&                  frame,
    const cv::Mat&                  gray,
    const std::vector<cv::Point2f>& corners,
    const std::vector<RejectedCandidate>& rejected = {}) const;
  cv::Mat visualizeThreshold(
    const cv::Mat& gray,
    int win_size = 11,
    double constant = 7.0) const;
  cv::Mat visualizeContours(
    const cv::Mat&                               frame,
    const std::vector<std::vector<cv::Point2f>>& candidates) const;
  cv::Mat visualizeRejected(
    const cv::Mat&                        frame,
    const std::vector<RejectedCandidate>& rejected) const;
  int cellPixels()  const { return cell_pixels_; }
  int borderBits()  const { return border_bits_; }
  void updateBorderBits(int bits) { border_bits_ = bits; }
  int totalSide()   const;
private:
  cv::Ptr<cv::aruco::Dictionary> dict_;
  int    cell_pixels_{8};
  int    border_bits_{1};
  double margin_frac_{0.13};
  int getBitsPerSide() const;
};
}
