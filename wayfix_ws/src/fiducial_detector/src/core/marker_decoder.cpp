#include "utils/marker_decoder.h"
#include "utils/visualization.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <numeric>
namespace fiducial_detector {
MarkerDecoder::MarkerDecoder(
  cv::Ptr<cv::aruco::Dictionary> dict,
  int cell_pixels, int border_bits, double margin_frac)
: dict_(dict), cell_pixels_(cell_pixels),
  border_bits_(border_bits), margin_frac_(margin_frac)
{}
int MarkerDecoder::getBitsPerSide() const {
  if (!dict_) return 4;
  return dict_->markerSize;
}
int MarkerDecoder::totalSide() const {
  return getBitsPerSide() + 2 * border_bits_;
}
cv::Mat MarkerDecoder::applyAdaptiveThreshold(
  const cv::Mat& gray,
  int win_min, int win_max, int win_step, double constant) const
{
  cv::Mat combined = cv::Mat::zeros(gray.size(), CV_8U);
  for (int win = win_min; win <= win_max; win += win_step) {
    int w = (win % 2 == 0) ? win + 1 : win;
    cv::Mat thresh;
    cv::adaptiveThreshold(gray, thresh, 255,
      cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, w,
      static_cast<int>(constant));
    cv::bitwise_or(combined, thresh, combined);
  }
  return combined;
}
std::vector<std::vector<cv::Point2f>> MarkerDecoder::extractCandidates(
  const cv::Mat& threshold,
  double min_perim_rate, double max_perim_rate,
  double approx_acc, int min_dist_border) const
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(threshold, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  double max_dim = std::max(threshold.cols, threshold.rows);
  double min_perim = max_dim * min_perim_rate;
  double max_perim = max_dim * max_perim_rate;
  std::vector<std::vector<cv::Point2f>> candidates;
  for (const auto& contour : contours) {
    double perim = cv::arcLength(contour, true);
    if (perim < min_perim || perim > max_perim) continue;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, perim * approx_acc, true);
    if (approx.size() != 4) continue;
    if (!cv::isContourConvex(approx)) continue;
    bool too_close = false;
    for (const auto& p : approx) {
      if (p.x < min_dist_border ||
          p.y < min_dist_border ||
          p.x > threshold.cols - 1 - min_dist_border ||
          p.y > threshold.rows - 1 - min_dist_border) {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;
    std::vector<cv::Point2f> quad;
    quad.reserve(4);
    for (const auto& p : approx) quad.emplace_back((float)p.x, (float)p.y);
    candidates.push_back(quad);
  }
  return candidates;
}
cv::Mat MarkerDecoder::perspectiveWarp(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners) const
{
  if (corners.size() != 4) return {};
  int n_total = totalSide();
  int out_size = n_total * cell_pixels_;
  std::vector<cv::Point2f> dst = {
    {0.f,                   0.f},
    {(float)(out_size - 1), 0.f},
    {(float)(out_size - 1), (float)(out_size - 1)},
    {0.f,                   (float)(out_size - 1)}
  };
  cv::Mat H = cv::getPerspectiveTransform(corners, dst);
  cv::Mat warped;
  cv::warpPerspective(gray, warped, H, {out_size, out_size},
                      cv::INTER_LINEAR);
  return warped;
}
std::vector<std::vector<cv::Mat>> MarkerDecoder::segmentCells(
  const cv::Mat& normalized) const
{
  int n_total = totalSide();
  int cell_px = cell_pixels_;
  int margin  = static_cast<int>(std::round(margin_frac_ * cell_px));
  std::vector<std::vector<cv::Mat>> cells(n_total, std::vector<cv::Mat>(n_total));
  for (int r = 0; r < n_total; ++r) {
    for (int c = 0; c < n_total; ++c) {
      int x0 = c * cell_px + margin;
      int y0 = r * cell_px + margin;
      int w  = cell_px - 2 * margin;
      int h  = cell_px - 2 * margin;
      x0 = std::max(0, x0);
      y0 = std::max(0, y0);
      w  = std::min(w, normalized.cols - x0);
      h  = std::min(h, normalized.rows - y0);
      if (w > 0 && h > 0) {
        cells[r][c] = normalized(cv::Rect(x0, y0, w, h));
      } else {
        cells[r][c] = cv::Mat::zeros(1, 1, CV_8U);
      }
    }
  }
  return cells;
}
std::vector<uint8_t> MarkerDecoder::extractBits(
  const std::vector<std::vector<cv::Mat>>& cells) const
{
  int n_total = (int)cells.size();
  std::vector<uint8_t> bits(n_total * n_total, 0);
  for (int r = 0; r < n_total; ++r) {
    for (int c = 0; c < n_total; ++c) {
      const cv::Mat& cell = cells[r][c];
      if (cell.empty()) { bits[r * n_total + c] = 0; continue; }
      cv::Mat binary;
      if (cell.total() < 4) {
        double mean = cv::mean(cell)[0];
        bits[r * n_total + c] = (mean > 127.0) ? 1 : 0;
      } else {
        double otsu_thresh = cv::threshold(
          cell, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        (void)otsu_thresh;
        double white_pct = (double)cv::countNonZero(binary) / (double)binary.total();
        bits[r * n_total + c] = (white_pct > 0.5) ? 1 : 0;
      }
    }
  }
  return bits;
}
bool MarkerDecoder::validateBorderBits(
  const std::vector<uint8_t>& bits,
  int total_side,
  double max_error_rate) const
{
  int n_border = 0, n_wrong = 0;
  for (int r = 0; r < total_side; ++r) {
    for (int c = 0; c < total_side; ++c) {
      bool is_border = (r < border_bits_ || r >= total_side - border_bits_ ||
                        c < border_bits_ || c >= total_side - border_bits_);
      if (!is_border) continue;
      ++n_border;
      if (bits[r * total_side + c] != 0) ++n_wrong;
    }
  }
  if (n_border == 0) return true;
  double error_rate = (double)n_wrong / (double)n_border;
  return error_rate <= max_error_rate;
}
bool MarkerDecoder::matchDictionary(
  const std::vector<uint8_t>& data_bits,
  int& id, int& hamming,
  double error_rate) const
{
  if (!dict_) return false;
  int n_bits  = dict_->markerSize;
  int n_total = n_bits * n_bits;
  if ((int)data_bits.size() < n_total) return false;
  int max_hamming = static_cast<int>(std::floor(n_total * error_rate));
  id      = -1;
  hamming = std::numeric_limits<int>::max();
  int n_markers = dict_->bytesList.rows;
  int bytes_per_marker = dict_->bytesList.cols;
  int n_bytes = (n_total + 7) / 8;
  std::vector<uint8_t> candidate_bytes(n_bytes, 0);
  for (int i = 0; i < n_total; ++i) {
    if (data_bits[i]) {
      candidate_bytes[i / 8] |= (1 << (7 - (i % 8)));
    }
  }
  for (int m = 0; m < n_markers; ++m) {
    const uint8_t* row = dict_->bytesList.ptr<uint8_t>(m);
    int h = 0;
    for (int i = 0; i < std::min(n_bytes, bytes_per_marker); ++i) {
      uint8_t xor_val = candidate_bytes[i] ^ row[i];
      h += __builtin_popcount(xor_val);
    }
    if (h < hamming) {
      hamming = h;
      id = m;
    }
  }
  return (id >= 0 && hamming <= max_hamming);
}
MarkerDecodeDetail MarkerDecoder::decodeMarker(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners) const
{
  MarkerDecodeDetail detail;
  detail.marker_bits = getBitsPerSide();
  cv::Mat normalized = perspectiveWarp(gray, corners);
  if (normalized.empty()) return detail;
  detail.perspective_image = normalized.clone();
  auto cells = segmentCells(normalized);
  int n_total = totalSide();
  detail.bits = extractBits(cells);
  int n_data = getBitsPerSide();
  for (int r = 0; r < n_total; ++r) {
    for (int c = 0; c < n_total; ++c) {
      bool is_border = (r < border_bits_ || r >= n_total - border_bits_ ||
                        c < border_bits_ || c >= n_total - border_bits_);
      if (is_border) detail.border_bits.push_back(detail.bits[r * n_total + c]);
    }
  }
  detail.border_valid = validateBorderBits(detail.bits, n_total);
  std::vector<uint8_t> data_bits;
  data_bits.reserve(n_data * n_data);
  for (int r = border_bits_; r < n_total - border_bits_; ++r) {
    for (int c = border_bits_; c < n_total - border_bits_; ++c) {
      data_bits.push_back(detail.bits[r * n_total + c]);
    }
  }
  bool matched = matchDictionary(data_bits, detail.id, detail.hamming_distance);
  if (!matched) detail.id = -1;
  return detail;
}
RejectedCandidate MarkerDecoder::analyseRejected(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners,
  double max_border_error, double error_rate) const
{
  RejectedCandidate rc;
  rc.corners = corners;
  if (corners.size() != 4) {
    rc.reason = RejectionReason::BAD_CONTOUR;
    return rc;
  }
  cv::Mat normalized = perspectiveWarp(gray, corners);
  if (normalized.empty()) {
    rc.reason = RejectionReason::PERSPECTIVE_FAILURE;
    return rc;
  }
  auto cells = segmentCells(normalized);
  auto bits  = extractBits(cells);
  int n_total = totalSide();
  if (!validateBorderBits(bits, n_total, max_border_error)) {
    rc.reason = RejectionReason::BORDER_FAILURE;
    return rc;
  }
  std::vector<uint8_t> data_bits;
  int n_data = getBitsPerSide();
  for (int r = border_bits_; r < n_total - border_bits_; ++r)
    for (int c = border_bits_; c < n_total - border_bits_; ++c)
      data_bits.push_back(bits[r * n_total + c]);
  int id, hamming;
  bool matched = matchDictionary(data_bits, id, hamming, error_rate);
  rc.hamming_distance = hamming;
  if (!matched) {
    rc.reason = RejectionReason::HAMMING_ERROR;
    return rc;
  }
  rc.reason = RejectionReason::LOW_CONFIDENCE;
  rc.confidence = 1.0f - (float)hamming / (float)(n_data * n_data);
  return rc;
}
cv::Mat MarkerDecoder::visualizeCellGrid(
  const cv::Mat& normalized,
  const std::vector<uint8_t>& bits,
  cv::Size out_size) const
{
  int n_total = totalSide();
  int cell_px = out_size.width / n_total;
  int canvas_sz = cell_px * n_total;
  cv::Mat canvas(canvas_sz + 30, canvas_sz, CV_8UC3, cv::Scalar(40, 40, 40));
  for (int r = 0; r < n_total; ++r) {
    for (int c = 0; c < n_total; ++c) {
      bool is_border = (r < border_bits_ || r >= n_total - border_bits_ ||
                        c < border_bits_ || c >= n_total - border_bits_);
      int x0 = c * cell_px;
      int y0 = r * cell_px;
      cv::Rect cell_rect(x0, y0, cell_px, cell_px);
      uint8_t bit = (r < n_total && c < n_total && !bits.empty())
        ? bits[r * n_total + c] : 0;
      cv::Scalar fill;
      if (is_border) {
        fill = (bit == 0) ? cv::Scalar(80, 80, 80) : cv::Scalar(0, 80, 200);
      } else {
        fill = (bit == 1) ? CLR_CELL_1 : CLR_CELL_0;
      }
      cv::rectangle(canvas, cell_rect, fill, -1);
      if (!normalized.empty() && normalized.rows >= n_total && normalized.cols >= n_total) {
        int src_cell = normalized.cols / n_total;
        cv::Rect src_rect(c * src_cell, r * src_cell, src_cell, src_cell);
        if (src_rect.x + src_rect.width  <= normalized.cols &&
            src_rect.y + src_rect.height <= normalized.rows) {
          cv::Mat src_cell_img = normalized(src_rect);
          cv::Mat resized;
          cv::resize(src_cell_img, resized, {cell_px - 4, cell_px - 4});
          cv::Mat color_cell;
          cv::cvtColor(resized, color_cell, cv::COLOR_GRAY2BGR);
          cv::Mat roi = canvas(cv::Rect(x0 + 2, y0 + 2, cell_px - 4, cell_px - 4));
          cv::addWeighted(color_cell, 0.5, roi, 0.5, 0, roi);
        }
      }
      cv::rectangle(canvas, cell_rect, CLR_CELL_BORDER, 1);
    }
  }
  cv::rectangle(canvas, cv::Rect(0, 0, canvas_sz, canvas_sz), CLR_CELL_BORDER, 3);
  std::string label = "Marker cells (" + std::to_string(n_total) + "x" + std::to_string(n_total) + ")";
  int baseline = 0;
  auto sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
  cv::putText(canvas, label,
    {(canvas_sz - sz.width) / 2, canvas_sz + 20},
    cv::FONT_HERSHEY_SIMPLEX, 0.55, CLR_WHITE, 1, cv::LINE_AA);
  return canvas;
}
cv::Mat MarkerDecoder::visualizeBitMatrix(
  const std::vector<uint8_t>& bits,
  int id,
  int hamming_dist,
  cv::Size out_size) const
{
  int n_data  = getBitsPerSide();
  (void)n_data;
  int n_total = totalSide();
  int cell    = out_size.width / n_total;
  cv::Mat canvas(out_size.height + 50, out_size.width, CV_8UC3, cv::Scalar(20, 20, 20));
  for (int r = 0; r < n_total; ++r) {
    for (int c = 0; c < n_total; ++c) {
      bool is_border = (r < border_bits_ || r >= n_total - border_bits_ ||
                        c < border_bits_ || c >= n_total - border_bits_);
      uint8_t bit = (!bits.empty()) ? bits[r * n_total + c] : 0;
      cv::Rect cell_rect(c * cell, r * cell, cell, cell);
      cv::Scalar fill;
      if (is_border) {
        fill = (bit == 0) ? cv::Scalar(30, 30, 80) : cv::Scalar(0, 0, 200);
      } else {
        fill = (bit == 1) ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 180);
      }
      cv::rectangle(canvas, cell_rect, fill, -1);
      cv::rectangle(canvas, cell_rect, CLR_GRAY, 1);
      std::string bstr = std::to_string((int)bit);
      int bsl = 0;
      auto bsz = cv::getTextSize(bstr, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &bsl);
      cv::putText(canvas, bstr,
        {cell_rect.x + (cell - bsz.width)/2, cell_rect.y + (cell + bsz.height)/2},
        cv::FONT_HERSHEY_SIMPLEX, 0.35, CLR_WHITE, 1, cv::LINE_AA);
    }
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "ID=%d  Hamming=%d", id, hamming_dist);
  cv::putText(canvas, buf, {4, out_size.height + 20},
    cv::FONT_HERSHEY_SIMPLEX, 0.5,
    hamming_dist == 0 ? CLR_ALIGNED : CLR_LOCKED, 1, cv::LINE_AA);
  return canvas;
}
cv::Mat MarkerDecoder::visualizeThreshold(
  const cv::Mat& gray,
  int win_size, double constant) const
{
  cv::Mat thresh = applyAdaptiveThreshold(gray, win_size, win_size, win_size, constant);
  cv::Mat color;
  cv::cvtColor(thresh, color, cv::COLOR_GRAY2BGR);
  cv::putText(color, "Adaptive Threshold", {10, 25},
    cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 200, 200}, 2, cv::LINE_AA);
  return color;
}
cv::Mat MarkerDecoder::visualizeContours(
  const cv::Mat& frame,
  const std::vector<std::vector<cv::Point2f>>& candidates) const
{
  cv::Mat out = frame.clone();
  for (const auto& cand : candidates) {
    for (std::size_t i = 0; i < cand.size(); ++i) {
      cv::line(out, cand[i], cand[(i+1)%cand.size()],
               {0, 180, 255}, 2, cv::LINE_AA);
    }
    for (const auto& p : cand) {
      cv::circle(out, p, 4, {255, 100, 0}, -1, cv::LINE_AA);
    }
  }
  char label[64];
  std::snprintf(label, sizeof(label), "Contour candidates: %zu", candidates.size());
  cv::putText(out, label, {10, 25},
    cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 200, 200}, 2, cv::LINE_AA);
  return out;
}
cv::Mat MarkerDecoder::visualizeRejected(
  const cv::Mat& frame,
  const std::vector<RejectedCandidate>& rejected) const
{
  cv::Mat out = frame.clone();
  for (const auto& rc : rejected) {
    if (rc.corners.size() != 4) continue;
    for (std::size_t i = 0; i < 4; ++i) {
      cv::line(out, rc.corners[i], rc.corners[(i+1)%4],
               {50, 50, 220}, 2, cv::LINE_AA);
    }
    cv::Point2f center(0, 0);
    for (const auto& p : rc.corners) center += p;
    center *= 0.25f;
    std::string reason = rejectionReasonString(rc.reason);
    cv::putText(out, reason,
      {(int)center.x - 30, (int)center.y},
      cv::FONT_HERSHEY_SIMPLEX, 0.4, {50, 50, 255}, 1, cv::LINE_AA);
  }
  char label[64];
  std::snprintf(label, sizeof(label), "Rejected: %zu", rejected.size());
  cv::putText(out, label, {10, 25},
    cv::FONT_HERSHEY_SIMPLEX, 0.65, {50, 50, 220}, 2, cv::LINE_AA);
  return out;
}
MarkerDebugImages MarkerDecoder::generateDebugImages(
  const cv::Mat& frame,
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners,
  const std::vector<RejectedCandidate>& rejected) const
{
  MarkerDebugImages dbg;
  dbg.threshold_image = visualizeThreshold(gray);
  if (corners.size() == 4) {
    cv::Mat norm = perspectiveWarp(gray, corners);
    if (!norm.empty()) {
      dbg.perspective_image = norm.clone();
      auto cells = segmentCells(norm);
      auto bits  = extractBits(cells);
      dbg.cell_grid_image  = visualizeCellGrid(norm, bits);
      MarkerDecodeDetail det = decodeMarker(gray, corners);
      dbg.bit_matrix_image = visualizeBitMatrix(
        det.bits, det.id, det.hamming_distance);
    }
  }
  auto thresh = applyAdaptiveThreshold(gray);
  auto cands  = extractCandidates(thresh);
  dbg.contour_image = visualizeContours(frame, cands);
  dbg.rejected_image = visualizeRejected(frame, rejected);
  dbg.valid = true;
  return dbg;
}
}
