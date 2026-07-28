#include "utils/visualization.h"
#include <cmath>
#include <cstdio>

namespace fiducial_detector {

const cv::Scalar CLR_ARUCO      {  0, 220,   0};
const cv::Scalar CLR_REJECTED   { 50,  50, 220};
const cv::Scalar CLR_LOCKED     {220, 220,   0};
const cv::Scalar CLR_ALIGNED    {  0, 255,   0};
const cv::Scalar CLR_UNALIGNED  {  0,   0, 255};
const cv::Scalar CLR_WHITE      {255, 255, 255};
const cv::Scalar CLR_BLACK      {  0,   0,   0};
const cv::Scalar CLR_GRAY       {130, 130, 130};
const cv::Scalar CLR_CELL_0     { 80,  80,  80};
const cv::Scalar CLR_CELL_1     {255, 255, 255};
const cv::Scalar CLR_CELL_BORDER{220,  20,  20};

Visualizer::Visualizer(int alignment_tolerance)
    : alignment_tol_(alignment_tolerance)
{}

void Visualizer::alphaRect(cv::Mat& frame, cv::Rect rect,
                           cv::Scalar color, double alpha) const
{
    cv::Mat roi = frame(rect & cv::Rect(0, 0, frame.cols, frame.rows));
    cv::Mat overlay = roi.clone();
    cv::rectangle(overlay, cv::Rect(0, 0, roi.cols, roi.rows), color, -1);
    cv::addWeighted(overlay, alpha, roi, 1.0 - alpha, 0, roi);
}

void Visualizer::labelText(cv::Mat& frame, const std::string& text,
                           cv::Point origin, cv::Scalar color,
                           double fs, int th) const
{
    int baseline = 0;
    auto sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fs, th, &baseline);
    cv::Rect bg(origin.x - 2, origin.y - sz.height - 2,
                sz.width + 4, sz.height + baseline + 4);
    bg &= cv::Rect(0, 0, frame.cols, frame.rows);
    cv::rectangle(frame, bg, CLR_BLACK, -1);
    cv::putText(frame, text, origin,
                cv::FONT_HERSHEY_SIMPLEX, fs, color, th, cv::LINE_AA);
}

bool Visualizer::isAligned(cv::Point2f pt, const cv::Size& sz) const {
    float cx = sz.width * 0.5f, cy = sz.height * 0.5f;
    return std::abs(pt.x - cx) <= alignment_tol_
        && std::abs(pt.y - cy) <= alignment_tol_;
}
bool Visualizer::isLeft (cv::Point2f pt, const cv::Size& sz) const {
    return pt.x < sz.width  * 0.5f - alignment_tol_;
}
bool Visualizer::isRight(cv::Point2f pt, const cv::Size& sz) const {
    return pt.x > sz.width  * 0.5f + alignment_tol_;
}
bool Visualizer::isUp   (cv::Point2f pt, const cv::Size& sz) const {
    return pt.y < sz.height * 0.5f - alignment_tol_;
}
bool Visualizer::isDown (cv::Point2f pt, const cv::Size& sz) const {
    return pt.y > sz.height * 0.5f + alignment_tol_;
}

std::string Visualizer::alignmentString(cv::Point2f pt, const cv::Size& sz) const {
    if (isAligned(pt, sz)) return "POSISI_CENTERING";
    std::string h, v;
    if      (isLeft(pt, sz))  h = "GESER_KIRI";
    else if (isRight(pt, sz)) h = "GESER_KANAN";
    if      (isUp(pt, sz))    v = "MUNDUR";
    else if (isDown(pt, sz))  v = "MAJU";
    if (h.empty()) return v;
    if (v.empty()) return h;
    return h + "|" + v;
}

void Visualizer::drawUI(cv::Mat& frame, bool any_locked) const {
    (void)any_locked;
    int W = frame.cols, H = frame.rows;
    int cx = W / 2, cy = H / 2, tol = alignment_tol_;
    int arm = 20;
    cv::line(frame, {cx - arm, cy}, {cx + arm, cy}, CLR_BLACK, 1, cv::LINE_AA);
    cv::line(frame, {cx, cy - arm}, {cx, cy + arm}, CLR_BLACK, 1, cv::LINE_AA);
    cv::Rect box(cx - tol, cy - tol, 2 * tol, 2 * tol);
    box &= cv::Rect(0, 0, W, H);
    cv::rectangle(frame, box, CLR_BLACK, 1, cv::LINE_AA);
}

void Visualizer::drawDetectedMarkers(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    MarkerType /*type*/) const
{
    if (corners.empty()) return;
    cv::aruco::drawDetectedMarkers(frame, corners, ids, CLR_ARUCO);
    for (const auto& c : corners) {
        cv::Point2f center(0, 0);
        for (const auto& p : c) center += p;
        center *= 0.25f;
        cv::circle(frame, center, 6, CLR_ARUCO, -1, cv::LINE_AA);
    }
}

void Visualizer::drawRejected(
    cv::Mat& frame,
    const std::vector<std::vector<cv::Point2f>>& rejected) const
{
    if (rejected.empty()) return;
    std::vector<int> empty_ids;
    cv::aruco::drawDetectedMarkers(frame, rejected, empty_ids, CLR_REJECTED);
}

void Visualizer::drawPoseAxis(
    cv::Mat& frame,
    const PoseResult& pose,
    const cv::Mat& K,
    const cv::Mat& D,
    float axis_length) const
{
    if (!pose.valid || K.empty()) return;
    cv::aruco::drawAxis(frame, K, D, pose.rvec, pose.tvec, axis_length);
}

void Visualizer::drawMarkerInfo(
    cv::Mat& frame,
    cv::Point2f center,
    int id,
    MarkerType /*type*/,
    const PoseResult& pose) const
{
    int x = static_cast<int>(center.x) + 12;
    int y = static_cast<int>(center.y) + 12;
    auto line = [&](const std::string& s) {
        labelText(frame, s, {x, y}, CLR_ARUCO, 0.44, 1);
        y += 15;
    };
    line("ID: " + std::to_string(id) + "  [7x7]");
    line("Center: (" + std::to_string(static_cast<int>(center.x))
         + "," + std::to_string(static_cast<int>(center.y)) + ")");
    if (pose.valid) {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "Pose: (%.3f, %.3f, %.3f) m",
            pose.tvec[0], pose.tvec[1], pose.tvec[2]);
        line(buf);
        std::snprintf(buf, sizeof(buf), "Dist: %.3f m", pose.distance);
        line(buf);
    }
}

void Visualizer::drawAlignment(
    cv::Mat& frame,
    cv::Point2f marker_center,
    const cv::Size& sz) const
{
    int W = sz.width, H = sz.height;
    int cx = W / 2, cy = H / 2, tol = alignment_tol_;
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fs = 0.75;
    int th = 2;
    if (isAligned(marker_center, sz)) {
        labelText(frame, "POSISI CENTERING",
                  {cx - 75, cy - tol - 22}, CLR_LOCKED, 0.85, 2);
        labelText(frame, "POSISI CENTERING",
                  {cx - 85, cy + tol + 40}, CLR_ALIGNED, 0.78, 2);
        return;
    }
    if (isLeft(marker_center, sz))
        cv::putText(frame, "GESER KIRI",  {18, cy},       font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
    else if (isRight(marker_center, sz))
        cv::putText(frame, "GESER KANAN", {W - 175, cy},  font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
    if (isUp(marker_center, sz))
        cv::putText(frame, "NAIK",  {cx - 35, 38},    font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
    else if (isDown(marker_center, sz))
        cv::putText(frame, "TURUN", {cx - 42, H - 22}, font, fs, CLR_UNALIGNED, th, cv::LINE_AA);
}

void Visualizer::drawHUD(
    cv::Mat& frame,
    float /*fps*/,
    double latency_ms,
    uint64_t frame_count,
    bool cam_ok) const
{
    char buf[128];
    int y = 25;
    auto hline = [&](const std::string& s, cv::Scalar c) {
        cv::putText(frame, s, {10, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.58, c, 2, cv::LINE_AA);
        y += 22;
    };
    std::snprintf(buf, sizeof(buf), "Latency: %.1f ms", latency_ms);
    hline(buf, CLR_GRAY);
    std::snprintf(buf, sizeof(buf), "Frame: %llu", static_cast<unsigned long long>(frame_count));
    hline(buf, CLR_GRAY);
    std::snprintf(buf, sizeof(buf), "CAM: %s", cam_ok ? "OK" : "RECONNECTING...");
    hline(buf, cam_ok ? CLR_ALIGNED : CLR_UNALIGNED);
}

void Visualizer::drawArrow(cv::Mat& f, cv::Point from, cv::Point to,
                           cv::Scalar c, int th, double frac) const
{
    cv::line(f, from, to, c, th, cv::LINE_AA);
    double dx = to.x - from.x, dy = to.y - from.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1) return;
    double ul = frac * len, angle = std::atan2(dy, dx);
    auto tip = [&](double a) {
        return cv::Point(static_cast<int>(to.x - ul * std::cos(a)),
                         static_cast<int>(to.y - ul * std::sin(a)));
    };
    cv::line(f, to, tip(angle + 0.4), c, th, cv::LINE_AA);
    cv::line(f, to, tip(angle - 0.4), c, th, cv::LINE_AA);
}

void Visualizer::drawRejectedWithReason(
    cv::Mat& f,
    const std::vector<RejectedCandidate>& rej) const
{
    for (const auto& rc : rej) {
        if (rc.corners.size() != 4) continue;
        for (int i = 0; i < 4; ++i)
            cv::line(f, rc.corners[i], rc.corners[(i + 1) % 4],
                     CLR_REJECTED, 2, cv::LINE_AA);
        cv::Point2f c(0, 0);
        for (auto& p : rc.corners) c += p;
        c *= 0.25f;
        labelText(f, rejectionReasonString(rc.reason),
                  {static_cast<int>(c.x) - 30, static_cast<int>(c.y)},
                  {50, 50, 255}, 0.38, 1);
    }
}

void Visualizer::drawCornerLabels(
    cv::Mat& f,
    const std::vector<cv::Point2f>& corners,
    cv::Scalar c) const
{
    if (corners.size() < 4) return;
    static const char* lbl[] = {"TL", "TR", "BR", "BL"};
    for (int i = 0; i < 4; ++i) {
        cv::circle(f, corners[i], 5, c, -1, cv::LINE_AA);
        labelText(f, lbl[i],
                  {static_cast<int>(corners[i].x) + 4,
                   static_cast<int>(corners[i].y) - 4}, c, 0.4, 1);
    }
}

void Visualizer::drawOrientationArrow(
    cv::Mat& f,
    const std::vector<cv::Point2f>& corners,
    cv::Scalar c) const
{
    if (corners.size() < 4) return;
    cv::Point2f center(0, 0);
    for (auto& p : corners) center += p;
    center *= 0.25f;
    drawArrow(f, static_cast<cv::Point>(center),
              static_cast<cv::Point>(corners[0]), c, 2);
}

void Visualizer::drawPoseXYZLabeled(
    cv::Mat& f,
    const PoseResult& p,
    const cv::Mat& K,
    const cv::Mat& D,
    float len) const
{
    if (!p.valid || K.empty()) return;
    std::vector<cv::Point3f> pts3 = {{len,0,0},{0,len,0},{0,0,len},{0,0,0}};
    std::vector<cv::Point2f> pts2;
    cv::projectPoints(pts3, p.rvec, p.tvec, K, D, pts2);
    if (pts2.size() < 4) return;
    cv::Point o = pts2[3];
    cv::line(f, o, pts2[0], {0,0,220},   3, cv::LINE_AA);
    cv::putText(f, "X", pts2[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255},   2, cv::LINE_AA);
    cv::line(f, o, pts2[1], {0,220,0},   3, cv::LINE_AA);
    cv::putText(f, "Y", pts2[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0},   2, cv::LINE_AA);
    cv::line(f, o, pts2[2], {220,0,0},   3, cv::LINE_AA);
    cv::putText(f, "Z", pts2[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,0,0},   2, cv::LINE_AA);
}

void Visualizer::drawAdvancedHUD(
    cv::Mat& f,
    float fps,
    double lat_ms,
    uint64_t fc,
    bool cam_ok,
    float mk_m,
    float conf_pct) const
{
    int y = 22;
    char buf[128];
    auto hl = [&](const std::string& s, cv::Scalar c, double fs = 0.52) {
        cv::putText(f, s, {10, y}, cv::FONT_HERSHEY_SIMPLEX, fs, c, 2, cv::LINE_AA);
        y += 20;
    };
    std::snprintf(buf, sizeof(buf), "FPS:%.1f Lat:%.1fms", fps, lat_ms);
    hl(buf, CLR_WHITE);
    std::snprintf(buf, sizeof(buf), "Frame:%llu", static_cast<unsigned long long>(fc));
    hl(buf, CLR_GRAY, 0.44);
    hl(std::string("CAM:") + (cam_ok ? "OK" : "LOST"),
       cam_ok ? CLR_ALIGNED : CLR_REJECTED, 0.44);
    hl("Dict:DICT_7X7_50", CLR_LOCKED, 0.44);
    if (mk_m > 0) {
        std::snprintf(buf, sizeof(buf), "Marker:%.0fmm", mk_m * 1000.f);
        hl(buf, CLR_GRAY, 0.40);
    }
    if (conf_pct >= 0) {
        std::snprintf(buf, sizeof(buf), "Conf:%.0f%%", conf_pct * 100.f);
        hl(buf, (conf_pct > 0.7f) ? CLR_ALIGNED
                                   : (conf_pct > 0.4f) ? CLR_LOCKED : CLR_REJECTED, 0.44);
    }
}

void Visualizer::drawCenterLockBox(
    cv::Mat& f, bool locked, uint64_t fc) const
{
    int W = f.cols, H = f.rows, cx = W / 2, cy = H / 2;
    // Visual box = 50% dari alignment_tol_ agar lebih kecil dan presisi
    int vis = alignment_tol_ / 2;
    cv::Scalar oc = locked ? CLR_LOCKED : CLR_UNALIGNED;
    int lw = locked ? (2 + (int)(fc % 4 < 2 ? 1 : 0)) : 2;
    cv::Rect box(cx - vis, cy - vis, 2 * vis, 2 * vis);
    box &= cv::Rect(0, 0, W, H);
    alphaRect(f, box, oc, locked ? 0.15 : 0.08);
    cv::rectangle(f, box, oc, lw, cv::LINE_AA);
    int tick = 8, arm = 12;
    auto ct = [&](cv::Point p, int dx, int dy) {
        cv::line(f, p, {p.x + dx * tick, p.y}, oc, 2, cv::LINE_AA);
        cv::line(f, p, {p.x, p.y + dy * tick}, oc, 2, cv::LINE_AA);
    };
    ct({cx - vis, cy - vis},  1,  1);
    ct({cx + vis, cy - vis}, -1,  1);
    ct({cx + vis, cy + vis}, -1, -1);
    ct({cx - vis, cy + vis},  1, -1);
    cv::line(f, {cx - arm, cy}, {cx + arm, cy}, CLR_WHITE, 1, cv::LINE_AA);
    cv::line(f, {cx, cy - arm}, {cx, cy + arm}, CLR_WHITE, 1, cv::LINE_AA);
    cv::circle(f, {cx, cy}, 3, locked ? CLR_LOCKED : CLR_WHITE, -1, cv::LINE_AA);
    if (locked) labelText(f, "GATE READY", {cx - 38, cy - vis - 8}, CLR_LOCKED, 0.55, 1);
}

void Visualizer::drawMarkerCells(
    cv::Mat& f, const cv::Mat& /*norm*/,
    const std::vector<uint8_t>& bits,
    cv::Point pos, int size, int n_side, int border) const
{
    int nt = n_side + 2 * border;
    int cpx = size / nt;
    if (cpx < 2) return;
    cv::Rect bg(pos.x, pos.y, nt * cpx, nt * cpx + 20);
    bg &= cv::Rect(0, 0, f.cols, f.rows);
    alphaRect(f, bg, CLR_BLACK, 0.7);
    for (int r = 0; r < nt; ++r) {
        for (int c = 0; c < nt; ++c) {
            bool bdr = (r < border || r >= nt - border || c < border || c >= nt - border);
            int x0 = pos.x + c * cpx, y0 = pos.y + r * cpx;
            cv::Rect cr(x0, y0, cpx, cpx);
            cr &= cv::Rect(0, 0, f.cols, f.rows);
            if (cr.area() <= 0) continue;
            uint8_t bit = (!bits.empty() && r * nt + c < (int)bits.size())
                          ? bits[r * nt + c] : 0;
            cv::Scalar fill = bdr
                ? (bit == 0 ? cv::Scalar(60,60,60) : cv::Scalar(0,60,200))
                : (bit == 1 ? CLR_CELL_1 : CLR_CELL_0);
            cv::rectangle(f, cr, fill, -1);
            cv::rectangle(f, cr, CLR_CELL_BORDER, 1);
        }
    }
    cv::rectangle(f,
        cv::Rect(pos.x, pos.y, nt * cpx, nt * cpx) & cv::Rect(0, 0, f.cols, f.rows),
        CLR_CELL_BORDER, 2);
    cv::putText(f, "Marker cells", {pos.x, pos.y + nt * cpx + 15},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, CLR_WHITE, 1, cv::LINE_AA);
}

void Visualizer::drawBitMatrix(
    cv::Mat& f,
    const std::vector<uint8_t>& bits,
    int id,
    int hdist,
    cv::Point pos,
    int cs,
    int n_side) const
{
    int nt = n_side + 2;
    for (int r = 0; r < nt; ++r) {
        for (int c = 0; c < nt; ++c) {
            uint8_t bit = (!bits.empty() && r * nt + c < (int)bits.size())
                          ? bits[r * nt + c] : 0;
            cv::Rect cr(pos.x + c * cs, pos.y + r * cs, cs, cs);
            cr &= cv::Rect(0, 0, f.cols, f.rows);
            if (cr.area() <= 0) continue;
            cv::rectangle(f, cr,
                bit == 1 ? cv::Scalar(0,180,0) : cv::Scalar(0,0,150), -1);
            cv::rectangle(f, cr, CLR_GRAY, 1);
            cv::putText(f, std::to_string(static_cast<int>(bit)),
                        {cr.x + cr.width / 4, cr.y + cr.height * 3 / 4},
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, CLR_WHITE, 1, cv::LINE_AA);
        }
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "ID=%d H=%d", id, hdist);
    labelText(f, buf, {pos.x, pos.y + nt * cs + 12},
              hdist == 0 ? CLR_ALIGNED : CLR_LOCKED, 0.42, 1);
}

void Visualizer::drawThresholdOverlay(
    cv::Mat& f, const cv::Mat& th, double alpha) const
{
    if (th.empty()) return;
    cv::Mat c;
    if (th.channels() == 1) cv::cvtColor(th, c, cv::COLOR_GRAY2BGR);
    else c = th;
    cv::Mat r;
    if (c.size() != f.size()) cv::resize(c, r, f.size());
    else r = c;
    cv::addWeighted(r, alpha, f, 1.0 - alpha, 0, f);
}

} // namespace fiducial_detector
