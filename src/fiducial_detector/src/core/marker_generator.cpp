#include "utils/marker_generator.h"
#include "utils/aruco.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cerrno>
#include <sys/stat.h>

namespace fiducial_detector {

static bool ensureDir(const std::string& path) {
    struct stat st{};
    if (stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    std::size_t pos = path.rfind('/');
    if (pos != std::string::npos && pos > 0) {
        if (!ensureDir(path.substr(0, pos))) return false;
    }
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

int MarkerGenerator::generateDictionarySet(
    const std::string& dict_name,
    const std::string& output_dir,
    int marker_size_px,
    int border_bits)
{
    if (dict_name != "DICT_7X7_50") {
        std::fprintf(stderr, "[MarkerGenerator] Only DICT_7X7_50 is supported\n");
        return -1;
    }
    auto dict  = fiducial_opencv_compat::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    int  total = dict->bytesList.rows;

    if (!ensureDir(output_dir)) {
        std::fprintf(stderr, "[MarkerGenerator] Cannot create directory: %s\n",
            output_dir.c_str());
        return -1;
    }

    int written = 0;
    for (int id = 0; id < total; ++id) {
        cv::Mat img;
        cv::aruco::drawMarker(dict, id, marker_size_px, img, border_bits);
        int pad = marker_size_px / 8;
        cv::Mat padded(img.rows + 2 * pad, img.cols + 2 * pad, CV_8UC1, cv::Scalar(255));
        img.copyTo(padded(cv::Rect(pad, pad, img.cols, img.rows)));
        char fname[512];
        std::snprintf(fname, sizeof(fname), "%s/marker_%d.png", output_dir.c_str(), id);
        if (cv::imwrite(fname, padded)) ++written;
        else std::fprintf(stderr, "[MarkerGenerator] Failed to write: %s\n", fname);
    }
    return written;
}

bool MarkerGenerator::validateGeneratedDictionary(
    const std::string& dict_name,
    const std::string& marker_dir,
    std::vector<MarkerValidationResult>& results)
{
    if (dict_name != "DICT_7X7_50") {
        std::fprintf(stderr, "[MarkerGenerator] Only DICT_7X7_50 is supported\n");
        return false;
    }
    auto dict  = fiducial_opencv_compat::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    int  total = dict->bytesList.rows;

    auto dp = fiducial_opencv_compat::makeDetectorParameters();
    // Perspective normalization optimized for 7×7 grid resolution
    dp->perspectiveRemovePixelPerCell         = 10;
    dp->perspectiveRemoveIgnoredMarginPerCell = 0.10;
    dp->adaptiveThreshWinSizeMin              = 3;
    dp->adaptiveThreshWinSizeMax              = 33;
    dp->adaptiveThreshWinSizeStep             = 10;
    dp->minMarkerPerimeterRate                = 0.02;
    dp->errorCorrectionRate                   = 0.6;
    dp->cornerRefinementMethod                = cv::aruco::CORNER_REFINE_SUBPIX;

    results.clear();
    results.reserve(total);
    bool all_pass = true;

    for (int id = 0; id < total; ++id) {
        char fname[512];
        std::snprintf(fname, sizeof(fname), "%s/marker_%d.png", marker_dir.c_str(), id);
        MarkerValidationResult r;
        r.id = id;
        cv::Mat img = cv::imread(fname, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            r.pass = false; r.reason = "file_not_found";
            results.push_back(r); all_pass = false; continue;
        }
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(img, dict, corners, ids, dp, rejected);
        if (ids.empty()) {
            r.pass = false; r.reason = "not_detected"; all_pass = false;
        } else if (ids[0] != id) {
            r.pass = false; r.hamming = std::abs(ids[0] - id);
            r.reason = "id_mismatch(got=" + std::to_string(ids[0]) + ")"; all_pass = false;
        } else {
            r.pass = true; r.hamming = 0; r.reason = "ok";
        }
        results.push_back(r);
    }
    return all_pass;
}

void MarkerGenerator::printValidationReport(
    const std::string& dict_name,
    const std::vector<MarkerValidationResult>& results)
{
    const std::string sep(54, '=');
    const std::string dash(54, '-');
    std::printf("\n%s\n  DICT_7X7_50 TEST: %s\n%s\n",
        sep.c_str(), dict_name.c_str(), dash.c_str());
    int pass_count = 0, fail_count = 0;
    for (const auto& r : results) {
        if (r.pass) { std::printf("  ID %3d → PASS\n", r.id); ++pass_count; }
        else        { std::printf("  ID %3d → FAIL  [%s]\n", r.id, r.reason.c_str()); ++fail_count; }
    }
    std::printf("%s\n  Result: %d/%d PASS %s\n%s\n\n",
        dash.c_str(), pass_count, (int)results.size(),
        fail_count == 0 ? "✓" : "✗", sep.c_str());
}

} // namespace fiducial_detector
