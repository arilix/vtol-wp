#include "utils/aruco.h"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>
#include <vector>

static const int DICT_ID = cv::aruco::DICT_7X7_50;
static const int NUM_MARKERS = 50;
static const int IMG_SIZE = 400;
static const int BORDER_BITS = 1;

static cv::Mat generateMarkerImage(cv::Ptr<cv::aruco::Dictionary> dict, int id)
{
  cv::Mat img;
  cv::aruco::drawMarker(dict, id, IMG_SIZE - 40, img, BORDER_BITS);

  cv::Mat bordered;
  cv::copyMakeBorder(
    img, bordered, 20, 20, 20, 20, cv::BORDER_CONSTANT, cv::Scalar(255));
  return bordered;
}

static cv::Ptr<cv::aruco::DetectorParameters> make7x7Params()
{
  auto params = fiducial_opencv_compat::makeDetectorParameters();
  params->perspectiveRemovePixelPerCell = 10;
  params->perspectiveRemoveIgnoredMarginPerCell = 0.10;
  params->markerBorderBits = BORDER_BITS;
  params->adaptiveThreshWinSizeMin = 3;
  params->adaptiveThreshWinSizeMax = 33;
  params->adaptiveThreshWinSizeStep = 10;
  params->adaptiveThreshConstant = 7.0;
  params->minMarkerPerimeterRate = 0.015;
  params->maxMarkerPerimeterRate = 4.0;
  params->polygonalApproxAccuracyRate = 0.03;
  params->minCornerDistanceRate = 0.05;
  params->minDistanceToBorder = 3;
  params->maxErroneousBitsInBorderRate = 0.35;
  params->errorCorrectionRate = 0.6;
  params->detectInvertedMarker = true;
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params->cornerRefinementWinSize = 5;
  params->cornerRefinementMaxIterations = 50;
  params->cornerRefinementMinAccuracy = 0.01;
  return params;
}

int main(int, char **)
{
  const std::string sep(54, '=');
  const std::string dash(54, '-');

  std::printf("\n%s\n", sep.c_str());
  std::printf("  DICT_7X7_50 Validation - ID 0 to %d\n", NUM_MARKERS - 1);
  std::printf("%s\n", dash.c_str());

  auto dict = fiducial_opencv_compat::getPredefinedDictionary(DICT_ID);
  auto params = make7x7Params();

  if (!dict || dict->bytesList.empty()) {
    std::printf("  FAILED: dictionary not loaded\n%s\n\n", sep.c_str());
    return 1;
  }
  if (dict->bytesList.rows != NUM_MARKERS) {
    std::printf(
      "  FAILED: expected %d markers, got %d\n%s\n\n",
      NUM_MARKERS, dict->bytesList.rows, sep.c_str());
    return 1;
  }

  std::printf(
    "  Dictionary: DICT_7X7_50 | Markers: %d | BorderBits: %d\n",
    dict->bytesList.rows, BORDER_BITS);
  std::printf("%s\n", dash.c_str());

  int pass_count = 0;
  std::vector<int> failed_ids;

  for (int id = 0; id < NUM_MARKERS; ++id) {
    cv::Mat marker_img = generateMarkerImage(dict, id);
    const cv::Mat & gray = marker_img;

    std::vector<int> detected_ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, dict, corners, detected_ids, params, rejected);

    const bool pass = detected_ids.size() == 1 && detected_ids[0] == id;
    if (pass) {
      ++pass_count;
    } else {
      failed_ids.push_back(id);
    }

    std::printf("  ID %2d -> %s\n", id, pass ? "PASS" : "FAIL");
  }

  std::printf("%s\n", dash.c_str());
  if (pass_count == NUM_MARKERS) {
    std::printf("  Result: %d/%d PASS\n", pass_count, NUM_MARKERS);
    std::printf("%s\n\n", sep.c_str());
    return 0;
  }

  std::printf("  Result: %d/%d PASS - FAILED IDs:", pass_count, NUM_MARKERS);
  for (int failed_id : failed_ids) {
    std::printf(" %d", failed_id);
  }
  std::printf("\n%s\n\n", sep.c_str());
  return 1;
}
