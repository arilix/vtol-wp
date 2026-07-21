#pragma once
#pragma GCC system_header

#if __has_include(<opencv2/objdetect/aruco_detector.hpp>)
#include <opencv2/objdetect/aruco_detector.hpp>
#define FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR 1
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include_next <opencv2/aruco.hpp>
#pragma GCC diagnostic pop
#define FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR 0
#endif
#include <opencv2/calib3d.hpp>

#if FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR
namespace cv {
namespace aruco {

using PREDEFINED_DICTIONARY_NAME = PredefinedDictionaryType;

inline void detectMarkers(
  cv::InputArray image,
  const cv::Ptr<cv::aruco::Dictionary>& dictionary,
  cv::OutputArrayOfArrays corners,
  cv::OutputArray ids,
  const cv::Ptr<cv::aruco::DetectorParameters>& parameters = cv::makePtr<cv::aruco::DetectorParameters>(),
  cv::OutputArrayOfArrays rejectedImgPoints = cv::noArray())
{
  cv::aruco::ArucoDetector detector(*dictionary, *parameters);
  detector.detectMarkers(image, corners, ids, rejectedImgPoints);
}

inline void drawMarker(
  const cv::Ptr<cv::aruco::Dictionary>& dictionary,
  int id,
  int sidePixels,
  cv::OutputArray img,
  int borderBits = 1)
{
  cv::aruco::generateImageMarker(*dictionary, id, sidePixels, img, borderBits);
}

inline void drawAxis(
  cv::InputOutputArray image,
  cv::InputArray cameraMatrix,
  cv::InputArray distCoeffs,
  cv::InputArray rvec,
  cv::InputArray tvec,
  float length)
{
  cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, length);
}

}  // namespace aruco
}  // namespace cv
#else
namespace cv {
namespace aruco {

inline void drawAxis(
  cv::InputOutputArray image,
  cv::InputArray cameraMatrix,
  cv::InputArray distCoeffs,
  cv::InputArray rvec,
  cv::InputArray tvec,
  float length)
{
  cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, length);
}

}  // namespace aruco
}  // namespace cv
#endif

namespace fiducial_opencv_compat {

inline cv::Ptr<cv::aruco::DetectorParameters> makeDetectorParameters()
{
#if FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR
  return cv::makePtr<cv::aruco::DetectorParameters>();
#else
  return cv::aruco::DetectorParameters::create();
#endif
}

inline cv::Ptr<cv::aruco::Dictionary> getPredefinedDictionary(
  cv::aruco::PREDEFINED_DICTIONARY_NAME dict)
{
#if FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR
  return cv::makePtr<cv::aruco::Dictionary>(
    cv::aruco::getPredefinedDictionary(dict));
#else
  return cv::aruco::getPredefinedDictionary(dict);
#endif
}

inline cv::Ptr<cv::aruco::Dictionary> getPredefinedDictionary(int dict)
{
#if FIDUCIAL_OPENCV_HAS_ARUCO_DETECTOR
  return cv::makePtr<cv::aruco::Dictionary>(
    cv::aruco::getPredefinedDictionary(dict));
#else
  return cv::aruco::getPredefinedDictionary(
    static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dict));
#endif
}

}  // namespace fiducial_opencv_compat
