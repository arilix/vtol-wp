#include "utils/detector_parameters.h"

namespace fiducial_detector {

DetectorParametersManager::DetectorParametersManager()
    : params_(fiducial_opencv_compat::makeDetectorParameters())
{}

void DetectorParametersManager::declareAll(rclcpp::Node* node) {
    node->declare_parameter("adaptiveThreshWinSizeMin",             3);
    node->declare_parameter("adaptiveThreshWinSizeMax",            33);
    node->declare_parameter("adaptiveThreshWinSizeStep",           10);
    node->declare_parameter("adaptiveThreshConstant",             7.0);
    node->declare_parameter("minMarkerPerimeterRate",            0.015);
    node->declare_parameter("maxMarkerPerimeterRate",              4.0);
    node->declare_parameter("polygonalApproxAccuracyRate",        0.03);
    node->declare_parameter("minCornerDistanceRate",              0.05);
    node->declare_parameter("minDistanceToBorder",                   3);
    node->declare_parameter("minMarkerDistanceRate",              0.05);
    node->declare_parameter("markerBorderBits",                      1);
    node->declare_parameter("perspectiveRemovePixelPerCell",         10);
    node->declare_parameter("perspectiveRemoveIgnoredMarginPerCell", 0.10);
    node->declare_parameter("maxErroneousBitsInBorderRate",        0.35);
    node->declare_parameter("errorCorrectionRate",                  0.6);
    node->declare_parameter("detectInvertedMarker",                true);
    node->declare_parameter("cornerRefinementMethod",                1);  // SUBPIX
    node->declare_parameter("cornerRefinementWinSize",               5);
    node->declare_parameter("cornerRefinementMaxIterations",        50);
    node->declare_parameter("cornerRefinementMinAccuracy",         0.01);
}

void DetectorParametersManager::bind(rclcpp::Node* node) {
    params_->adaptiveThreshWinSizeMin    = node->get_parameter("adaptiveThreshWinSizeMin").as_int();
    params_->adaptiveThreshWinSizeMax    = node->get_parameter("adaptiveThreshWinSizeMax").as_int();
    params_->adaptiveThreshWinSizeStep   = node->get_parameter("adaptiveThreshWinSizeStep").as_int();
    params_->adaptiveThreshConstant      = node->get_parameter("adaptiveThreshConstant").as_double();
    params_->minMarkerPerimeterRate      = node->get_parameter("minMarkerPerimeterRate").as_double();
    params_->maxMarkerPerimeterRate      = node->get_parameter("maxMarkerPerimeterRate").as_double();
    params_->polygonalApproxAccuracyRate = node->get_parameter("polygonalApproxAccuracyRate").as_double();
    params_->minCornerDistanceRate       = node->get_parameter("minCornerDistanceRate").as_double();
    params_->minDistanceToBorder         = node->get_parameter("minDistanceToBorder").as_int();
    params_->minMarkerDistanceRate       = node->get_parameter("minMarkerDistanceRate").as_double();
    params_->markerBorderBits            = node->get_parameter("markerBorderBits").as_int();
    params_->perspectiveRemovePixelPerCell =
        node->get_parameter("perspectiveRemovePixelPerCell").as_int();
    params_->perspectiveRemoveIgnoredMarginPerCell =
        node->get_parameter("perspectiveRemoveIgnoredMarginPerCell").as_double();
    params_->maxErroneousBitsInBorderRate =
        node->get_parameter("maxErroneousBitsInBorderRate").as_double();
    params_->errorCorrectionRate =
        node->get_parameter("errorCorrectionRate").as_double();
    params_->detectInvertedMarker =
        node->get_parameter("detectInvertedMarker").as_bool();
    int cr = node->get_parameter("cornerRefinementMethod").as_int();
    params_->cornerRefinementMethod =
        static_cast<cv::aruco::CornerRefineMethod>(cr);
    params_->cornerRefinementWinSize =
        node->get_parameter("cornerRefinementWinSize").as_int();
    params_->cornerRefinementMaxIterations =
        node->get_parameter("cornerRefinementMaxIterations").as_int();
    params_->cornerRefinementMinAccuracy =
        node->get_parameter("cornerRefinementMinAccuracy").as_double();
}

void DetectorParametersManager::apply7x7Profile() {
    // Perspective normalization
    // Margin 0.13 (lebih besar dari default 0.10) agar border yang berbagi dengan
    // marker tetangga tidak ikut terbaca sebagai bit data
    params_->perspectiveRemovePixelPerCell          = 10;
    params_->perspectiveRemoveIgnoredMarginPerCell  = 0.13;
    params_->markerBorderBits                       = 1;

    // Adaptive threshold: step=4 (lebih rapat dari 10) agar lebih banyak window size
    // yang dicoba — kritis untuk marker tergabung yang memiliki variasi kontras tinggi
    params_->adaptiveThreshWinSizeMin               = 3;
    params_->adaptiveThreshWinSizeMax               = 53;
    params_->adaptiveThreshWinSizeStep              = 4;
    params_->adaptiveThreshConstant                 = 7.0;

    // minMarkerDistanceRate = 0.01 (dari 0.05) — membolehkan marker yang sangat
    // berdekatan/berbagi border untuk sama-sama terdeteksi tanpa saling di-reject
    params_->minMarkerPerimeterRate                 = 0.015;
    params_->maxMarkerPerimeterRate                 = 4.0;
    params_->polygonalApproxAccuracyRate            = 0.03;
    params_->minCornerDistanceRate                  = 0.05;
    params_->minDistanceToBorder                    = 1;   // dari 3, agar marker di pinggir frame tetap terdeteksi
    params_->minMarkerDistanceRate                  = 0.01; // dari 0.05 — marker tergabung saling berdekatan
    params_->maxErroneousBitsInBorderRate           = 0.40; // sedikit lebih toleran untuk shared border
    params_->errorCorrectionRate                    = 0.6;
    params_->detectInvertedMarker                   = true;

    // Subpixel corner refinement — penting untuk akurasi pose pada marker berdekatan
    params_->cornerRefinementMethod                 = cv::aruco::CORNER_REFINE_SUBPIX;
    params_->cornerRefinementWinSize                = 5;
    params_->cornerRefinementMaxIterations          = 50;
    params_->cornerRefinementMinAccuracy            = 0.01;
}

} // namespace fiducial_detector
