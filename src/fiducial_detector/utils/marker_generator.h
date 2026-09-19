#pragma once
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>
namespace fiducial_detector {

struct MarkerValidationResult {
  int         id{-1};
  bool        pass{false};
  int         hamming{std::numeric_limits<int>::max()};
  std::string reason;
};

class MarkerGenerator {
public:


  static int generateDictionarySet(
    const std::string& dict_name,
    const std::string& output_dir,
    int marker_size_px = 200,
    int border_bits = 1);



  static bool validateGeneratedDictionary(
    const std::string& dict_name,
    const std::string& marker_dir,
    std::vector<MarkerValidationResult>& results);


  static void printValidationReport(
    const std::string& dict_name,
    const std::vector<MarkerValidationResult>& results);
};

}
