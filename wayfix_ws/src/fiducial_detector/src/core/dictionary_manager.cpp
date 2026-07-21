#include "utils/dictionary_manager.h"
#include <stdexcept>
#include <cstdio>

namespace fiducial_detector {

DictionaryManager::DictionaryManager() {
    init7x7();
}

void DictionaryManager::init7x7() {
    dict_info_.name          = "DICT_7X7_50";
    dict_info_.dict_id       = cv::aruco::DICT_7X7_50;
    dict_info_.dict          = fiducial_opencv_compat::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    dict_info_.marker_bits   = 7;
    dict_info_.total_markers = 50;
    dict_info_.border_bits   = 1;
}

cv::Ptr<cv::aruco::Dictionary> DictionaryManager::getDictionaryByName(
    const std::string& name)
{
    if (name != "DICT_7X7_50")
        throw std::invalid_argument("Only DICT_7X7_50 is supported: " + name);
    return fiducial_opencv_compat::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
}

int DictionaryManager::getDictIdByName(const std::string& name) {
    if (name != "DICT_7X7_50")
        throw std::invalid_argument("Only DICT_7X7_50 is supported: " + name);
    return cv::aruco::DICT_7X7_50;
}

int DictionaryManager::getBorderBitsForDict(const std::string& /*name*/) {
    return 1;
}

bool DictionaryManager::setActive(const std::string& name) {
    return (name == "DICT_7X7_50");
}

std::string DictionaryManager::activeName() const {
    return "DICT_7X7_50";
}

cv::Ptr<cv::aruco::Dictionary> DictionaryManager::activeDict() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return dict_info_.dict;
}

const DictionaryInfo& DictionaryManager::info(const std::string& name) const {
    if (name != "DICT_7X7_50")
        throw std::invalid_argument("Only DICT_7X7_50 is supported");
    return dict_info_;
}

bool DictionaryManager::validateDictionary(const std::string& name) const {
    if (name != "DICT_7X7_50") return false;
    if (!dict_info_.dict) return false;
    if (dict_info_.dict->bytesList.empty()) return false;
    if (dict_info_.dict->bytesList.rows != dict_info_.total_markers) return false;
    return true;
}

void DictionaryManager::printStartupValidation(const std::string& name) const {
    const std::string sep(54, '=');
    const std::string dash(54, '-');
    std::printf("\n%s\n", sep.c_str());
    std::printf("  DICT_7X7_50 DICTIONARY SELF-TEST\n");
    std::printf("  Note         : not a camera detection\n");
    std::printf("%s\n", dash.c_str());
    if (name != "DICT_7X7_50") {
        std::printf("  Status : FAILED — unsupported dictionary: %s\n", name.c_str());
        std::printf("%s\n\n", sep.c_str());
        return;
    }
    std::printf("  Dictionary   : %s\n", dict_info_.name.c_str());
    std::printf("  MarkerBits   : %d\n", dict_info_.marker_bits);
    std::printf("  TotalMarkers : %d\n", dict_info_.total_markers);
    std::printf("  BorderBits   : %d\n", dict_info_.border_bits);
    if (!dict_info_.dict || dict_info_.dict->bytesList.empty()) {
        std::printf("  BytesList    : EMPTY — dictionary not loaded!\n");
        std::printf("  Status       : FAILED ✗\n");
    } else if (dict_info_.dict->bytesList.rows != dict_info_.total_markers) {
        std::printf("  BytesList    : MISMATCH (got %d, expected %d)\n",
            dict_info_.dict->bytesList.rows, dict_info_.total_markers);
        std::printf("  Status       : FAILED ✗\n");
    } else {
        std::printf("  BytesList    : OK (%d rows)\n", dict_info_.dict->bytesList.rows);
        std::printf("  Status       : VALID ✓\n");
    }
    std::printf("%s\n\n", sep.c_str());
}

} // namespace fiducial_detector
