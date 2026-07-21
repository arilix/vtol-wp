#pragma once
#include <opencv2/aruco.hpp>
#include <mutex>
#include <string>

namespace fiducial_detector {

struct DictionaryInfo {
    std::string                    name;
    int                            dict_id;
    cv::Ptr<cv::aruco::Dictionary> dict;
    int                            marker_bits;
    int                            total_markers;
    int                            border_bits;
};

class DictionaryManager {
public:
    DictionaryManager();

    static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string& name);
    static int getDictIdByName(const std::string& name);
    static int getBorderBitsForDict(const std::string& name);

    bool        setActive(const std::string& name);
    std::string activeName() const;
    cv::Ptr<cv::aruco::Dictionary> activeDict() const;

    bool validateDictionary(const std::string& name) const;
    void printStartupValidation(const std::string& name) const;

    const DictionaryInfo& info(const std::string& name) const;

private:
    void init7x7();
    DictionaryInfo       dict_info_;
    mutable std::mutex   mutex_;
};

} // namespace fiducial_detector
