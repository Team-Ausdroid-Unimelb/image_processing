#include "HSVFilter.hpp"

HSVFilter::HSVFilter(const std::string& color) : color_(color) {
    auto thresholds = getThresholdsFromROSParam(color);
    lower_threshold_ = thresholds[0];
    upper_threshold_ = thresholds[1];
}

std::vector<cv::Scalar> HSVFilter::getThresholdsFromROSParam(const std::string& color) {
    std::map<std::string, std::pair<std::vector<int>, std::vector<int>>> default_thresholds = {
        {"R", {{0, 50, 50}, {10, 255, 255}}},
        {"G", {{35, 50, 50}, {85, 255, 255}}},
        {"B", {{100, 50, 50}, {140, 255, 255}}}
    };

    std::vector<int> lower, upper;
    if(!ros::param::get(color + "_lower_threshold", lower))
        lower = default_thresholds[color].first;

    if(!ros::param::get(color + "_upper_threshold", upper))
        upper = default_thresholds[color].second;

    return {cv::Scalar(lower[0], lower[1], lower[2]), cv::Scalar(upper[0], upper[1], upper[2])};
}

bool HSVFilter::apply(const cv::Mat& image, cv::Mat* mask) {
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
    
    cv::Mat local_mask;
    cv::inRange(hsv_image, lower_threshold_, upper_threshold_, local_mask);
    double avg_mask_val = cv::mean(local_mask)[0];

    if (mask) {  // If a mask pointer is provided, copy the local_mask to it.
        local_mask.copyTo(*mask);
    }
    
    bool detection = avg_mask_val > 15;
    if (detection) {
        ROS_INFO("Detection successful for color: %s", color_.c_str());
    }

    return detection;
}


