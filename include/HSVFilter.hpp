#ifndef HSVFILTER_HPP
#define HSVFILTER_HPP

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

class HSVFilter {
public:
    explicit HSVFilter(const std::string& color);
    bool apply(const cv::Mat& image, cv::Mat* mask = nullptr);

private:
    cv::Scalar lower_threshold_;
    cv::Scalar upper_threshold_;
    std::string color_;
    bool gpu_available_;

    std::vector<cv::Scalar> getThresholdsFromROSParam(const std::string& color);
};

#endif
