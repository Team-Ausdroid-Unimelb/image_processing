#include "HSVFilter.hpp"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>

HSVFilter::HSVFilter(const std::string& color) : color_(color), gpu_available_(cv::cuda::getCudaEnabledDeviceCount() > 0) {
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

    if (gpu_available_) {
        try {
            cv::cuda::GpuMat image_gpu(image), hsv_image_gpu, local_mask_gpu;
            
            cv::cuda::cvtColor(image_gpu, hsv_image_gpu, cv::COLOR_BGR2HSV);
            cv::cuda::GpuMat lower_bound(hsv_image_gpu.size(), hsv_image_gpu.type(), lower_threshold_);
            cv::cuda::GpuMat upper_bound(hsv_image_gpu.size(), hsv_image_gpu.type(), upper_threshold_);

            cv::cuda::GpuMat lower_mask, upper_mask, combined_mask;
            cv::cuda::compare(hsv_image_gpu, lower_bound, lower_mask, cv::CMP_GE);
            cv::cuda::compare(hsv_image_gpu, upper_bound, upper_mask, cv::CMP_LE);

            cv::cuda::bitwise_and(lower_mask, upper_mask, combined_mask);
            
            local_mask_gpu.download(hsv_image);  // Copying back the processed data from GPU to CPU for further operations
            
        } catch (const cv::Exception& e) {
            gpu_available_ = false;  // If there's an error using the GPU, fallback to CPU
            ROS_INFO("Error accessing GPU: %s", e.what());
        }
    }

    if (!gpu_available_) {
        ROS_INFO("Processing on CPU");
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, lower_threshold_, upper_threshold_, hsv_image);
    }


    double avg_mask_val = cv::mean(hsv_image)[0];

    if (mask) {  // If a mask pointer is provided, copy the local_mask to it.
        hsv_image.copyTo(*mask);
    }
    
    bool detection = avg_mask_val > 15;
    // if (detection) {
    //     ROS_INFO("Detection successful for color: %s", color_.c_str());
    // }

    return detection;
}


