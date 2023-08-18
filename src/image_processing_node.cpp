#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <vector>
#include "HSVFilter.hpp"
#include <thread>
#include <mutex>

class ImageProcessor {
private:
    std::vector<std::thread> camera_threads;
    std::mutex detection_mutex;

    std::vector<cv::VideoCapture> cams;
    std::vector<int> camera_indices;
    std_msgs::Bool msg_r, msg_g, msg_b;
    ros::NodeHandle nh, pnh;
    ros::Publisher pub_robot_r, pub_robot_g, pub_robot_b;

    HSVFilter r_filter, g_filter, b_filter;

    bool verbose_filter, verbose_original;
    bool detected_r, detected_g, detected_b;
    bool last_msg_r, last_msg_g, last_msg_b;
    int frame_count = 0;
    double elapsed_seconds = 0.0;

public:
    ImageProcessor() 
        : pnh("~"), 
          r_filter("R"), 
          g_filter("G"), 
          b_filter("B"),
          detected_r(false), detected_g(false), detected_b(false)
    {
        pnh.param("verbose_filter", verbose_filter, false);
        pnh.param("verbose_original", verbose_original, false);
        ROS_INFO_STREAM("Filter Verbose mode: " << verbose_filter);
        ROS_INFO_STREAM("Original frame Verbose mode: " << verbose_original);

        if(pnh.getParam("/image_processing/camera_indices", camera_indices)) {
            for (int i : camera_indices) {
                ROS_INFO("Loaded camera index: %d", i);
            }
        }else {
            camera_indices.push_back(0);  
            ROS_WARN("Using default camera index: 0");
        }

        pub_robot_r = nh.advertise<std_msgs::Bool>("detection_robot_r", 10);
        pub_robot_g = nh.advertise<std_msgs::Bool>("detection_robot_g", 10);
        pub_robot_b = nh.advertise<std_msgs::Bool>("detection_robot_b", 10);

        for(int index : camera_indices) {
            cv::VideoCapture cam(index);
            if (!cam.isOpened()) {
                ROS_ERROR("Failed to open camera %d", index);
                continue;
            } else {
                ROS_INFO("Successfully opened camera %d", index);
            }
            cams.push_back(std::move(cam));
        }
    }

    ~ImageProcessor() {
        for(auto& cam : cams) {
            cam.release();
        }
        cv::destroyAllWindows();
    }

    void displayFPS() {
        double fps = (frame_count * cams.size()) / elapsed_seconds;
        ROS_INFO("Average FPS: %f", fps);

        frame_count = 0;
        elapsed_seconds = 0.0;
    }

    void processCamera(size_t i) {
        cv::Mat frame;
        if(cams[i].read(frame)) {
            if(verbose_original) {
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(640, 480));
                cv::imshow("Camera " + std::to_string(camera_indices[i]) + " Original", resized_frame);
            }

            cv::Mat mask_r, mask_g, mask_b;

            bool local_detected_r = r_filter.apply(frame, verbose_filter ? &mask_r : nullptr);
            bool local_detected_g = g_filter.apply(frame, verbose_filter ? &mask_g : nullptr);
            bool local_detected_b = b_filter.apply(frame, verbose_filter ? &mask_b : nullptr);

            if(verbose_filter) {
                cv::imshow("Camera " + std::to_string(camera_indices[i]) + " R", mask_r);
                cv::imshow("Camera " + std::to_string(camera_indices[i]) + " G", mask_g);
                cv::imshow("Camera " + std::to_string(camera_indices[i]) + " B", mask_b);
            }

            std::lock_guard<std::mutex> lock(detection_mutex);
            detected_r = detected_r || local_detected_r;
            detected_g = detected_g || local_detected_g;
            detected_b = detected_b || local_detected_b;
        }
        else {
            ROS_WARN_THROTTLE(5.0, "Failed to read from camera index: %d", camera_indices[i]);
        }
    }

    void run() {
        while(ros::ok()) {
            auto start = std::chrono::high_resolution_clock::now();

            detected_r = false;
            detected_g = false;
            detected_b = false;

            for(size_t i = 0; i < cams.size(); ++i) {
                camera_threads.push_back(std::thread(&ImageProcessor::processCamera, this, i));
            }

            for(auto& thread : camera_threads) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
            camera_threads.clear();

            if (detected_r != last_msg_r) {
                last_msg_r = detected_r;
                msg_r.data = detected_r;
                pub_robot_r.publish(msg_r);
            }

            if (detected_g != last_msg_g) {
                last_msg_g = detected_g;
                msg_g.data = detected_g;
                pub_robot_g.publish(msg_g);
            }

            if (detected_b != last_msg_b) {
                last_msg_b = detected_b;
                msg_b.data = detected_b;
                pub_robot_b.publish(msg_b);
            }

            if(cv::waitKey(1) == 'q') {
                break;
            }
            ros::spinOnce();

            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
            elapsed_seconds += elapsed;
            frame_count++;

            if (elapsed_seconds >= 1.0) {
                displayFPS();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ImageProcessor processor;
    processor.run();
    return 0;
}