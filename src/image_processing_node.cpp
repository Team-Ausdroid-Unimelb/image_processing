#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <vector>
#include "HSVFilter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");  // Private NodeHandle for parameters

    bool verbose_filter, verbose_original;
    pnh.param("verbose_filter", verbose_filter, false);  // Defaults to false if not set
    pnh.param("verbose_original", verbose_original, false);    // Defaults to false if not set
    ROS_INFO_STREAM("Filter Verbose mode: " << verbose_filter);
    ROS_INFO_STREAM("Original frame Verbose mode: " << verbose_original);

    HSVFilter r_filter("R"), g_filter("G"), b_filter("B");

    std::vector<int> camera_indices;

    if(pnh.getParam("/image_processing/camera_indices", camera_indices)) {
        for (int i : camera_indices) {
            ROS_INFO("Loaded camera index: %d", i);
        }
    }else {
        camera_indices.push_back(0);  
        ROS_WARN("Using default camera index: 0");
    }

    ros::Publisher pub_robot_r = nh.advertise<std_msgs::Bool>("detection_robot_r", 10);
    ros::Publisher pub_robot_g = nh.advertise<std_msgs::Bool>("detection_robot_g", 10);
    ros::Publisher pub_robot_b = nh.advertise<std_msgs::Bool>("detection_robot_b", 10);

    ROS_INFO_STREAM("Camera indices from param: " << camera_indices.size());
    std::vector<cv::VideoCapture> cams;
    for(int index : camera_indices) {
        cv::VideoCapture cam;
        if (!cam.open(index)) {
            ROS_ERROR("Failed to open camera %d", index);
            continue;
        }else {
            ROS_INFO("Successfully opened camera %d", index);
        }

        /*For debug purpose, if the camera doesn't work. Comment to reduce window numbers.*/
        // cv::Mat test_frame;
        // if (cam.read(test_frame)) {
        //     cv::imshow("Test frame for cam " + std::to_string(index), test_frame);
        //     cv::waitKey(1); // Allow time for the frame to be shown.
        //     ROS_INFO("Showing test frame for camera %d", index);
        // } else {
        //     ROS_ERROR("Test read failed for camera %d", index);
        // }

        cams.push_back(cam);
    }

    std_msgs::Bool msg_r, msg_g, msg_b;

    bool last_msg_r = false;
    bool last_msg_g = false;
    bool last_msg_b = false;

    int frame_count = 0;
    double elapsed_seconds = 0.0;

    
    while(ros::ok()) {

        auto start = std::chrono::high_resolution_clock::now();

        bool detected_r = false;
        bool detected_g = false;
        bool detected_b = false;

        for(size_t i = 0; i < cams.size(); ++i) {
            // ROS_INFO("Cam: %d", camera_indices[i]);
            cv::Mat frame;
            if(cams[i].read(frame)) {
                if(verbose_original) {
                    cv::Mat resized_frame;
                    cv::resize(frame, resized_frame, cv::Size(640, 480));
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " Original", resized_frame);
                }

                cv::Mat mask_r, mask_g, mask_b;
                detected_r = detected_r || r_filter.apply(frame, verbose_filter ? &mask_r : nullptr);
                detected_g = detected_g || g_filter.apply(frame, verbose_filter ? &mask_g : nullptr);
                detected_b = detected_b || b_filter.apply(frame, verbose_filter ? &mask_b : nullptr);
                
                if(verbose_filter) {
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " R", mask_r);
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " G", mask_g);
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " B", mask_b);
                }
            }
            else {
                ROS_WARN_THROTTLE(5.0, "Failed to read from camera index: %d", camera_indices[i]);
            }
        }

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
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;  // Convert to seconds
        elapsed_seconds += elapsed;
        frame_count++;

        if (elapsed_seconds >= 1.0) {  // Display FPS every second
            double fps = (frame_count * cams.size()) / elapsed_seconds;
            ROS_INFO("Average FPS: %f", fps);

            // Reset the counters
            frame_count = 0;
            elapsed_seconds = 0.0;
        }

    }

    for (auto& cam : cams) {
        cam.release();
    }

    cv::destroyAllWindows();
    return 0;
}
