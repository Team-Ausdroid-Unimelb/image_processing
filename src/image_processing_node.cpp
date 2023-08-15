#include <opencv2/opencv.hpp>
#include <ros/ros.h>
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

    // Get the list of camera indices
    int single_camera_index;
    std::vector<int> camera_indices;

    // Try fetching the parameter as a single integer
    if(pnh.getParam("camera_indices", single_camera_index)) {
        ROS_INFO("Loaded single camera index: %d", single_camera_index);
        camera_indices.push_back(single_camera_index);
    }
    // If not found as single integer, try fetching as a list of integers
    else if(pnh.getParam("camera_indices", camera_indices)) {
        for (int i : camera_indices) {
            ROS_INFO("Loaded camera index: %d", i);
        }
    }
    // If neither worked, default to camera 0
    else {
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

    while(ros::ok()) {
        for(size_t i = 0; i < cams.size(); ++i) {
            // ROS_INFO("Cam: %d", camera_indices[i]);
            cv::Mat frame, resized_frame;
            if(cams[i].read(frame)) {
                // ROS_INFO("Reading from camera index: %d", camera_indices[i]);
                cv::resize(frame, resized_frame, cv::Size(640, 480));

                cv::Mat mask_r, mask_g, mask_b;
                bool detection_r = r_filter.apply(frame, verbose_filter ? &mask_r : nullptr);
                bool detection_g = g_filter.apply(frame, verbose_filter ? &mask_g : nullptr);
                bool detection_b = b_filter.apply(frame, verbose_filter ? &mask_b : nullptr);

                std_msgs::Bool msg_r, msg_g, msg_b;
                msg_r.data = detection_r;
                msg_g.data = detection_g;
                msg_b.data = detection_b;

                pub_robot_r.publish(msg_r);
                pub_robot_g.publish(msg_g);
                pub_robot_b.publish(msg_b);

                if(verbose_original) {
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " Original", resized_frame);
                }
                if(verbose_filter) {
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " R", mask_r);
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " G", mask_g);
                    cv::imshow("Camera " + std::to_string(camera_indices[i]) + " B", mask_b);
                }
            }
            else {
                ROS_WARN("Failed to read from camera index: %d", camera_indices[i]);
            }
        }
        if(cv::waitKey(1) == 'q') {
            break;
        }
        ros::spinOnce();
    }

    for (auto& cam : cams) {
        cam.release();
    }

    cv::destroyAllWindows();
    return 0;
}
