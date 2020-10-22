/***
 * Two-stage for input:
 *  1. stage 1: waiting for camera_info: create camera
 *  2. stage 2: waiting for other input message (color image and cloud)
 *
 *
 *
 * Handle inbound ROS messages (via in-class subscibers)
 * Handle outbound ROS message (via ROSVisualizer)
 * Create instance of camera
 * Create instance of frames
 * Call frame.process (use std::thread to exec this later)
 *
 * Get camera intrinsic
 * Assemble input messages (cloud and color images)
 *
 * All input subscriber will be activated only after receiving one camera info msg
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_RECTIFIER_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_RECTIFIER_H

#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include "settings.h"
#include "frame.h"
#include "camera.h"
#include "ros_visualizer.h"
#include "type_conversation.h"
#include "file_io.h"
#include "ExternalDetector.h"

class PlanarRectifier {
public:
    PlanarRectifier(ros::NodeHandlePtr nh, std::shared_ptr<Settings> settings, std::shared_ptr<FileIO> io_handler);

private:
    // ================== Functions ============================
    /***
     * Callback for setting camera info
     * In the current implementation, this callback should be activate exacylt once
     * @param msg_ptr
     */
    void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg_ptr);

    /***
     * Callback for inputs
     * @param msg_ptr
     */
    void callbackColorImage(const sensor_msgs::ImageConstPtr &msg_ptr);
    void callbackCloud(const sensor_msgs::PointCloud2ConstPtr &msg_ptr);

    /***
     * The function should be invoked when the frame is ready to get processing
     * @param ts: timestamp coresponding to the frame
     */
    void onFrameReady(ros::Time ts);

    // ================= Attrributions ========================
    ros::NodeHandlePtr node_handler_;

    std::vector<ros::Time> frames_timestamps_;              //tracking all exsiting timestamp in frames_
    std::map<ros::Time, std::shared_ptr<Frame>> frames_;    // all existing frames

    std::vector<ros::Subscriber> input_subscibers_;     // all inputs
    ros::Subscriber camera_sub_;

    std::shared_ptr<Camera> camera_;                    // the color camera
    bool camera_ready_;

    std::shared_ptr<Settings> settings_;
    std::shared_ptr<ROSVisualizer> visualizer_;
    std::shared_ptr<FileIO> io_handler_;
    std::shared_ptr<ExternalDetector> external_detector_;   // the external detector wrapper

    size_t num_frames_;                 // total number of frames processed (including ready and non-ready frames)
    size_t num_ready_frames_;

    tf::Transform tf_cloud2color_;      // the transform from pointcloud to color
    Eigen::Matrix4f tf_cloud2_color_eigen_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_RECTIFIER_H
