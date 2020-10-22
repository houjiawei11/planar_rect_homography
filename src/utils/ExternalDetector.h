/***
 * A warpper class for external detection
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_EXTERNALDETECTOR_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_EXTERNALDETECTOR_H

#include <mutex>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include "Hazmat.h"
#include "logging_util.h"


class ExternalDetector {
public:
    ExternalDetector(ros::NodeHandlePtr nh);

    // perform hazmat detection on one image
    // return vector of hazmat signs
    std::vector<Hazmat>  detect_hazmat_raw(const cv::Mat img);

    // generate visualization image
    static cv::Mat gen_vis_im(const cv::Mat& img, std::vector<Hazmat> hazmats);
private:
    // helper function for external detcetion

    // set up subscriber for receiving detection result
    void receiveDetectionResultCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_ptr);

    // publish one image to the input topic for detector
    void publishImage(const cv::Mat img);

    // ==================== Visualization related =========================
    static Eigen::Vector3i get_color(int sub_id);


    // make sure only one detection is happening
    // concurrent detection is not supported for now
    std::mutex mutex_;          // internal usgae only

    ros::NodeHandlePtr node_handler_;
    ros::Publisher img_pub_;            // to publish image for detector (detector input image)
    ros::Subscriber img_sub_;

    ros::CallbackQueue detection_result_queue_;

    // the buffer for saving the detection result
    std::mutex mutex_result_buffer_;
    bool result_ready_;
    std::vector<Hazmat> result_buffer_;

//    std::thread thread_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_EXTERNALDETECTOR_H
