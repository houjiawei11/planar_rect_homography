//
// Created by mars-lab on 19-5-9.
//

#include "ExternalDetector.h"

ExternalDetector::ExternalDetector(ros::NodeHandlePtr nh): mutex_(), node_handler_(new ros::NodeHandle()), detection_result_queue_(),
        result_buffer_(),mutex_result_buffer_(), result_ready_(false){
    img_pub_ = node_handler_->advertise<sensor_msgs::Image>("/detector/input", 1000);

//    ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
//            "/objects", // topic name
//            10, // queue length
//            &ExternalDetector::receiveDetectionResultCallback, // callback
//            this, // tracked object, we don't need one thus NULL
//            &detection_result_queue_ // pointer to callback queue object
//    );

//    img_sub_ = node_handler_->subscribe(ops);
    node_handler_->setCallbackQueue(&detection_result_queue_);
    img_sub_ = node_handler_->subscribe("/objects", 1000, &ExternalDetector::receiveDetectionResultCallback, this);

//    ros::AsyncSpinner result_spinner(1, &detection_result_queue_);
//    result_spinner.start();
}


std::vector<Hazmat> ExternalDetector::detect_hazmat_raw(const cv::Mat img) {
//    LOG_INFO << "detection request received" << std::endl;

    mutex_.lock();
//    LOG_INFO << "lock obtained" << std::endl;

    publishImage(img);

    std::vector<Hazmat> results;

    // wait for the detection result
    while (true){
        if (result_ready_){
//            LOG_INFO << "query result available!" << std::endl;
            mutex_result_buffer_.lock();

            results = result_buffer_;
            result_buffer_.clear();
            result_ready_ = false;

            mutex_result_buffer_.unlock();
            break;
        }
        else{
//            LOG_INFO << "Start of spin" << std::endl;
//            std::cout << "query result is empty!" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            detection_result_queue_.callOne();
//            LOG_INFO << "End of spin" << std::endl;
        }
    }

    mutex_.unlock();

//    LOG_INFO << "detection request completed" << std::endl;

    return results;
}

cv::Mat ExternalDetector::gen_vis_im(const cv::Mat& img, std::vector<Hazmat> hazmats) {
    cv::Mat im_vis = img.clone();
    for (const Hazmat& s : hazmats){
        Eigen::Vector3i color_eigen = ExternalDetector::get_color(s.id_);
        cv::Scalar line_color = cv::Scalar(color_eigen(0), color_eigen(1), color_eigen(2));
        cv::polylines(im_vis, s.corners_, true, line_color, 2);
    }

    return im_vis;
}

Eigen::Vector3i ExternalDetector::get_color(int sub_id) {
    int num = sub_id % 4;

    if (num == 0)                       return Eigen::Vector3i(255, 0, 0);
    else if (num == 1)                  return Eigen::Vector3i(0, 255, 0);
    else if (num == 2)                  return Eigen::Vector3i(0, 0, 255);
    else if (num == 3)                  return Eigen::Vector3i(0, 0, 0);
}

void ExternalDetector::publishImage(const cv::Mat img) {
    // convert the image to ROS message
    sensor_msgs::Image msg_out;
    cv_bridge::CvImage cv_ptr;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    cv_ptr = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    cv_ptr.toImageMsg(msg_out);
    img_pub_.publish(msg_out);
}

void ExternalDetector::receiveDetectionResultCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_ptr) {
    // Parsing the detection result
//    LOG_INFO << "detection result received" << std::endl;

    // check the buffer should be empty
    if (!result_buffer_.empty()){
        LOG_ERROR << "detection buffer is not empty!" << std::endl;
    }

    // Data Format: ID + Width + Height + Homography Matrix
    int n_detection = (int)msg_ptr->data.size() / 11;

    // extarct all ids and the homography matrixs
    std::vector<int> ids;
//    std::vector<cv::Matx33f> tfs;

    // make sure result buffer operations are atomic
    mutex_result_buffer_.lock();
    for (int i = 0; i < n_detection; i++){
        int id = msg_ptr->data[i * 12];
        ids.push_back(id);
        cv::Matx33f tf_tmp;
        for (int iter_i = 0; iter_i < 3; iter_i++) {
            for (int iter_j = 0; iter_j < 3; iter_j++) {
                tf_tmp(iter_j, iter_i) = msg_ptr->data[i * 12 + 3 * iter_i + iter_j + 3];
            }
        }

        // apply the transform to corners
        float obj_width = msg_ptr->data[i * 12 + 1];
        float obj_height = msg_ptr->data[i * 12 + 2];
        std::vector<cv::Point3d> points_base;
        points_base.push_back(cv::Point3d(0, 0, 1));
        points_base.push_back(cv::Point3d(0, obj_height, 1));
        points_base.push_back(cv::Point3d(obj_width, obj_height, 1));
        points_base.push_back(cv::Point3d(obj_width, 0, 1));

        std::vector<cv::Point3d> points_final;
        points_final.push_back(cv::Point3d(0, 0, 1));
        points_final.push_back(cv::Point3d(0, 0, 1));
        points_final.push_back(cv::Point3d(0, 0, 1));
        points_final.push_back(cv::Point3d(0, 0, 1));

        cv::transform(points_base, points_final, tf_tmp);

        // normalize the points
        // points_2d_final contains 4 cv::Point, representing 4 corners points of the target hazmat sign
        std::vector<cv::Point> points_2d_final;

        for (int point_i = 0; point_i < points_final.size(); point_i ++){
            points_2d_final.push_back(
                    cv::Point2f(
                            points_final[point_i].x / points_final[point_i].z,
                            points_final[point_i].y / points_final[point_i].z)
            );
        }

        // push back to buffer
        result_buffer_.push_back(Hazmat(points_2d_final, id));
    }
    result_ready_ = true;
    mutex_result_buffer_.unlock();
}