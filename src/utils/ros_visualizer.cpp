//
// Created by mars-lab on 19-4-19.
//

#include "ros_visualizer.h"

ROSVisualizer::ROSVisualizer(ros::NodeHandlePtr node_handler):publishers_(),
                                node_handler_(node_handler), tf_boardcaster_() {
    // publisher
    publishers_["input/color/image"] = node_handler_->advertise<sensor_msgs::Image>("input/color/image", 1000);
    publishers_["input/cloud"] = node_handler_->advertise<sensor_msgs::PointCloud2>("input/cloud", 1000);

    publishers_["segamented/cloud"] = node_handler_->advertise<sensor_msgs::PointCloud2>("segamented/cloud", 1000);
    publishers_["plane_polygon"] = node_handler_->advertise<sensor_msgs::PointCloud2>("plane_polygon", 1000);
    publishers_["im_vis"] = node_handler_->advertise<sensor_msgs::Image>("im_vis", 1000);
    publishers_["plane_centroid"] = node_handler_->advertise<sensor_msgs::PointCloud2>("plane_centroid", 1000);
    publishers_["planes"] = node_handler_->advertise<geometry_msgs::PoseArray>("planes", 1000);
    publishers_["viewpoin_tfs"] = node_handler_->advertise<geometry_msgs::PoseArray>("viewpoin_tfs", 1000);

    publishers_["rectified_img"] = node_handler_->advertise<sensor_msgs::Image>("rectified_img", 1000);

    publishers_["status_flag"] = node_handler->advertise<std_msgs::String>("status_flag", 1000, true);
}

void ROSVisualizer::publishPointcloud(std::string topic_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                      const std::string& frame_id) {
    // validate topic name
    if (publishers_.count(topic_name) == 0){
        LOG_ERROR << "unrecognized topic name " << topic_name << std::endl;
        throw std::runtime_error("unrecognized topic name");
    }

    // convert to ROS message and get it published
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    msg_out.header.frame_id = frame_id;
    msg_out.header.stamp = ros::Time::now();
    publishers_[topic_name].publish(msg_out);
}

void ROSVisualizer::publishPointcloud(std::string topic_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      const std::string& frame_id) {
    // validate topic name
    if (publishers_.count(topic_name) == 0){
        LOG_ERROR << "unrecognized topic name " << topic_name << std::endl;
        throw std::runtime_error("unrecognized topic name");
    }

    // convert to ROS message and get it published
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    msg_out.header.frame_id = frame_id;
    msg_out.header.stamp = ros::Time::now();
    publishers_[topic_name].publish(msg_out);
}

void ROSVisualizer::publishImage(std::string topic_name, cv::Mat img, const std::string& frame_id) {
    // validate topic name
    if (publishers_.count(topic_name) == 0){
        LOG_ERROR << "unrecognized topic name " << topic_name << std::endl;
        throw std::runtime_error("unrecognized topic name");
    }

    // convert the image to ROS message
    sensor_msgs::Image msg_out;
    cv_bridge::CvImage cv_ptr;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    cv_ptr = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    cv_ptr.toImageMsg(msg_out);
    publishers_[topic_name].publish(msg_out);
}

void ROSVisualizer::publishViewpoint(std::string topic_name, std::vector<Viewpoint> vp,
                                        const std::string& frame_id) {
    // validate topic name
    if (publishers_.count(topic_name) == 0){
        LOG_ERROR << "unrecognized topic name " << topic_name << std::endl;
        throw std::runtime_error("unrecognized topic name");
    }

    geometry_msgs::PoseArray msg_out;
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = frame_id;

    for (auto v : vp){  // for each viewpoint
        geometry_msgs::Pose pose;

        pose.position.x = v.position(0);
        pose.position.y = v.position(1);
        pose.position.z = v.position(2);

        geometry_msgs::Quaternion q;
        tf::quaternionEigenToMsg(v.orientation, q);
        pose.orientation = q;

        msg_out.poses.push_back(pose);
    }

    publishers_[topic_name].publish(msg_out);
}

void ROSVisualizer::publishTransform(std::string topic_name, std::vector<Transform> tfs,
                                     const std::string& frame_id) {
    // validate topic name
    // deprecated
//    if (publishers_.count(topic_name) == 0){
//        LOG_ERROR << "unrecognized topic name " << topic_name << std::endl;
//        throw std::runtime_error("unrecognized topic name");
//    }
//
//    geometry_msgs::PoseArray msg_out;
//    msg_out.header.stamp = ros::Time::now();
//    msg_out.header.frame_id = frame_id;
//
//    for (auto v : tfs){  // for each transform
//        // x-axis
//        geometry_msgs::Pose pose;
//
//        pose.position.x = v.T(0);
//        pose.position.y = v.T(1);
//        pose.position.z = v.T(2);
//
//        geometry_msgs::Quaternion q;
//        tf::quaternionEigenToMsg(v.R, q);
//        pose.orientation = q;
//
//        msg_out.poses.push_back(pose);
//    }
//    publishers_[topic_name].publish(msg_out);

    for (int i = 0; i < tfs.size(); i++){
        Transform v = tfs[i];
//            Transform v_raw = tfs[i];         // to test inverse function
//            Transform v = v_raw.inverse();
        // publish the transform via tf boardcaster
        tf::Transform msg_out_tf;
        msg_out_tf.setIdentity();
        msg_out_tf.setOrigin(tf::Vector3(v.T(0), v.T(1), v.T(2)));

        tf::Quaternion q_tf;
        tf::quaternionEigenToTF(v.R, q_tf);
        msg_out_tf.setRotation(q_tf);
//            msg_out_tf.inverse();

        tf_boardcaster_.sendTransform(
                tf::StampedTransform(msg_out_tf,
                                     ros::Time::now(),
                                     frame_id,
                                     "test_frame_" + std::to_string(i)));
//            tf_boardcaster_.sendTransform(
//                    tf::StampedTransform(msg_out_tf,
//                                         ros::Time::now(),
//                                         "test_frame_" + std::to_string(i),
//                                         frame_id));
    }
}

void ROSVisualizer::publishStatusFinishedOnce(){
    std_msgs::String msg;
    msg.data = "Image Processed";
    publishers_["status_flag"].publish(msg);
}
