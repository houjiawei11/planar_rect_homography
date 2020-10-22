/***
 * Handle all out-bound messages (mainly for visualization purpose)
 *
 * Each node should has exactly one ROSVisualizer (std::shared_ptr)
 */


#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_ROSVISUALIZER_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_ROSVISUALIZER_H

#include <string>
#include <map>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include "viewpoint.h"
#include "transform.h"
#include "logging_util.h"

class ROSVisualizer {
public:
    /***
     * Create publishers for visualization upon creation
     */
    ROSVisualizer(ros::NodeHandlePtr node_handler);

    /***
     * Publish one pointcloud message (sensor_msgs/PointCloud2), given the frame id and cloud
     * Convert pcl pointcloud to ros sensor_msgs
     * @param topic_name : the given topic name
     * @param cloud : pcl cloud
     * @param frame_id : the frame id
     */
    void publishPointcloud(std::string topic_name,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            const std::string& frame_id);
    void publishPointcloud(std::string topic_name,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            const std::string& frame_id);

    /***
     * Publish one image message (sensor_msgs/Image), given the frame id and cloud
     * convert the image (Opencv Mat) to ros message
     * @param topic_name: the topic name
     * @param img: the opencv Mat Image
     */
    void publishImage(std::string topic_name, cv::Mat img, const std::string& frame_id);

    /***
     * Publish viewpoints
     * @param topic_name
     * @param vp
     * @param frame_id
     */
    void publishViewpoint(std::string topic_name, std::vector<Viewpoint> vp, const std::string& frame_id);
    void publishTransform(std::string topic_name, std::vector<Transform> tfs, const std::string& frame_id);

    void publishStatusFinishedOnce();
private:
    // store all publisher for visualization
    std::map<std::string, ros::Publisher> publishers_;
//    std::map<std::string, int> seq_numbers_;

    ros::NodeHandlePtr node_handler_;
    tf::TransformBroadcaster tf_boardcaster_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_ROSVISUALIZER_H
