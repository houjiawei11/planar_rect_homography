//
// Created by mars-lab on 19-4-19.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_TYPE_CONVERSATION_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_TYPE_CONVERSATION_H

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/io/pcd_io.h>

namespace planar_rectifier {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromROSMsg(const sensor_msgs::PointCloud2ConstPtr &msg_ptr);

}
#endif //PLANAR_RECT_HOMOGRAPHY_PKG_TYPE_CONVERSATION_H
