//
// Created by mars-lab on 19-4-19.
//

#include "type_conversation.h"

namespace planar_rectifier {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromROSMsg(const sensor_msgs::PointCloud2ConstPtr &msg_ptr){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        size_t num_points = msg_ptr->width * msg_ptr->height;

        for (size_t i = 0; i < num_points; i++){
            result_cloud->push_back(pcl::PointXYZRGB());
        }

        size_t i = 0;
        for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg_ptr, "x"); it != it.end(); ++it) {
            result_cloud->points[i].x = it[0];
            result_cloud->points[i].y = it[1];
            result_cloud->points[i].z = it[2];
            i ++;
        }

        i = 0;
        for (sensor_msgs::PointCloud2ConstIterator<uint8_t> it(*msg_ptr, "rgb"); it != it.end(); ++it) {
            result_cloud->points[i].b = it[0];
            result_cloud->points[i].g = it[1];
            result_cloud->points[i].r = it[2];
            i ++;
        }

//        pcl::io::savePCDFileASCII("/home/mars-lab/results/o.xyz", *result_cloud);

        return result_cloud;
    }

}