//
// Created by mars-lab on 19-4-19.
//

#include "plane.h"

Plane::Plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        Eigen::Vector4d plane,
        size_t id):cloud_(cloud),
                    plane_(plane),
                    id_(id) {

}

Transform Plane::calc_viewpoint_tf() {
    // the result viewpoint (the transfrom between new frame and original one)
    // Direction: transfrom points from new frame => original frame
    Transform result;

    // fixed parameter
    // distance to the plane
    double d = 1.2;

    // position = centroid - d * noraml
    pcl::PointXYZ centroid = calc_centroid(cloud_);
    result.T(0) = centroid.x - d * plane_(0);
    result.T(1) = centroid.y - d * plane_(1);
    result.T(2) = centroid.z - d * plane_(2);

    // oriention (same as plane)
//    Eigen::Vector3d origin_direction = Eigen::Vector3d::Zero();
//    origin_direction(0) = 0.0;
//    origin_direction(1) = 0.0;
//    origin_direction(2) = 1.0;
//    Eigen::Vector3d plane_direction = Eigen::Vector3d::Zero();
//    plane_direction(0) = plane_(0);
//    plane_direction(1) = plane_(1);
//    plane_direction(2) = plane_(2);
//    result.orientation = Eigen::Quaterniond::FromTwoVectors(origin_direction, plane_direction);

//    auto rpy = plane_to_rpy(plane_);
//    result.orientation = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
//                         * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
//                         * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
////    result.orientation = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())
////                         * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
////                         * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
////    result.orientation = Eigen::AngleAxisd(-1 * M_PI / 2, Eigen::Vector3d::UnitY()) * result.orientation;

//    double pitch = atan2(plane_[0], plane_[2]);
//    double roll = atan2(-1 * plane_[1], std::sqrt(std::pow(plane_[0], 2) + std::pow(plane_[2], 2)));

//    result.orientation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
//    result.orientation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX())
//                            * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
//    result.orientation = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
//    result.orientation = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    // new z-axis
    Eigen::Vector3d new_unitz = Eigen::Vector3d::Zero();
    new_unitz(0) = plane_(0);
    new_unitz(1) = plane_(1);
    new_unitz(2) = plane_(2);

    // calculate the new x-axis
    Eigen::Vector3d new_unitx = Eigen::Vector3d::Zero();
    new_unitx(0) = 1;
    new_unitx(2) = -1 * (plane_(0) * new_unitx(0)) / plane_(2);

    result.R = qFromTwoVectorPairs(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), new_unitz, new_unitx);

    // Y-axis should always facing down
    // use y to check if the camera has been rotated
    Eigen::Vector3d new_yaxis = applyTransform({Eigen::Vector3d::UnitY()}, result)[0] - result.T;
    while (new_yaxis(1) < 0){
        result.R = qFromTwoVectorPairs(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), new_unitz, -1 * new_unitx);
        new_yaxis = applyTransform({Eigen::Vector3d::UnitY()}, result)[0] - result.T;
    }
    LOG_INFO << "new y-axis: " << new_yaxis << std::endl;
    return result;
}


Viewpoint Plane::calc_plane_vis() {
    // the viewpoint (z-axis) direction
    // Note: the viewpoint can not be used as transform. A transform should transform (0,0,1) to the viewpoint
    Viewpoint result;

    pcl::PointXYZ centroid = calc_centroid(cloud_);
    result.position(0) = centroid.x;
    result.position(1) = centroid.y;
    result.position(2) = centroid.z;

    // oriention (same as plane)
    Eigen::Vector3d origin_direction = Eigen::Vector3d::Zero();
    origin_direction(0) = 1.0;
    origin_direction(1) = 0.0;
    origin_direction(2) = 0.0;
    Eigen::Vector3d plane_direction = Eigen::Vector3d::Zero();
    plane_direction(0) = plane_(0);
    plane_direction(1) = plane_(1);
    plane_direction(2) = plane_(2);
    result.orientation = Eigen::Quaterniond::FromTwoVectors(origin_direction, plane_direction);

    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Plane::get_convate_polygon(){
    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud (cloud_);
    chull.reconstruct (*cloud_hull);

    return cloud_hull;
}

