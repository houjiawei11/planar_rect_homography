//
// Created by ernest on 19-4-20.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_VIEWPOINT_HPP
#define PLANAR_RECT_HOMOGRAPHY_PKG_VIEWPOINT_HPP

#include <Eigen/Dense>

class Viewpoint {
public:
    Viewpoint();

    // Public Attributions
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_VIEWPOINT_HPP
