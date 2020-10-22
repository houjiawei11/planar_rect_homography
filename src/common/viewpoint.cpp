//
// Created by ernest on 19-4-20.
//

#include "viewpoint.h"
Viewpoint::Viewpoint() {
    position = Eigen::Vector3d::Zero();
    orientation = Eigen::Quaterniond::Identity();
}