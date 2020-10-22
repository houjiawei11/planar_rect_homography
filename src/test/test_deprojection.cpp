//
// Created by mars-lab on 19-4-22.
//

#include <Eigen/Dense>

#include "pcl_helper_functions.h"


int main(int argc, char * argv[]){
    Eigen::Matrix3d camera = Eigen::Matrix3d::Identity();
    camera(0,0) = 125;
    camera(1,1) = 52;
    camera(0,2) = 300;
    camera(1,2) = 400;

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal(0) = 1;
    normal(1) = 1;
    normal(2) = 1;
    normal.normalize();

    double d = 1;

    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    point(0) = 1;
    point(1) = 1;
    point(2) = 1;

    Eigen::Vector3d result = deprojectionToPlane(point,normal, d, camera);

    std::cout << "result " << result << std::endl;

    // check if it is correct
//    Eigen::Vector3d point_2d_non_h = camera * X;
//    Eigen::Vector3d

    return 0;
}