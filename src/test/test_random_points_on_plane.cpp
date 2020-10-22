//
// Created by mars-lab on 19-4-22.
//
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "pcl_helper_functions.h"


bool test_1(){
    std::cout << "test_1 start" << std::endl;
    Eigen::Vector3d plane_normal = Eigen::Vector3d(1.0, 2.0, 3.0);
    plane_normal.normalize();
    double d = 2;

    // construct the plane
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    plane(0) = plane_normal(0);
    plane(1) = plane_normal(1);
    plane(2) = plane_normal(2);
    plane(3) = d;

    std::vector<Eigen::Vector3d> result = fourRectPointsFromPlane(plane);
    for (int i = 0; i < 4; i++){
        Eigen::Vector3d& v = result[i];
        std::cout << "expected 0, actual: " << (v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) << std::endl;
        assert((v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) - 0 < 1e-6);
    }

    std::cout << "test_1 done" << std::endl;

    return true;
}

bool test_2(){
    std::cout << "test_2 start" << std::endl;
    Eigen::Vector3d plane_normal = Eigen::Vector3d(1.0, 2.0, 0.0);
    plane_normal.normalize();
    double d = 2;

    // construct the plane
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    plane(0) = plane_normal(0);
    plane(1) = plane_normal(1);
    plane(2) = plane_normal(2);
    plane(3) = d;

    std::vector<Eigen::Vector3d> result = fourRectPointsFromPlane(plane);
    for (int i = 0; i < 4; i++){
        Eigen::Vector3d& v = result[i];
        std::cout << "expected 0, actual: " << (v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) << std::endl;
        assert((v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) - 0 < 1e-6);
    }

    std::cout << "test_2 done" << std::endl;

    return true;
}

bool test_3(){
    std::cout << "test_3 start" << std::endl;

    Eigen::Vector3d plane_normal = Eigen::Vector3d(1.3, 0.0, 0.0);
    plane_normal.normalize();
    double d = 2;

    // construct the plane
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    plane(0) = plane_normal(0);
    plane(1) = plane_normal(1);
    plane(2) = plane_normal(2);
    plane(3) = d;

    std::vector<Eigen::Vector3d> result = fourRectPointsFromPlane(plane);
    for (int i = 0; i < 4; i++){
        Eigen::Vector3d& v = result[i];
        std::cout << "expected 0, actual: " << (v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) << std::endl;
        assert((v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) - 0 < 1e-6);
    }

    std::cout << "test_3 done" << std::endl;

    return true;
}

bool test_4(){
    std::cout << "test_4 start" << std::endl;

    Eigen::Vector3d plane_normal = Eigen::Vector3d(0.0, 2.3, 0.0);
    plane_normal.normalize();
    double d = 2;

    // construct the plane
    Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    plane(0) = plane_normal(0);
    plane(1) = plane_normal(1);
    plane(2) = plane_normal(2);
    plane(3) = d;

    std::vector<Eigen::Vector3d> result = fourRectPointsFromPlane(plane);
    for (int i = 0; i < 4; i++){
        Eigen::Vector3d& v = result[i];
        std::cout << "expected 0, actual: " << (v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) << std::endl;
        assert((v(0) * plane(0) + v(1) * plane(1) + v(2) * plane(2) + plane(3)) - 0 < 1e-6);
    }

    std::cout << "test_4 done" << std::endl;

    return true;
}

int main(int argc, char * argv[]){
    test_1();
    test_2();
    test_3();
    test_4();

    return 0;
}