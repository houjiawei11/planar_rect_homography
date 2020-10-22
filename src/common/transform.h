//
// Created by mars-lab on 19-4-21.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_TRANSFORM_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_TRANSFORM_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class Transform {
public:
    Transform();

    Eigen::Vector3d T;
    Eigen::Quaterniond R;

    // calclate the inverse transform of the current transform
    Transform inverse() const;

    /***
     * Return translation in opencv format
     * @return
     */
    cv::Mat T_cv2() const;
    cv::Mat R_cv2() const;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_TRANSFORM_H
