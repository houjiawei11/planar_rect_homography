//
// Created by mars-lab on 19-4-19.
//

#include "camera.h"

void Camera::set_from_topic(const cv::Mat& K_in, const std::vector<double>& D_in,
                            const unsigned int h, const unsigned int w) {
    // K
    K_ = K_in;

    // Note: for Kinect2, it is all zeros (5 zeros)
    // D
    D_ = D_in;

    img_height_ = h;
    img_width_ = w;
}


unsigned int Camera::height() const {
    return img_height_;
}

unsigned int Camera::width() const {
    return width();
}

cv::Mat Camera::K() const {
    return K_;
}

Eigen::Matrix3d Camera::toEigenK() {
    Eigen::Matrix3d result = Eigen::Matrix3d::Zero();

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            result(i,j) = K_.at<double>(i,j);
        }
    }

    return result;
}
