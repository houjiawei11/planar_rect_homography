//
// Created by mars-lab on 19-4-19.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_CAMERA_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_CAMERA_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <Eigen/Dense>

class Camera {
public:

    void set_from_topic(const cv::Mat& K_in, const std::vector<double>& D_in,
                        const unsigned int h, const unsigned int w);

    unsigned int height() const;
    unsigned int width() const;
    cv::Mat K() const;

    /***
     * Get the intrinsic matrix (Eigen)
     * @return
     */
    Eigen::Matrix3d toEigenK();

private:
    cv::Mat K_;                  // intrinsic matrix <double(3,3)>
    std::vector<double> D_;      // distortion vector of float

    unsigned int img_height_;
    unsigned int img_width_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_CAMERA_H
