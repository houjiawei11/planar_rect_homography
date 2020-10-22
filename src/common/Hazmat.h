/***
 * One single hazmat object
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_HAZMAT_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_HAZMAT_H

#include <opencv2/core/core.hpp>

class Hazmat {
public:
    Hazmat(const std::vector<cv::Point> corners, int id);

    std::vector<cv::Point> corners_;
    int id_;

    /***
     * Warp back to original image
     */
    void warpback(const cv::Mat& H);
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_HAZMAT_H
