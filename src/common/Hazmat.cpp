//
// Created by mars-lab on 19-5-9.
//

#include "Hazmat.h"

Hazmat::Hazmat(const std::vector<cv::Point> corners, int id) :corners_(corners), id_(id) {}

void Hazmat::warpback(const cv::Mat& H) {
    std::vector<cv::Point> results;
    for (int i = 0; i< corners_.size(); i++) {
        results.push_back(
                cv::Point2f(0.0, 0.0)
        );
    }

    cv::perspectiveTransform(corners_, results, H);

    corners_ = results;
}