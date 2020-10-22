//
// Created by mars-lab on 19-4-23.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_FILE_IO_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_FILE_IO_H

#include <cstdlib>
#include <chrono>
#include <map>
#include <fstream>
#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "logging_util.h"
#include "pcl_helper_functions.h"

class FileIO {
public:
    /***
     * (Constructor) initialize a the file structure
     */
    explicit FileIO(const std::string& result_dir);

    /***
     * Save the input image to disk
     * @param img 3-channel RGB image (OpenCV)
     * @param id the id number of the frame
     */
    void save_input_img(const cv::Mat& img, int frame_id);
    void save_detection_result_raw(const cv::Mat& img, int frame_id);

    // save individual detection result (after rectification)
    void save_detection_result_rect_single(const cv::Mat& img, int frame_id, int sub_img_id);
    void save_detection_result_rect_multi(const cv::Mat& img, int frame_id, int sub_img_id);
    /***
     * Save the rectified image to disk
     * @param img
     * @param frame_id : the id of the frame
     * @param sub_img_id : the sub image id (depend on each frame)
     */
    void save_rectified_img(const cv::Mat& img, const cv::Mat H, int frame_id, int sub_img_id);

    static int dirExists(const char *path);
    static int dirExists(const std::string& path);

private:

    std::string result_dir_;

    // map for various dirs
    std::map<std::string, std::string> dirs_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_FILE_IO_H
