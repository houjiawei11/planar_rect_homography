/***
 * Single frame instance
 * Implement core functions for rectification algorithm:
 *  - extarct planes
 *  - calculate new viewpoints
 *  - do the reprojection
 *
 * Frame Creation Logic:
 *  - create a new frame (empty one)
 *  - add the color image
 *  - add the pointcloud
 *  - double-check that the frame containes all necessary inputs
 *  - change the ready flag to ready
 *
 * Note: the timestamp is used for aligning pointcloud and images
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_FRAME_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_FRAME_H


#include <vector>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include "plane.h"
#include "logging_util.h"
#include "ros_visualizer.h"
#include "settings.h"
#include "pcl_helper_functions.h"
#include "viewpoint.h"
#include "camera.h"
#include "file_io.h"
#include "ExternalDetector.h"

class Frame {
public:
    /***
     * Set timestamp upon creation
     * @param timestamp
     */
    Frame(ros::Time timestamp, size_t init_id,
            std::shared_ptr<Settings> settings,
            std::shared_ptr<ROSVisualizer> visualizer,
            std::shared_ptr<Camera> camera,
            std::shared_ptr<FileIO> io_handler,
            std::shared_ptr<ExternalDetector> external_detector);

    /***
     * Add the pointcloud into the frame
     * Check if the timestamp align with the given one
     */
    void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Time timestamp, std::string frame_id);

    /***
     * Add image into the frame
     * Check if the timestamp align with the given one
     */
    void addColorImage(cv::Mat img, ros::Time timestamp, std::string frame_id);

    /***
     * check if the frame is ready to get process
     * @return true if all input is ready, false otherwise
     */
    bool isReady();

    /***
     * Start processing the frame
     */
    void process();

    /***
     * Plane extarction:
     *  extract planes and save to planes_
     * @param params
     *  cloud_: the pointcloud of the frame (RGBXYZ)
     *  settings_: the configuration for plane extraction
     * @return
     */
    void planeExtractionRANSAC();

    // ============= Core algorithm ====================


    // ============== End of core algorithm ============

    // Public attributions
    // TODO: the current implementation put them as publish for simpility
    // TODO: these attr should be private with get(), change that later
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    cv::Mat color_image;
    std::string cloud_frame_id;                 // the frame id for pointcloud
    std::string color_img_frame_id;             // the frame id for color image

    ros::Time timestamp_;
    size_t init_id_;                    // the initial id used for all frames (including non-ready frames)
    size_t id_;                         // only frames with ready status get assigned a new id

    // General shared atti
    std::shared_ptr<Settings> settings_;
    std::shared_ptr<ROSVisualizer> visualizer_;
    std::shared_ptr<FileIO> io_handler_;

    // the planes
    std::vector<std::shared_ptr<Plane>> planes_;

    std::shared_ptr<Camera> camera_;

    std::shared_ptr<ExternalDetector> external_detector_;

private:
    bool cloud_ready_;
    bool color_image_ready_;
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_FRAME_H
