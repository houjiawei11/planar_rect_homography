//
// Created by mars-lab on 19-4-19.
//

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_PCL_HELPER_FUNCTIONS_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_PCL_HELPER_FUNCTIONS_H

#include <cmath>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include "logging_util.h"
#include "transform.h"
#include "viewpoint.h"
#include "Hazmat.h"

// ============== Normal Related =============
/***
 * Convert plane coefficient (from PCL::RANSAC) to plane normal
 * @param coeff: the plane coefficient provided by ransac
 * @return: Eigen::Vector4d: the plane (in normal form) (3-dimensional) + the distance to plane(1-dimensional)
 * @note: the model coefficient satisfy: ax + by + xz + d = 0
 *      Reference: http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
 */
Eigen::Vector4d plane_from_coeff(pcl::ModelCoefficients::Ptr coeff);

/***
 * Convert normal to euler angle
 * @param normal: the normal vector is assumed normalized vector (i.e. first three-element has l2 norm of 1)
 * @return
 *      vector of 3 double: R_x, R_y, R_z
 */
std::vector<double> plane_to_rpy(const Eigen::Vector4d &plane);

/***
 * Get an unique normal that represent the plane
 * All plane has two normals with opposite direction.
 * The function return the normal facing opposite the viewpoint (z>0)
 * @param normal
 * @return
 */
//Eigen::Vector4d unique_plane(const Eigen::Vector4d &plane);

Eigen::Vector4d unique_plane(const Eigen::Vector4d &plane, pcl::PointXYZ centroid);

// ================= PointCloud Related ===============
/***
 * Assign color to the pointcloud
 * Copy the input pointcloud and return a colored version
 * Note: the color is assigned based on the id
 * @param cloud_in
 * @return a colored pointcloud
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, size_t id);

/***
 * Calculate plane centroid from the point cloud
 * @return 3D Point (PointXYZ)
 */
pcl::PointXYZ calc_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

// ==================== Transform Related ===============

/***
 * Calculate the transfrom from two quaternion
 * Refer to for details: https://stackoverflow.com/questions/19445934/quaternion-from-two-vector-pairs
 * @param u0
 * @param v0
 * @param u2
 * @param v2
 * @return
 */
Eigen::Quaterniond qFromTwoVectorPairs(Eigen::Vector3d u0, Eigen::Vector3d v0, Eigen::Vector3d u2, Eigen::Vector3d v2);

/***
 * Project v0 to the plane with normal of n0
 * @return the projected normal
 */
Eigen::Vector3d projectPlane(Eigen::Vector3d v0, Eigen::Vector3d u0);

// ========================= Homography related =======================
//Eigen::Vector3d randomPointFromPlane(const Eigen::Vector4d& plane);
//std::vector<Eigen::Vector3d> randomPointsFromPlane(const Eigen::Vector4d& plane, size_t num_points);

/***
 * Get the four points from the plane, which has x,y = (-1,-1), (-1, 1), (1, -1), (1, 1)
 * Note: if it is not possible, use random points on the plane instead
 * @param plane
 * @return
 */
std::vector<Eigen::Vector3d> fourRectPointsFromPlane(const Eigen::Vector4d& plane);

float randomFloat();

/***
 * Apply transform
 * @param points_a: points in frame a
 * @param tf_ab: transform from a to b
 * @return
 */
std::vector<Eigen::Vector3d> applyTransform(const std::vector<Eigen::Vector3d> &points_a, Transform tf_ab);
std::vector<cv::Point2f> projectToImage(const std::vector<Eigen::Vector3d>& points, const Eigen::Matrix3d& camera);
// ========================= Homography related =======================
/***
 * Calculate the homography to warp image from frame a to frame b
 * @param plane: the plane in frame a
 * @param tf_ab: transform from frame a to frame b
 * @param camera: the camera intrinsic matrix (3*3)
 * @return
 */
cv::Mat calc_homography(Eigen::Vector4d plane, Transform tf_ab, Eigen::Matrix3d camera);

/***
 * De-project one point to the plane
 * @param point  (x, y, 1)
 * @param normal: the normal of the plane
 * @param d: ax + by + cz + d = 0;
 * @return 3d point coordinate
 */
Eigen::Vector3d deprojectionToPlane(Eigen::Vector3d point, Eigen::Vector3d normal, double d, Eigen::Matrix3d camera);

std::string string_padding(const std::string& x, int width);

/***
 * Perform pose processing on the homography so that the origin image can be rectified and get one single image
 * @param H : the original homography
 * @return A post-processed homography matrix
 */
cv::Mat postprocess_single_img(const cv::Mat H, size_t img_width, size_t img_height,
                               const std::vector<Eigen::Vector3d>& boarder_pts1);

/***
 * Keep a fixed distance to the plane and generate multiple homography to cover the whole plane
 * @param H
 * @param img_width
 * @param img_height
 * @param boarder_pts1
 * @return
 */
std::vector<cv::Mat> postprocess_multiple_img(const cv::Mat H, int img_width, int img_height,
                               const std::vector<Eigen::Vector3d>& boarder_pts1);

/***
 * Transform 2D points from one image to another image with the provided homography transform
 * @return
 */
std::vector<Eigen::Vector3d> perspectiveTransform(cv::Mat H, size_t img_width, size_t img_height,
                                                  const std::vector<Eigen::Vector3d>& boarder_pts1);

std::vector<Hazmat> filter_hazmat(std::vector<Hazmat> x);

#endif //PLANAR_RECT_HOMOGRAPHY_PKG_PCL_HELPER_FUNCTIONS_H
