/***
 * Represent a sinlge plane
 *
 * Note: the plane normal is representation by an eigen vector (3*1)
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_PLANE_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_PLANE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include "pcl_helper_functions.h"
#include "viewpoint.h"
#include "transform.h"

class Plane {
public:
    Plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector4d plane, size_t id);

    /***
     * Calculate the transform to the new tf
     * Direction: transfrom points from new frame => original frame
     * @return
     */
    Transform calc_viewpoint_tf();

    /***
     * To visualize the plane
     * @return a viewpoint origins from plane centroid and points towards plane direction vector
     */
    Viewpoint calc_plane_vis();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_convate_polygon();

    // ================= Attributions ==================
    // TODO: attr should be private. For simpility, the current implmentation put them as public
    // containing all points belonging to the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // unique represention with direction vector toward origin
    Eigen::Vector4d plane_;

    size_t id_;
private:

};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_PLANE_H
