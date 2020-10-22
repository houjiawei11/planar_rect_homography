/***
 * ROS Node for Planar Rectification
 * Algorithm:
 *  1. receive one complete frame (color, pointcloud) and assemble to one Frame
 *  2. estimate planes and get normals, for each RGBD-Frame
 *  3. calculate the new viewpoints
 *  4. calculate the homography and do the rectification
 * Visualization (via ROS):
 *  1. the input frames (assemabled)
 *  2. the segamented planes
 *  3. the new viewpoints
 *  4. the result image (rectified images)
 *  Note: visualization in each object is achieved by invoking ROSVisualizer
 * File IO: (will be implemented later)
 *  1. the input frames
 *  2. the segmentation planes
 *  3. the result images
 *
 * Note:
 *  - frame assembling currently record all frames. Implement it with auto-expiring in future version
 */

#include <ros/ros.h>
#include "settings.h"
#include "file_io.h"
#include "planar_rectifier.h"

int main(int argc, char * argv[]){
    ros::init(argc, argv, "planar_rect_homography_node");
    ros::NodeHandlePtr nh(new ros::NodeHandle);

    // create instance of rectifier
    std::shared_ptr<Settings> settings = std::make_shared<Settings>(Settings(argc, argv));
    settings->printParams();

    const std::string result_dir = settings->result_saving_dir;
    std::shared_ptr<FileIO> io_handler = std::make_shared<FileIO>(result_dir);

    PlanarRectifier rectifier(nh, settings, io_handler);

    ros::spin();
    return 0;
}
