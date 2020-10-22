/***
 * Use argv (ROS argv) for loading the configuration
 */

#ifndef PLANAR_RECT_HOMOGRAPHY_PKG_SETTINGS_H
#define PLANAR_RECT_HOMOGRAPHY_PKG_SETTINGS_H

#include <string>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include "logging_util.h"

using nlohmann::json;

class Settings {
public:
    std::string plane_extraction_method;            // only support "RANSAC"
    double ransac_min_perserve_ratio;               // i.e. 0.8
    double ransac_max_iteration;                    // i.e. 1000
    double ransac_distance_threshold;               // i.e. 0.08

    std::string result_saving_dir;

    std::string color_frame_;           // the pre-set color frame
    std::string cloud_frame_;           // the pre-set cloud fram

    // tf bags

    /***
     * Create the setting from the command-line arguments
     * @param argc
     * @param argv
     */
    Settings(int argc, char * argv[]);
    Settings();

    /***
     * Helper info displayed for usage
     */
    static void printUsage();

    void printParams();

    bool fromJSON(const std::string& filename);
    void toJSON(const std::string& filename);

    /***
     * An example of default values
     */
    void setDefault();
};


#endif //PLANAR_RECT_HOMOGRAPHY_PKG_SETTINGS_H
