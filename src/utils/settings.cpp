//
// Created by mars-lab on 19-4-19.
//

#include "settings.h"

Settings::Settings() {
    this->setDefault();
}

Settings::Settings(int argc, char **argv) {
    // if second param is -cfg, use config file instead of cmd parsing
    if (strcmp("-cfg", argv[1]) == 0){
        if (argc < 3){
            LOG_ERROR << "[ERROR] usage not accepted." << std::endl;
        }
        this->fromJSON(argv[2]);

        std::cerr << "" << std::endl;
        std::cerr << "" << std::endl;
        LOG_INFO << "Parameters Loaded "<< std::endl;
        return;
    }

    // TODO: add thresholding for lengh checking

    if (argc != 8) {
//        LOG_ERROR << "Invalid usage" << std::endl;
        Settings::printUsage();
        throw std::runtime_error("Invalid usage!");
    }


    // load parameters
    plane_extraction_method = std::string(argv[1]);
    ransac_min_perserve_ratio = std::stod(argv[2]);
    ransac_max_iteration = std::stoi(argv[3]);
    ransac_distance_threshold = std::stod(argv[4]);
    result_saving_dir = std::string(argv[5]);

    color_frame_ = std::string(argv[6]);
    cloud_frame_ = std::string(argv[7]);
}

void Settings::printUsage() {
    std::cerr << std::endl;
    std::cerr << "Usage: " << std::endl;
    std::cerr << "planar_rectifier_node ${plane_extraction_method} ${ransac_min_perserve_ratio} " <<
              "${ransac_max_iteration} ${ransac_distance_threshold} ${result_saving_dir}" << std::endl;

    std::cerr << "[WARN] API modifying" << std::endl;
    std::cerr << "planar_rectifier_node -cfg {PATH_TO_CONFIG}" << std::endl;
}

void Settings::printParams(){
    // print parameters
    std::cerr << "============= Parameters ===============" << std::endl;
    std::cerr << "plane_extraction_method: " << plane_extraction_method << std::endl;
    std::cerr << "ransac_min_perserve_ratio: " << ransac_min_perserve_ratio << std::endl;
    std::cerr << "ransac_max_iteration: " << ransac_max_iteration << std::endl;
    std::cerr << "ransac_distance_threshold: " << ransac_distance_threshold << std::endl;
    std::cerr << "result_saving_dir: " << result_saving_dir << std::endl;
    std::cerr << "============= End of Parameters ==========" << std::endl;
}

void Settings::toJSON(const std::string& filename) {
    json j;
    j["plane_extraction_method"] = plane_extraction_method;
    j["ransac_min_perserve_ratio"] = ransac_min_perserve_ratio;
    j["ransac_max_iteration"] = ransac_max_iteration;
    j["ransac_distance_threshold"] = ransac_distance_threshold;
    j["result_saving_dir"] = result_saving_dir;
    j["color_frame"] = color_frame_;
    j["cloud_frame"] = cloud_frame_;

    std::string content = j.dump();
    std::ofstream output(filename.c_str());
    output << content << std::endl;
}

bool Settings::fromJSON(const std::string& filename) {
    std::ifstream input(filename.c_str());
    json j;
    input >> j;

    plane_extraction_method = j["plane_extraction_method"];
    ransac_min_perserve_ratio = j["ransac_min_perserve_ratio"];
    ransac_max_iteration = j["ransac_max_iteration"];
    ransac_distance_threshold = j["ransac_distance_threshold"];
    result_saving_dir = j["result_saving_dir"];
    color_frame_ = j["color_frame"];
    cloud_frame_ = j["cloud_frame"];
}

void Settings::setDefault() {
    plane_extraction_method = "RANSAC";
    ransac_min_perserve_ratio = 0.9;
    ransac_max_iteration = 1000;
    ransac_distance_threshold = 0.04;
    result_saving_dir = "/home/mars-lab/caijx_ws/experiments/results";
    color_frame_ = "camera_color_optical_frame";
    cloud_frame_ = "camera_depth_optical_frame";
}