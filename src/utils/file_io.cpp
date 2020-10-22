//
// Created by mars-lab on 19-4-23.
//

#include "file_io.h"

FileIO::FileIO(const std::string& result_dir) : result_dir_(result_dir) {
    LOG_INFO << "Initializing file IO: " << result_dir_ << std::endl;

    if (result_dir_.empty()){
        LOG_WARN << "result directory not set!" << std::endl;
    }
    else{       // create dir structure
        if (result_dir_[result_dir_.size()-1] == '/'){
            result_dir_.pop_back();
        }

        // rename existing dir to archived
        if (dirExists(result_dir_)) {
            int i = 1;
            while (true) {
                auto arch_dir = result_dir_ + "_" + std::to_string(i);
                if (!dirExists(arch_dir)){
                    LOG_INFO << "Renaming " << result_dir_ << " to " << arch_dir << std::endl;
                    std::rename(result_dir_.c_str(), arch_dir.c_str());
                    break;
                }
                i++;
            }
        }

        LOG_INFO << "File IO initialized: " << result_dir_ << std::endl;

        dirs_["RawImages"] = result_dir_ + "/RawImages";
        dirs_["RectifiedImages"] = result_dir + "/RectifiedImages";
        dirs_["DetectionResultRaw"] = result_dir + "/DetectionResultRaw";
        dirs_["DetectionResultSingleRect"] = result_dir + "/DetectionResultSingleRect";
        dirs_["DetectionResultMultiRect"] = result_dir + "/DetectionResultMultiRect";

        // create file structure
        mkdir(result_dir_.c_str(), 0755);
        for (auto& p : dirs_){
            mkdir(p.second.c_str(), 0755);
        }
    }
}


int FileIO::dirExists(const char *path){
    struct stat info;

    if(stat( path, &info ) != 0)
        return false;
    else if(info.st_mode & S_IFDIR)
        return true;
    else
        return false;
}

int FileIO::dirExists(const std::string& path){
    return dirExists(path.c_str());
}

void FileIO::save_input_img(const cv::Mat &img, int frame_id) {
    std::string filename = dirs_["RawImages"]+"/" + string_padding(std::to_string(frame_id), 6) + ".png";
    cv::imwrite(filename, img);
}

void FileIO::save_detection_result_raw(const cv::Mat &img, int frame_id) {
    std::string filename = dirs_["DetectionResultRaw"]+"/" + string_padding(std::to_string(frame_id), 6) + ".png";
    cv::imwrite(filename, img);
}

void FileIO::save_detection_result_rect_single(const cv::Mat &img, int frame_id, int sub_img_id) {
    std::string dir_path = dirs_["DetectionResultSingleRect"] + "/" + string_padding(std::to_string(frame_id), 6);
    if (!dirExists(dir_path)){
        mkdir(dir_path.c_str(), 0755);
    }

    // the filename for the image (full path)
    std::string img_filename = dir_path + "/" + string_padding(std::to_string(sub_img_id), 3) + ".png";
    cv::imwrite(img_filename, img);
}

void FileIO::save_detection_result_rect_multi(const cv::Mat &img, int frame_id, int sub_img_id) {
    std::string dir_path = dirs_["DetectionResultMultiRect"] + "/" + string_padding(std::to_string(frame_id), 6);
    if (!dirExists(dir_path)){
        mkdir(dir_path.c_str(), 0755);
    }

    // the filename for the image (full path)
    std::string img_filename = dir_path + "/" + string_padding(std::to_string(sub_img_id), 3) + ".png";
    cv::imwrite(img_filename, img);
}

void FileIO::save_rectified_img(const cv::Mat &img, const cv::Mat H, int frame_id, int sub_img_id) {
    std::string dir_path = dirs_["RectifiedImages"] + "/" + string_padding(std::to_string(frame_id), 6);
    if (!dirExists(dir_path)){
        mkdir(dir_path.c_str(), 0755);
    }

    // the filename for the image (full path)
    std::string img_filename = dir_path + "/" + string_padding(std::to_string(sub_img_id), 3) + ".png";
    cv::imwrite(img_filename, img);

    // save the homography transform from original image to the current image
    std::string H_filename = dir_path + "/" + string_padding(std::to_string(sub_img_id), 3) + ".txt";
    std::ofstream out(H_filename, std::fstream::out);
    for (int i = 0; i< 3; i++){
        for (int j = 0; j < 3; j++){
            out << H.at<double>(i,j);
            if ((i != 2) || (j != 2)){
                out << " ";
            }
        }
    }
    out << std::endl;
}