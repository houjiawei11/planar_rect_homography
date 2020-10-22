//
// Created by mars-lab on 19-4-19.
//

#include <chrono>
#include "frame.h"

Frame::Frame(ros::Time timestamp, size_t init_id,
        std::shared_ptr<Settings> settings,
        std::shared_ptr<ROSVisualizer> visualizer,
        std::shared_ptr<Camera> camera,
        std::shared_ptr<FileIO> io_handler,
        std::shared_ptr<ExternalDetector> external_detector): timestamp_(timestamp), cloud_(),
                                            color_image(), cloud_frame_id(),
                                            color_img_frame_id(), cloud_ready_(false),
                                            color_image_ready_(false), planes_(), camera_(camera),
                                            init_id_(init_id), settings_(settings), id_(0),
                                            visualizer_(visualizer), io_handler_(io_handler),
                                            external_detector_(external_detector) {

}

void Frame::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Time timestamp, std::string frame_id) {
    if (cloud_ready_){
        LOG_ERROR << "cloud has been set before this assignment!" << std::endl;
        throw std::runtime_error("Duplicate initialization");
    }

    if (timestamp_ != timestamp){
        LOG_ERROR << "Timestamp does not match! " << std::endl;
        LOG_ERROR << "Frame Timestamp: " << timestamp_ << std::endl;
        LOG_ERROR << "Input Timestamp: " << timestamp << std::endl;
    }

    cloud_ = cloud;
    cloud_frame_id = frame_id;
    cloud_ready_ = true;
}

void Frame::addColorImage(cv::Mat img, ros::Time timestamp, std::string frame_id) {
    if (color_image_ready_){
        LOG_ERROR << "image has been set before this assignment!" << std::endl;
        throw std::runtime_error("Duplicate initialization");
    }

    if (timestamp_ != timestamp){
        LOG_ERROR << "Timestamp does not match! " << std::endl;
        LOG_ERROR << "Frame Timestamp: " << timestamp_ << std::endl;
        LOG_ERROR << "Input Timestamp: " << timestamp << std::endl;
    }

    color_image = img;
    color_img_frame_id = frame_id;
    color_image_ready_ = true;
}

bool Frame::isReady() {
    if (cloud_ready_ && color_image_ready_){
        return true;
    } else{
        return false;
    }
}


void Frame::process() {
    // extract planes
    planeExtractionRANSAC();

    // visualize all the planes
    // for visualization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto plane: planes_){
//        auto debug_cloud = color_pointcloud(plane->cloud_, plane->id_);
        *cloud_vis += (*color_pointcloud(plane->cloud_, plane->id_));
    }
    // Note: the frame id is kept same with the input frame, as there is no transformation happened so far
    visualizer_->publishPointcloud("segamented/cloud", cloud_vis, cloud_frame_id);
    // visualize the plane direction vector
    std::vector<Viewpoint> vps_vis;
    // just for visualizing the centroids
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid_vis(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto plane: planes_){
        // visualize the centroid
        pcl::PointXYZ centroid_p = calc_centroid(plane->cloud_);
        cloud_centroid_vis->points.push_back(centroid_p);
        std::cout << "plane_id: " << plane->id_ << "     # of points: " << plane->cloud_->points.size() << std::endl;

        // visualize the plane
        auto vp_vis = plane->calc_plane_vis();
        vps_vis.push_back(vp_vis);
    }
    visualizer_->publishViewpoint("planes", vps_vis, cloud_frame_id);
    visualizer_->publishPointcloud("plane_centroid", cloud_centroid_vis, cloud_frame_id);


    // do detection on the raw image (input image)
    // TODO: disable detector for now
//    auto detection_result_raw = external_detector_->detect_hazmat_raw(this->color_image);
//    LOG_INFO << "Number of detection from raw input image: " << detection_result_raw.size() << std::endl;
//    cv::Mat detection_img_input = ExternalDetector::gen_vis_im(this->color_image, detection_result_raw);
//    io_handler_->save_detection_result_raw(detection_img_input, id_);


    // calculate new transform to viewpoint (for each plane)
    // calculate the homography for rectification
    std::vector<Transform> vp_tfs;
//    std::string frame_id;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_polygon_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    int rectified_img_sub_id = 0; // depend on each frame
    for (auto plane: planes_){
        Transform vp = plane->calc_viewpoint_tf();
        vp_tfs.push_back(vp);

        // the polygon of the plane
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_poly = plane->get_convate_polygon();
        *plane_polygon_vis += (*color_pointcloud(plane_poly, plane->id_));
        // project to 2D
        std::vector<Eigen::Vector3d> boarder_pts_im;            // boarders points on image space

        cv::Mat im_vis = cv::Mat::zeros(color_image.size(), color_image.type());
        color_image.copyTo(im_vis);

        for (auto& p: plane_poly->points){
            Eigen::Vector3d pt_3d = Eigen::Vector3d(p.x, p.y, p.z);
            Eigen::Vector3d pt_2d = camera_->toEigenK() * pt_3d;
            pt_2d(0) /= pt_2d(2);
            pt_2d(1) /= pt_2d(2);
            pt_2d(2) /= pt_2d(2);
            boarder_pts_im.push_back(pt_2d);

//            if ((plane->id_ == 1) || (plane->id_ == 0)) {
                // visualize the 2d points on images
//                for (int i = -2; i < 3; i++){
//                    for (int j = -2; j < 3; j++){
//                        int p_x = pt_2d(1) + i;
//                        int p_y = pt_2d(0) + j;
//
//                        if ((p_x < 0) || (p_x >= 640) || (p_y < 0) || (p_y >= 480)){
//                            continue;
//                        }
//                        cv::Vec3b& tmp = im_vis.at<cv::Vec3b>(p_x, p_y);
//                        tmp[0] = (uchar)0;
//                        tmp[1] = (uchar)0;
//                        tmp[2] = (uchar)0;
//                    }
//                }
//                im_vis.at<cv::Vec3b>((int)pt_2d(0), (int)pt_2d(1)) = cv::Vec3b(0, 0, 0);

//                cv::imwrite("/home/mars-lab/caijx_ws/catkin_ws/output.png", im_vis);

//                visualizer_->publishImage("im_vis", im_vis, color_img_frame_id);\
//                LOG_INFO << "Points (2D): " << "plane_id: " << plane->id_ << std::endl;
//                std::cout << pt_2d << std::endl;
//                LOG_INFO << "Points (3D): " << "plane_id: " << plane->id_ << std::endl;
//                std::cout << pt_3d << std::endl;

//                std::cout << std::endl;

//            }
        }

        // calculate the H
        cv::Mat H = calc_homography(plane->plane_, vp.inverse(), camera_->toEigenK());

        if (false){     // single image rectification (method 1)
            cv::Mat H_rect = postprocess_single_img(H, this->color_image.cols, this->color_image.rows, boarder_pts_im);

            // rectify the image and publish it
            cv::Mat result_im = cv::Mat::zeros(color_image.size(), color_image.type());

            cv::warpPerspective(color_image, result_im, H_rect, result_im.size());

            // save and visualize
            io_handler_->save_rectified_img(result_im, H_rect, id_, rectified_img_sub_id);

            // publish rectified image for visualization
//            visualizer_->publishImage("rectified_img", result_im, color_img_frame_id);

            // do detection and save the output image
            std::vector<Hazmat> detection_result_rect = filter_hazmat(external_detector_->detect_hazmat_raw(result_im));
            LOG_INFO << "Number of detection from rect image: " << detection_result_rect.size() << std::endl;
            cv::Mat detection_img_rect = ExternalDetector::gen_vis_im(result_im, detection_result_rect);
            io_handler_->save_detection_result_rect_single(detection_img_rect, id_, rectified_img_sub_id);

            rectified_img_sub_id ++;
        }

        if(true){           // multiple image rectification (method 2)
            // TODO: use shared_ptr to avoid copying
            std::vector<cv::Mat> H_rects = postprocess_multiple_img(H,
                    this->color_image.cols,
                    this->color_image.rows, boarder_pts_im);
            for (cv::Mat& H_rect : H_rects){
                // rectify the image and publish it
                cv::Mat result_im = cv::Mat::zeros(color_image.size(), color_image.type());

                cv::warpPerspective(color_image, result_im, H_rect, result_im.size());

                // save and visualize
                io_handler_->save_rectified_img(result_im, H_rect, id_, rectified_img_sub_id);

//                visualizer_->publishImage("rectified_img", result_im, color_img_frame_id);

                // do detection and save the output image
                // TODO: disable SIFT detector for now
//                std::vector<Hazmat> detection_result_rect = filter_hazmat(external_detector_->detect_hazmat_raw(result_im));
//                LOG_INFO << "Number of detection from rect image: " << detection_result_rect.size() << std::endl;
//                cv::Mat detection_img_rect = ExternalDetector::gen_vis_im(result_im, detection_result_rect);
//                io_handler_->save_detection_result_rect_multi(detection_img_rect, id_, rectified_img_sub_id);

                // warp back to get the detection result on original image
//                for (int i = 0; i < detection_result_rect.size(); i++){
//                    detection_result_rect[i].warpback(H_rect.inv());
//                }
//                cv::Mat detection_img_rect = ExternalDetector::gen_vis_im(this->color_image, detection_result_rect);
//                io_handler_->save_detection_result_rect_multi(detection_img_rect, id_, rectified_img_sub_id);

                rectified_img_sub_id ++;
            }
        }
    }
    // visualize the plane polygon
    visualizer_->publishPointcloud("plane_polygon", plane_polygon_vis, cloud_frame_id);
    // visualize new transform to viewpoints (for each plane)
    visualizer_->publishTransform("viewpoin_tfs", vp_tfs, cloud_frame_id);

    // notify other nodes of the current status (one image get processed)
    visualizer_->publishStatusFinishedOnce();
    LOG_INFO << "image processed" << std::endl;
}


void Frame::planeExtractionRANSAC() {
    // TODO: new parameter to be added
    int max_num_planes = 1;

    if (!planes_.empty()){
        LOG_ERROR << "plane vectors are not empty now. Maybe planes have been extracted before." << std::endl;
        throw std::runtime_error("Internal Error: vector planes_ is non-empty!");
    }

    // Create the segmentation object and set configuration
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(settings_->ransac_max_iteration);
    seg.setDistanceThreshold(settings_->ransac_distance_threshold);

    // Create the filtering object (for later swaping the pointcloud)
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // produce a local pointcloud for RANSAC usage
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_local = *cloud_;
    int nr_points = (int) cloud_local->points.size();
    int plane_id = 0;

    // cloud_p: contains points within the plane
    // cloud_f: contains points outside of the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>),
            cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    // While >30% of the original cloud is still there
    while (cloud_local->points.size() > (1-settings_->ransac_min_perserve_ratio) * nr_points) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_local);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty()) {
            LOG_ERROR << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_local);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

//        LOG_INFO << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height
//                 << " data points." << std::endl;
//        LOG_INFO << "Model Coefficients: ";

        // outliner removal
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud_p);
        sor.setMeanK (100);                     // TODO: new parameter to be added
        sor.setStddevMulThresh (3.0);           // TODO: new parameter to be added
        sor.filter (*cloud_filtered);

        LOG_DEBUG << "Model Coefficients: ";
        for (int i = 0 ; i < coefficients.get()->values.size(); i++){
            std::cout << coefficients.get()->values[i] << "  ";
        }
        std::cout << std::endl;

        // Save the pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_save(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud_to_save = *cloud_filtered;
        // TODO: optimize centroid calculation: this is duplacated
        pcl::PointXYZ centroid = calc_centroid(cloud_to_save);
        Eigen::Vector4d plane_normal = unique_plane(plane_from_coeff(coefficients), centroid);
        if (std::abs(plane_normal[1]) < 0.8){
            // if it is not a ground plane, keep it
            // otherwise, just skip it
            std::shared_ptr<Plane> plane_ptr(new Plane(cloud_to_save, plane_normal, plane_id));
            planes_.push_back(plane_ptr);
            plane_id ++;
        }
        else{
            LOG_DEBUG << "Skipping this plane as it is a ground plane" << std::endl;
        }

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_local.swap (cloud_f);

        if (planes_.size() == max_num_planes){
            break;
        }
    }

    LOG_DEBUG << "Extracted " << planes_.size() << " from RGB-D Image" << std::endl;
}

