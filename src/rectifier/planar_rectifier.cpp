//
// Created by mars-lab on 19-4-19.
//

#include "planar_rectifier.h"

PlanarRectifier::PlanarRectifier(ros::NodeHandlePtr nh,
        std::shared_ptr<Settings> settings,
        std::shared_ptr<FileIO> io_handler) : node_handler_(nh), frames_(), input_subscibers_(), num_frames_(0),
                                                camera_sub_(), camera_(new Camera), camera_ready_(false),
                                                frames_timestamps_(), visualizer_(new ROSVisualizer(nh)),
                                                io_handler_(io_handler), num_ready_frames_(0),
                                                settings_(settings), tf_cloud2color_(),
                                                tf_cloud2_color_eigen_(Eigen::Matrix4f::Zero()),
                                                external_detector_(new ExternalDetector(nh)) {
    // activate camera info subscriber
    camera_sub_ = node_handler_->subscribe("camera_info", 1000, &PlanarRectifier::callbackCameraInfo, this);
    LOG_INFO << "waiting for camera_info" << std::endl;

//    input_subscibers_.push_back(node_handler_->subscribe("color_image", 1000, &PlanarRectifier::callbackColorImage, this));
//    input_subscibers_.push_back(node_handler_->subscribe("cloud", 1000, &PlanarRectifier::callbackCloud, this));
}


void PlanarRectifier::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg_ptr) {
    auto tfToMat4f = [](tf::Transform tf_ab){
        Eigen::Quaterniond q;
        tf::quaternionTFToEigen(tf_ab.getRotation(), q);
        Eigen::Matrix3d q_mat = q.toRotationMatrix();

        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        tf::vectorTFToEigen(tf_ab.getOrigin(), t);

        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                m(i,j) = q_mat(i,j);
            }
        }
        for (int i = 0; i < 3; i++){
            m(i, 3) = t(i);
        }

        return m;
    };


    // K
    cv::Mat K_new = cv::Mat_<double>(3,3);
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++) {
            K_new.at<double>(3*i+j) = msg_ptr->K.at(3*i+j);
        }
    }

    // D
    // For kinect and Realsense, the distortion is zero
    std::vector<double> D_new;
    for (int i = 0; i < msg_ptr->D.size(); i++){
        D_new.push_back(msg_ptr->D.at(i));
    }

//    LOG_DEBUG << "D ready" << std::endl;

    if (!camera_ready_) {       // camera not yet initialized
        camera_->set_from_topic(K_new, D_new, msg_ptr->height, msg_ptr->width);
        camera_ready_ = true;
        LOG_INFO << "camera info received and settled from ROS topic" << std::endl;
        std::cout << camera_->K() << std::endl;

        // look up transform
        // initialize the transform if not ready
        tf::StampedTransform tf_stamped;
        tf::TransformListener listener;
        while (ros::ok()){
            LOG_INFO << "waiting for transform from " << settings_->cloud_frame_ <<
                " to " << settings_->color_frame_ << std::endl;
            try {
                listener.lookupTransform(settings_->color_frame_,
                                         settings_->cloud_frame_,
                                         ros::Time(0),
                                         tf_stamped);
                break;
            }
            catch (tf::TransformException &ex){
                ros::Duration(0.5).sleep();
                continue;
            }
        }
        tf_cloud2color_ = tf::Transform(tf_stamped.getRotation(), tf_stamped.getOrigin());
        tf_cloud2_color_eigen_ = tfToMat4f(tf_cloud2color_);
        LOG_INFO << "tf_cloud2color ready!" << std::endl;


        // activate other subscribers
        input_subscibers_.push_back(node_handler_->subscribe("color_image", 1000, &PlanarRectifier::callbackColorImage, this));
        input_subscibers_.push_back(node_handler_->subscribe("cloud", 1000, &PlanarRectifier::callbackCloud, this));
    }
    else{
        // TODO: Not yet implemented. check if the received camera info is same as the one using now
        // This will be useful if the camera info is loaded from config file on the disk
    }
}

void PlanarRectifier::callbackColorImage(const sensor_msgs::ImageConstPtr &msg_ptr) {
    LOG_INFO << "Receive one frame of color image: " << msg_ptr->header.stamp << std::endl;

    // convert the color image from ros message
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_ptr, sensor_msgs::image_encodings::BGR8);

    cv::Mat img = cv_ptr->image;

    ros::Time ts = msg_ptr->header.stamp;
    if (frames_.count(ts)){             // the frame already exists
        frames_[ts]->addColorImage(img, ts, msg_ptr->header.frame_id);
    }
    else{
        frames_timestamps_.push_back(ts);
        frames_[ts] = std::make_shared<Frame>(
                Frame(ts, num_frames_, settings_, visualizer_, camera_, io_handler_, external_detector_)
                );
        num_frames_ ++;
        frames_[ts]->addColorImage(img, ts, msg_ptr->header.frame_id);
    }

    // trigger the processing steps when the frame is ready
    if (frames_[ts]->isReady()) {
        this->onFrameReady(ts);
    }
}

void PlanarRectifier::callbackCloud(const sensor_msgs::PointCloud2ConstPtr &msg_ptr) {

    LOG_INFO << "Receive one frame of cloud: " << msg_ptr->header.stamp << std::endl;

    // convert the pointcloud to pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_raw = planar_rectifier::fromROSMsg(msg_ptr);

    // filter the pointcloud
    // the raw pointcloud has lots of NaN / 0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto point : *pcl_cloud_raw){
        // filter out poins with NaN
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) continue;
        // filter out points with all zeros
        // Note: Generally, it is not possible for sensor to get input pointcloud of NaN
        if ((point.x == 0) && (point.y == 0) && (point.z == 0))         continue;
        else{
            pcl_cloud_filtered->push_back(point);
        }
    }

    // transform the pointcloud to color frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*pcl_cloud_filtered, *transformed_cloud, tf_cloud2_color_eigen_);

    ros::Time ts = msg_ptr->header.stamp;
    if (frames_.count(ts)){             // the frame already exists
        frames_[ts]->addCloud(transformed_cloud, ts, settings_->color_frame_);
    }
    else{
        frames_timestamps_.push_back(ts);
        frames_[ts] = std::make_shared<Frame>(
                Frame(ts, num_frames_, settings_, visualizer_, camera_, io_handler_, external_detector_)
                );
        num_frames_ ++;
        frames_[ts]->addCloud(transformed_cloud, ts, settings_->color_frame_);
    }

    // trigger the processing steps when the frame is ready
    if (frames_[ts]->isReady()) {
        this->onFrameReady(ts);
    }
}

void PlanarRectifier::onFrameReady(ros::Time ts) {
    LOG_INFO << "Frame with timestamp: " << ts << "is ready" << std::endl;

    frames_[ts]->id_ = num_ready_frames_;
    num_ready_frames_ ++;

    // visualize the input through visualizer
    visualizer_->publishImage("input/color/image", frames_[ts]->color_image, frames_[ts]->color_img_frame_id);
    visualizer_->publishPointcloud("input/cloud", frames_[ts]->cloud_, frames_[ts]->cloud_frame_id);

    // save input image to disk
    io_handler_->save_input_img(frames_[ts]->color_image, frames_[ts]->id_);

    frames_[ts]->process();
}