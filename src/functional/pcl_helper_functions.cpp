//
// Created by mars-lab on 19-4-19.
//

#include <Hazmat.h>
#include "pcl_helper_functions.h"

Eigen::Vector4d plane_from_coeff(pcl::ModelCoefficients::Ptr coeff){
    // plane model with normal vector (ax+by+cz+d=0)
    Eigen::Vector4d model_normal = Eigen::Vector4d::Zero();
    model_normal(0) = coeff->values[0];
    model_normal(1) = coeff->values[1];
    model_normal(2) = coeff->values[2];

    // normalize them
    double z = sqrt(pow(model_normal(0), 2.0) + pow(model_normal(1), 2.0) + pow(model_normal(2), 2.0));
    model_normal(0) = model_normal(0) / z;
    model_normal(1) = model_normal(1) / z;
    model_normal(2) = model_normal(2) / z;

    // set d (distance to plane)
    model_normal(3) = coeff->values[3] / z;

    return model_normal;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                                            size_t id){
    std::vector<uint8_t> color = {0,0,0};
    auto id_tmp = id % 4;
    if (id_tmp == 0)                color = {255, 0 ,0};
    else if (id_tmp == 1)           color = {0, 255, 0};
    else if (id_tmp == 2)           color = {0, 0, 255};

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *result_cloud = *cloud_in;
    for (size_t i = 0; i < result_cloud->points.size(); i++){
        result_cloud->points[i].r = color[0];
        result_cloud->points[i].g = color[1];
        result_cloud->points[i].b = color[2];
    }

    return result_cloud;
}

pcl::PointXYZ calc_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
    for (auto p : cloud_in->points){
        centroid.add(p);
    }

    pcl::PointXYZ c;
    centroid.get(c);
    return c;
}

Eigen::Vector4d unique_plane(const Eigen::Vector4d &plane){
    // TODO: this function might be problematic
    Eigen::Vector4d result_plane = plane;
    if (result_plane(2) > 0){
        return result_plane;
    }
    else if (result_plane(2) < 0){
        return -1 * result_plane;
    }
    else{       // d == 0
        LOG_WARN << "One plane is going through origin! " <<
                    "The current implementation does not support this!" << std::endl;
        return result_plane;
    }
}

Eigen::Vector4d unique_plane(const Eigen::Vector4d &plane, pcl::PointXYZ centroid){
    Eigen::Vector3d centroid_v(centroid.x, centroid.y, centroid.z);
    Eigen::Vector3d plane_normal(plane(0), plane(1), plane(2));

    if (centroid_v.dot(plane_normal) > 0){
        return plane;
    }
    else if (centroid_v.dot(plane_normal) < 0){
        return -1 * plane;
    }
    else{
        LOG_WARN << "plane normal is exactly vertical " <<
                 "The current implementation does not support this!" << std::endl;
        return plane;
    }
}

//std::vector<double> plane_to_rpy(const Eigen::Vector4d &plane){
//    double yaw = atan2(plane(1), plane(0));
//    double pitch = -1 * atan2(plane[2], std::sqrt(std::pow(plane(0), 2) + std::pow(plane(1), 2)));
//
//    // Note: Because the roll can not be obtained from direction vector, it is set as 0.0
////    double pitch_angle = pitch / M_PI * 180.0;
////    double yaw_angle = yaw / M_PI * 180;
//
//    std::vector<double> out = {0.0, pitch, yaw};
//
//    return out;
//}

Eigen::Quaterniond qFromTwoVectorPairs(Eigen::Vector3d u0, Eigen::Vector3d v0, Eigen::Vector3d u2, Eigen::Vector3d v2){
    u0.normalize();
    v0.normalize();
    u2.normalize();
    v2.normalize();

    Eigen::Quaterniond q2 = Eigen::Quaterniond::FromTwoVectors(u0, u2);
    Eigen::Vector3d v1 = q2.conjugate() * v2;
//    Eigen::Vector3d v0_proj = v0.projectPlane(u0);
    Eigen::Vector3d v0_proj = projectPlane(v0, u0);
//    Vector v1_proj = v1.projectPlane(u0);
    Eigen::Vector3d v1_proj = projectPlane(v1, u0);
    Eigen::Quaterniond q1 = Eigen::Quaterniond::FromTwoVectors(v0_proj, v1_proj);
    return (q2 * q1).normalized();
}

Eigen::Vector3d projectPlane(Eigen::Vector3d v, Eigen::Vector3d n){
    Eigen::Vector3d result = v - (v.dot(n) / n.norm()) * n;
    return result.normalized();
}

std::vector<Eigen::Vector3d> fourRectPointsFromPlane(const Eigen::Vector4d& plane){
    if (plane[2] != 0) {            // the most common case
        std::vector<Eigen::Vector3d> result;

        result.push_back(Eigen::Vector3d(-1, -1, 0));
        result.push_back(Eigen::Vector3d(-1, 1, 0));
        result.push_back(Eigen::Vector3d(1, -1, 0));
        result.push_back(Eigen::Vector3d(1, 1, 0));

        for (int i = 0; i < 4; i++){
            Eigen::Vector3d& v = result[i];
            v[2] = -1 * (plane(3) + plane(0) * v[0] + plane(1) * v[1]) / plane(2);
        }
        return result;
    }
    else{
        std::vector<Eigen::Vector3d> result;

        if (plane[0] == 0){
            for (int i = 0; i < 4; i++){
                result.push_back(Eigen::Vector3d(randomFloat(), -1 * plane(3) / plane(1), randomFloat()));
            }
            return result;
        }
        if (plane[1] == 0){
            for (int i = 0; i < 4; i++){
                result.push_back(Eigen::Vector3d(-1 * plane(3) / plane(0), randomFloat(), randomFloat()));
            }
            return result;
        }
        // ax + by + d = 0
        for (int i = 0; i < 4; i++){
            float x = randomFloat();
            result.push_back(Eigen::Vector3d(x, -1 * (plane(3) + plane(0) * x) / plane(1), randomFloat()));
        }
        return result;
    }
}

float randomFloat(){
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    return r;
}

std::vector<Eigen::Vector3d> applyTransform(const std::vector<Eigen::Vector3d> &points_a, Transform tf_ab){
    Eigen::Matrix3d R = tf_ab.R.toRotationMatrix();
    Eigen::Vector3d T = tf_ab.T;

    std::vector<Eigen::Vector3d> result;
    for (const Eigen::Vector3d& p: points_a){
        Eigen::Vector3d p_b = (R * p) + T;
        result.push_back(p_b);
    }
    return result;
}

std::vector<cv::Point2f> projectToImage(const std::vector<Eigen::Vector3d>& points, const Eigen::Matrix3d& camera){
    std::vector<cv::Point2f> im_points;
    for (const Eigen::Vector3d& p : points){
        Eigen::Vector3d p_im = camera * p;
        cv::Point2f p_cv(p_im(0) / p_im(2), p_im(1) / p_im(2));
        im_points.push_back(p_cv);
    }
    return im_points;
}

cv::Mat calc_homography(Eigen::Vector4d plane, Transform tf_ab, Eigen::Matrix3d camera){
    // The current implementation use 4-point algorithm to calculate the homography
    // Given 4 points on the plane, calculate the projection points on Frame a and b
    // Then the H matrix is obtained via cv::findHomography with the 4 corresponding point pairs.

    std::vector<Eigen::Vector3d> points_3d_a = fourRectPointsFromPlane(plane);
    std::vector<Eigen::Vector3d> points_3d_b = applyTransform(points_3d_a, tf_ab);
    std::vector<cv::Point2f> points_im_a = projectToImage(points_3d_a, camera);
    std::vector<cv::Point2f> points_im_b = projectToImage(points_3d_b, camera);

    //  the resulting homography matrix
    cv::Mat H = cv::findHomography(points_im_a, points_im_b);
    return H;
}

Eigen::Vector3d deprojectionToPlane(Eigen::Vector3d point, Eigen::Vector3d normal, double d, Eigen::Matrix3d camera){
    Eigen::Matrix<double, 3, 1> normal_m = Eigen::Matrix<double, 3, 1>::Zero();
    normal_m(0) = normal(0);
    normal_m(1) = normal(1);
    normal_m(2) = normal(2);

    Eigen::Matrix3d A = normal_m * normal_m.transpose() - camera;
    Eigen::Vector3d B = -1 * normal_m * d;

    Eigen::MatrixXd A_dynamic = A;
    Eigen::VectorXd B_dynamic = B;

//    std::cout << "B: " <<  B << std::endl;
    std::cout << "Solvable: " << Eigen::FullPivLU<Eigen::Matrix3d>(A).isInvertible() << std::endl;

    Eigen::Matrix<double, 3, 1> x = A_dynamic.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_dynamic);

    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    result(0) = x(0);
    result(1) = x(1);
    result(2) = x(2);

    std::cout << "A: " << A << std::endl;
    std::cout << "A*x: " << A * x << std::endl;
    std::cout << "B: " << B << std::endl;
    std::cout << "x: " <<  x << std::endl;

    return result;
}

std::string string_padding(const std::string& x, int width){
    std::string result = x;

    if(width > result.size())
        result.insert(0, width - result.size(), '0');

    return result;
}

cv::Mat postprocess_single_img(const cv::Mat H, size_t img_width, size_t img_height,
                               const std::vector<Eigen::Vector3d>& boarder_pts1){
    // choose the minimum of the two numbers
    auto min = [](double x, double y){
        if (x > y)          return y;
        else                return x;
    };
    auto max = [](double x, double y){
        if (x < y)          return y;
        else                return x;
    };

    cv::Mat H_rect = H.clone();
    // boarder points in the second image
    auto boarder_pts_2 = perspectiveTransform(H, img_width, img_height, boarder_pts1);

    // calculate the translation and scaling matrix for warping the whole original image
//    // translation matrix
//    double x_min = min(boarder_pts_2[0][0], boarder_pts_2[2][0]);
//    double y_min = min(boarder_pts_2[0][1], boarder_pts_2[1][1]);
//
//    // compute the translation matrix
//    // the homography are of the type double (CV_64F)
//    cv::Mat T = cv::Mat::eye(H.size(), H.type());
//    T.at<double>(0, 2) = double(-1 * x_min);
//    T.at<double>(1, 2) = double(-1 * y_min);
//
//    // compute the resizing matrix: resize the image so that it fits in the image
//    // the width and height of the new image
//    double width_2 = max(boarder_pts_2[1][0] - boarder_pts_2[0][0], boarder_pts_2[3][0] - boarder_pts_2[2][0]);
//    double height_2 = max(boarder_pts_2[3][1] - boarder_pts_2[1][1], boarder_pts_2[2][1] - boarder_pts_2[0][1]);
//
//    double width_scale = img_width / width_2;
//    double height_scale = img_height / height_2;
//
//    cv::Mat S = cv::Mat::eye(H.size(), H.type());
//    S.at<double>(0, 0) = width_scale;
//    S.at<double>(1, 1) = height_scale;

    // calculate the translation and scaling matrix for warping only the target plane
    // translation matrix
    double x_min = boarder_pts_2[0][0];
    double x_max = boarder_pts_2[0][0];
    double y_min = boarder_pts_2[0][1];
    double y_max = boarder_pts_2[0][1];

    for (Eigen::Vector3d& pt : boarder_pts_2){
        if (pt[0] < x_min)          x_min = pt[0];
        if (pt[0] > x_max)          x_max = pt[0];
        if (pt[1] < y_min)          y_min = pt[1];
        if (pt[1] > y_max)          y_max = pt[1];
    }

    cv::Mat T = cv::Mat::eye(H.size(), H.type());       // translation matrix
    cv::Mat S = cv::Mat::eye(H.size(), H.type());       // scaling (resizing) matrix

    // translation
    T.at<double>(0, 2) = double(-1 * x_min);
    T.at<double>(1, 2) = double(-1 * y_min);

    // scaling
    double width_2 = x_max - x_min;
    double height_2 = y_max - y_min;

    double width_scale = img_width / width_2;
    double height_scale = img_height / height_2;
    S.at<double>(0, 0) = min(width_scale, height_scale);
    S.at<double>(1, 1) = min(width_scale, height_scale);

    H_rect = S * T * H;
//    LOG_INFO << T << std::endl;
//    LOG_INFO << H_rect << std::endl;
    return H_rect;
}

std::vector<cv::Mat> postprocess_multiple_img(const cv::Mat H, int img_width, int img_height,
                                 const std::vector<Eigen::Vector3d>& boarder_pts1){

    // choose the minimum of the two numbers
    auto min = [](double x, double y){
        if (x > y)          return y;
        else                return x;
    };
    auto max = [](double x, double y){
        if (x < y)          return y;
        else                return x;
    };


    std::vector<cv::Mat> results;


    cv::Mat H_rect = H.clone();
    // boarder points in the second image
    auto boarder_pts_2 = perspectiveTransform(H, img_width, img_height, boarder_pts1);

    // translation matrix
    double x_min = boarder_pts_2[0][0];
    double x_max = boarder_pts_2[0][0];
    double y_min = boarder_pts_2[0][1];
    double y_max = boarder_pts_2[0][1];

    for (Eigen::Vector3d& pt : boarder_pts_2){
        if (pt[0] < x_min)          x_min = pt[0];
        if (pt[0] > x_max)          x_max = pt[0];
        if (pt[1] < y_min)          y_min = pt[1];
        if (pt[1] > y_max)          y_max = pt[1];
    }

    double width_new = x_max - x_min;
    double height_new = y_max - y_min;

    // base translation matrix to warp the image to left-up corner
    cv::Mat T = cv::Mat::eye(H.size(), H.type());
    T.at<double>(0, 2) = double(-1 * x_min);
    T.at<double>(1, 2) = double(-1 * y_min);

    int num_img_x = int(width_new / img_width * 2) + 1;         // number of images along x direction
    int num_img_y = int(height_new / img_height * 2) + 1;
    for (int i = 0; i < num_img_x; i++){
        for (int j = 0; j < num_img_y; j++){
            // the new translation
            cv::Mat T_2 = cv::Mat::eye(H.size(), H.type());
            double x_offset = -1 * i * ((double)img_width / 2);
            double y_offset = -1 * j * ((double)img_height / 2);
            T_2.at<double>(0, 2) = x_offset;
            T_2.at<double>(1, 2) = y_offset;

//            LOG_INFO << "T_2: " <<  T_2 << std::endl;
            results.push_back(T_2 * T.clone() * H_rect.clone());
        }
    }
//    cv::Mat T_2 = cv::Mat::eye(H.size(), H.type());
////    T_2.at<double>(0,2) = double(0);
//    LOG_INFO << "T_2: " << T_2 << std::endl;
//    H_rect = T_2 * T * H.clone();
//    LOG_INFO << "H_rect: " << H_rect << std::endl;
//    results.push_back(H_rect);
    return results;
}

std::vector<Eigen::Vector3d> perspectiveTransform(cv::Mat H, size_t img_width, size_t img_height,
                                                  const std::vector<Eigen::Vector3d>& boarder_pts1){
    Eigen::Matrix3d H_eigen = Eigen::Matrix3d::Zero();
    cv::cv2eigen(H, H_eigen);

    std::vector<Eigen::Vector3d> results;
    for (const Eigen::Vector3d& x : boarder_pts1){
        Eigen::Vector3d x_new = H_eigen * x;
        x_new(0) /= x_new(2);
        x_new(1) /= x_new(2);
        x_new(2) /= x_new(2);
        results.push_back(x_new);
    }

    return results;
}

std::vector<Hazmat> filter_hazmat(std::vector<Hazmat> x){
    std::vector<Hazmat> result;
    for (const Hazmat& h : x){
        double x = (h.corners_[0].x + h.corners_[1].x + h.corners_[2].x + h.corners_[3].x) / 4;
        double y = (h.corners_[0].y + h.corners_[1].y + h.corners_[2].y + h.corners_[3].y) / 4;
        bool flag = true;
        for (int i = 0; i < 4; i++){
            if ((x - h.corners_[i].x) > 100)          flag = false;
            if ((y - h.corners_[i].y) > 100)          flag = false;
        }

        if (flag){
            result.push_back(h);
        }
    }

    return result;
}

