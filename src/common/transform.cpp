//
// Created by mars-lab on 19-4-21.
//

#include "transform.h"


Transform::Transform() {
    T = Eigen::Vector3d::Zero();
    R = Eigen::Quaterniond::Identity();
}

Transform Transform::inverse() const {
    Transform tf_new;
    Eigen::Matrix<double, 3, 3> rot_mat = R.toRotationMatrix();
    auto rot_mat_new = rot_mat.transpose();

    Eigen::Vector3d t_new = -1 * rot_mat.transpose() * T;

    tf_new.T = t_new;
    tf_new.R = Eigen::Quaterniond(rot_mat_new);

    return tf_new;
}
