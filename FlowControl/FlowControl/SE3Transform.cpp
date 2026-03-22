#include "SE3Transform.h"

SE3Transform from_rotation_translation(Eigen::Matrix3d R, Eigen::Vector3d t) {
	SE3Transform T(Eigen::Matrix4d::Identity());
	T.data.block<3, 3>(0, 0) = R;
	T.data.block<3, 1>(0, 3) = t;
	return T;
}

Eigen::Matrix3d SE3Transform::R() {
	return Eigen::Matrix3d(this->data.block<3, 3>(0, 0));
}

Eigen::Vector3d SE3Transform::t() {
	return Eigen::Vector3d(this->data.block<3, 1>(0, 3));
}

SE3Transform SE3Transform::compose(Eigen::Matrix4d other) const {
	return SE3Transform(this->data * other);
}

SE3Transform SE3Transform::__matmul__(Eigen::Matrix4d other) {
	return this->compose(other);
}

SE3Transform SE3Transform::inverse() {
	Eigen::Matrix3d R_inv = this->R().transpose();
	Eigen::Vector3d t_inv = -R_inv * this->t();
	return from_rotation_translation(R_inv, t_inv);
}

Eigen::Vector3d SE3Transform::apply_to_vector(Eigen::Vector3d v) {
	return this->R() * v;
}

Eigen::Vector4d SE3Transform::apply_to_point(Eigen::Vector3d p) {
	Eigen::Vector4d p_homogeneous;
	p_homogeneous << p[0], p[1], p[2], 1.0;
	return this->data * p_homogeneous;
}


SE3Transform identity() {
	return SE3Transform(Eigen::Matrix4d::Identity());
}

SE3Transform from_translation(Eigen::Vector3d t) {
	return from_rotation_translation(Eigen::Matrix3d::Identity(), t);

}

SE3Transform from_rotation_z(double theta) {
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	R(0, 0) = cos(theta);
	R(0, 1) = -sin(theta);
	R(1, 0) = sin(theta);
	R(1, 1) = cos(theta);
	return from_rotation_translation(R, Eigen::Vector3d::Zero());
}