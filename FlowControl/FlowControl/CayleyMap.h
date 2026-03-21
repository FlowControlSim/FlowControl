#ifndef CAYLEY_MAP_H
#define CAYLEY_MAP_H

#include "helpers.h"
#include "SE3Transform.h"

class CayleyMap {
public:
	CayleyMap() {};
    ~CayleyMap() {};

    float sigma(Vector6D Y);
    SE3Transform cayley_map(Vector6D Y);
    Vector6D inverse_cayley_map(SE3Transform T);
    Matrix6d cayley_differential(Vector6D Y);
    Matrix6d inverse_cayley_differential(Vector6D Y);

    static Eigen::Matrix3d to_rotation_matrix(const Eigen::Vector3d& omega);
    static Eigen::Vector3d to_angular_velocity(const Eigen::Matrix3d& R);
};

#endif // CAYLEY_MAP_H
