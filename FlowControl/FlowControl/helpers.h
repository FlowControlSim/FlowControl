#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>


// types
using vec3 = Eigen::Vector3d;
using vec6 = Eigen::Matrix<double, 6, 1>;
using mat3 = Eigen::Matrix3d;
using mat4 = Eigen::Matrix4d;
using Matrix6x6 = Eigen::Matrix<double, 6, 6>;

using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 3, 3> Matrix3d;
typedef Vector<double, 3> Vector3d;
typedef Vector<double, 6> Vector6d;


// utility functions

inline Matrix3d skew(const Vector3d& v) {
    Matrix3d m;
    m << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return m;
}

inline Vector3d unskew(const Matrix3d& m) {
	return Vector3d(m(2, 1), m(0, 2), m(1, 0));
}
