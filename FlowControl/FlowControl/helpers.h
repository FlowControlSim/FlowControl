#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>


// types
using vec3 = std::array<double, 3>;
using mat3 = std::array<std::array<double, 3>, 3>;
using Vector6D = std::array<double, 6>;
using Matrix6x6 = std::array<std::array<double, 6>, 6>;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// utility functions
inline mat3 skew(const vec3& v) {
    return {{
        {  0.0,  -v[2],  v[1] },
        {  v[2],  0.0,  -v[0] },
        { -v[1],  v[0],  0.0  }
    }};
}

inline vec3 unskew(const mat3& m) {
    return { m[2][1], m[0][2], m[1][0] };
}


// vector6D
inline Vector6D operator+(const Vector6D& a, const Vector6D& b) {
	return { a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3], a[4]+b[4], a[5]+b[5] };
}

inline Vector6D operator-(const Vector6D& a, const Vector6D& b) {
    return { a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3], a[4]-b[4], a[5]-b[5] };
}

inline Vector6D scaleVec6(const Vector6D& a, double s) {
    return { a[0]*s, a[1]*s, a[2]*s, a[3]*s, a[4]*s, a[5]*s };
}

inline double dotVec6(const Vector6D& a, const Vector6D& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4] + a[5]*b[5];
}

inline double normVec6(const Vector6D& v) {
    return std::sqrt(dotVec6(v, v));
}
