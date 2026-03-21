#ifndef VECTOR6D_H
#define VECTOR6D_H

#include "helpers.h"
#include "SE3Transform.h"

class Vector6D 
{
public:
    vec6 data;

    Vector6D() : data(vec6::Zero()) {}
    Vector6D(const vec6& d) : data(d) {}

    Vector6D operator+(const Vector6D& rhs) const {
        return Vector6D(data + rhs.data); 
    }

    Vector6D operator-(const Vector6D& rhs) const {
        return Vector6D(data - rhs.data);
    }

    Vector6D operator*(const double scalar) const {
        return Vector6D(data * scalar);
    }

    vec3 omega() const { 
        return vec3(data[0], data[1], data[2]); 
    }
    vec3 vel() const { 
        return vec3(data[3], data[4], data[5]); 
    }

    Vector6D zero() const {
        return Vector6D(vec6::Zero());
	}

    SE3Transform to_se3_transform() const {
        mat4 M = mat4::Zero();
        M.topLeftCorner<3, 3>() = skew(omega());
        M.topRightCorner<3, 1>() = vel();
        return SE3Transform(M);
	}

};

#endif // VECTOR6D_H