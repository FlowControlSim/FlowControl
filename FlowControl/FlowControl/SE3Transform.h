#ifndef SE3TRANSFORM_H
#define SE3TRANSFORM_H

#include "helpers.h"


class SE3Transform
{
    /*
        Represents elements of SE(3) Lie group.
        4×4 homogeneous transformation matrix :
        [[R₃ₓ₃, t₃ₓ₁],
        [0₁ₓ₃, 1]]
        where R ∈ SO(3) is rotation, t ∈ ℝ³ is translation
    */
public:
    Eigen::Matrix4d data;
    SE3Transform(Eigen::Matrix4d data) : data(data) {};
	virtual ~SE3Transform() {};

    Eigen::Matrix3d R();
    Eigen::Vector3d t();

    SE3Transform compose(Eigen::Matrix4d other);
    SE3Transform __matmul__(Eigen::Matrix4d other);
    SE3Transform inverse();
	Eigen::Vector3d apply_to_vector(Eigen::Vector3d v);
	Eigen::Vector4d apply_to_point(Eigen::Vector3d p);

};

SE3Transform identity();
SE3Transform from_rotation_translation(Eigen::Matrix3d R, Eigen::Vector3d t);
SE3Transform from_translation(Eigen::Vector3d t);
SE3Transform from_rotation_z(double theta);


#endif // SE3TRANSFORM_H
