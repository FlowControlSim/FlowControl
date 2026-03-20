#ifndef SE3TRANSFORM_H
#define SE3TRANSFORM_H

#include "Vector6D.h"

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
	SE3Transform();
	virtual ~SE3Transform();

    Vector6D R();
    void t();

};

#endif // SE3TRANSFORM_H
