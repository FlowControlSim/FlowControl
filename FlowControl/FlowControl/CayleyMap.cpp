#include "CayleyMap.h"

float CayleyMap::sigma(Vector3d Y) {
    float res = Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2];
    return 2.0 / (1.0 + res);
}

SE3Transform CayleyMap::cayley_map(Vector6D Y) {
    Vector3d x = {Y[0], Y[1], Y[2]};
	Vector3d y = { Y[3], Y[4], Y[5] };

    Matrix3d x_skew = skew(x);
    float sig = CayleyMap::sigma(x);

    Matrix3d I = Matrix3d::Identity();

    // Eq. 3.7
    Matrix3d R = I + sig * (x_skew + x_skew * x_skew);

    // Eq. 3.14
    Vector3d t = (I + R) * y;

    return from_rotation_translation(R, t);
}

Vector6d CayleyMap::inverse_cayley_map(SE3Transform T) {
    
    Matrix3d R = T.R();
	Vector3d t = T.t();
    Matrix3d I = Matrix3d::Identity();

    Matrix3d x_skew = 2 * (R - I) * (R + I).inverse();
    Vector3d x = unskew(x_skew);

    // y = (I + R)⁻¹ t
    Vector3d y = (I + R).inverse() * t;

    Vector6d out = Vector6d::Zero();
    out << x, y;
    return out;
}

Matrix6d CayleyMap::cayley_differential(Vector6d Y) {
    /*
    cayley differential - Müller Lemma 3.2, Eq. 3.16
    */
    Vector3d x = { Y[0], Y[1], Y[2] };
    Vector3d y = { Y[3], Y[4], Y[5] };

    float sig = CayleyMap::sigma(x);
    Matrix3d x_skew = skew(x);
    Matrix3d y_skew = skew(y);
    Matrix3d I3 = Matrix3d::Identity();

    Matrix3d p00 = sig * (I3 + x_skew);
    Matrix3d p01 = Matrix3d::Zero();
    Matrix3d p10 = sig * y_skew * (I3 + x_skew);
    Matrix3d p11 = 2 * I3 + sig * (x_skew + x_skew * x_skew);

    Matrix6d out = Matrix6d::Zero();
    out << p00, p01,
		p10, p11;
    return out;

}

Matrix6d CayleyMap::inverse_cayley_differential(Vector6d Y) {
    Vector3d x = { Y[0], Y[1], Y[2] };
    Vector3d y = { Y[3], Y[4], Y[5] };

    float sig = CayleyMap::sigma(x);
    Matrix3d x_skew = skew(x);
    Matrix3d y_skew = skew(y);
    Matrix3d I3 = Matrix3d::Identity();

    // Direct formula, no matrix inversion
    Matrix3d p00 = (2.0 / sig) * I3 + x_skew * x_skew - x_skew;
    Matrix3d p01 = Matrix3d::Zero();
    Matrix3d p10 = -y_skew * (I3 - x_skew);
    Matrix3d p11 = I3 - x_skew;

    return Matrix6d(0.5 * (Matrix6d() << p00, p01, p10, p11).finished());
}
