#include "FlowIntegrator.h"

// external force class

Vector6D ExternalForceComputer::gravity_buoyancy(double mass_body, double volume, double rho_fluid, const Vector3d& g_vec) {
    double mass_displaced = volume * rho_fluid;
    Vector3d F = (mass_body - mass_displaced) * g_vec;
    Vector6d data;
    data << Vector3d::Zero(), F;
    return Vector6D(data);
}

Vector6D ExternalForceComputer::drag_simple(const Vector6D& velocity, double rho_fluid, double ref_area, double C_d) {
    Vector3d v = velocity.vel();
    double v_norm = v.norm();

    if (v_norm < 1e-10) {
        return Vector6D();
    }

    double F_mag = 0.5 * rho_fluid * ref_area * C_d * v_norm * v_norm;
    Vector3d F = -F_mag * (v / v_norm);
    Vector6d data;
    data << Vector3d::Zero(), F;
    return Vector6D(data);
}


// flow integrator class

Matrix6d FlowIntegrator::compute_body_inertia(const std::vector<Vector3d>& vertices, const std::vector<double>& mass_density) {
    Matrix3d I_ww = Matrix3d::Zero();
    Matrix3d I_wv = Matrix3d::Zero();
    Matrix3d I_vv = Matrix3d::Zero();

    for (size_t i = 0; i < vertices.size(); ++i) {
        Matrix3d gammaX = skew(vertices[i]);
        double rho_i  = mass_density[i];
        I_ww += gammaX.transpose() * gammaX * rho_i;
        I_wv += gammaX * rho_i;
        I_vv += Matrix3d::Identity() * rho_i;
    }

    Matrix6d K;
    K << I_ww, I_wv, I_wv.transpose(), I_vv;
    return K;
}

Vector6D FlowIntegrator::compute_body_momentum(const std::vector<Vector3d>& vertices_k,
                                               const std::vector<Vector3d>& vertices_k1,
                                               const std::vector<double>& mass_density, double dt) {
    Vector3d l_b = Vector3d::Zero();
    Vector3d p_b = Vector3d::Zero();

    for (size_t i = 0; i < vertices_k.size(); ++i) {
        Vector3d gamma_dot = (vertices_k1[i] - vertices_k[i]) / dt;
        double rho_i = mass_density[i];
        l_b += vertices_k[i].cross(gamma_dot) * rho_i;
        p_b += gamma_dot * rho_i;
    }

    Matrix6d data;
    data << l_b, p_b;
    return Vector6D(data);
}

Vector6D FlowIntegrator::compute_total_force(const Vector6D& velocity, double mass_body, double volume,
                                             double rho_fluid, double ref_area, double C_d, bool include_drag) {
    Vector6D F = ExternalForceComputer::gravity_buoyancy(mass_body, volume, rho_fluid);

    if (include_drag) {
        F = F + ExternalForceComputer::drag_simple(velocity, rho_fluid, ref_area, C_d);
    }

    return F;
}



