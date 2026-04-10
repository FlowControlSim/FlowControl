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


static Vector6D lift(const Vector6D& velocity, const Vector3d& leaf_normal_world,
    double rho_fluid, double ref_area, double C_l) {
    Vector3d v = velocity.vel();
    double v_norm = v.norm();
    if (v_norm < 1e-10) return Vector6D();

    Vector3d v_hat = v / v_norm;
    // Lift acts perpendicular to velocity, in the plane of v and the leaf normal
    Vector3d lift_dir = v_hat.cross(leaf_normal_world).cross(v_hat);
    double lift_mag_norm = lift_dir.norm();
    if (lift_mag_norm < 1e-10) return Vector6D();
    lift_dir /= lift_mag_norm;

    double F_mag = 0.5 * rho_fluid * ref_area * C_l * v_norm * v_norm;
    Vector6d data;
    data << Vector3d::Zero(), F_mag* lift_dir;
    return Vector6D(data);
}

static Vector6D angular_drag(const Vector6D& velocity, double k_ang) {
    // Damps spinning so leaf tumbles rather than spinning forever
    Vector6d data;
    data << -k_ang * velocity.omega(), Vector3d::Zero();
    return Vector6D(data);
}


// flow integrator class

float FlowIntegrator::compute_delta(std::vector<double> face_areas, std::vector<MVector> face_normals, const MeshData& mesh) {
    double numer_sum = std::accumulate(face_areas.begin(), face_areas.end(), 0.0);
    double denom_sum = 0.0;

    for (const Edge& e : mesh.m_edges) {
        if (e.f1 == -1) {
            // edge has only one face so we skip it
            continue;
        }
        double alpha = mesh.getEdgeDihedralAngle(e);
        denom_sum += alpha * e.length;
    }

    if (std::abs(denom_sum) < 1e-8) {
        return static_cast<float>(std::sqrt(numer_sum));
    }

    double delta = numer_sum / denom_sum;
    return delta;
}

Matrix6d FlowIntegrator::compute_body_inertia(const std::vector<Vector3d>& vertices, const std::vector<double>& mass_density) {
    // compute center of mass
    Vector3d com = Vector3d::Zero();
    double total_mass = 0.0;

    for (size_t i = 0; i < vertices.size(); ++i) {
        com += vertices[i] * mass_density[i];
        total_mass += mass_density[i];
    }
    com /= total_mass;

    Matrix3d I_ww = Matrix3d::Zero();
    Matrix3d I_wv = Matrix3d::Zero();
    Matrix3d I_vv = Matrix3d::Zero();

    for (size_t i = 0; i < vertices.size(); ++i) {
		Vector3d r_i = vertices[i] - com;
        Matrix3d gammaX = skew(r_i);
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

    //Matrix6d data;
    //data << l_b, p_b;

    Vector6d data;
    data << l_b, p_b;


    return Vector6D(data);
}

Matrix6d FlowIntegrator::compute_added_mass_tensor(const MeshData& mesh, double rho_fluid) {
    // compute face areas
    std::vector<double> face_areas = mesh.m_faceAreas;

    // compute face normals
    std::vector<MVector> face_normals;
    int normals_num = mesh.m_faceNormals.length();
    face_normals.reserve(normals_num);
    for (unsigned int i = 0; i < normals_num; ++i) {
        face_normals.push_back(mesh.m_faceNormals[i]);
    }

    // compute delta
	double delta = compute_delta(face_areas, face_normals, mesh);

    // compute added mass tensor
    Matrix6d I = Matrix6d::Zero();

    for (int i = 0; i < mesh.numTriangles(); ++i) {
        MPoint x_maya = mesh.m_faceCenters[i];
        Vector3d x(x_maya.x, x_maya.y, x_maya.z); // face center
        MVector n_maya = mesh.m_faceNormals[i];
        Vector3d n(n_maya.x, n_maya.y, n_maya.z); // face normal
        double A = face_areas[i]; // face area

        Vector3d xn = x.cross(n);

        Matrix3d A11 = xn * xn.transpose();
        Matrix3d A12 = xn * n.transpose();
        Matrix3d A21 = A12.transpose();
        Matrix3d A22 = n * n.transpose();

        Matrix6d J = Matrix6d::Zero();
        J << A11, A12,
             A21, A22;
        I += rho_fluid * delta * A * J;
    }

    return I;
}

Vector6D FlowIntegrator::compute_fluid_momentum(const std::vector<Vector3d>& vertices_k,
                                                const std::vector<Vector3d>& vertices_k1,
                                                const MeshData& mesh, double rho_fluid, double dt) {
    // compute face areas
    std::vector<double> face_areas = mesh.m_faceAreas;

    // compute face normals
    std::vector<MVector> face_normals;
    int normals_num = mesh.m_faceNormals.length();
    face_normals.reserve(normals_num);
    for (unsigned int i = 0; i < normals_num; ++i) {
        face_normals.push_back(mesh.m_faceNormals[i]);
    }

    // compute delta
    const double delta = compute_delta(face_areas, face_normals, mesh);

	// compute fluid momentum
    const double prefactor = rho_fluid * delta;

    Vector3d l_f = Vector3d::Zero();
    Vector3d p_f = Vector3d::Zero();

    for (int i = 0; i < mesh.numTriangles(); ++i) {
        // triangle vertices
        const int i0 = mesh.m_triangleFaces[3 * i];
        const int i1 = mesh.m_triangleFaces[3 * i + 1];
        const int i2 = mesh.m_triangleFaces[3 * i + 2];

        const Vector3d gamma_dot_face = ((vertices_k1[i0] - vertices_k[i0]) +
                                         (vertices_k1[i1] - vertices_k[i1]) +
                                         (vertices_k1[i2] - vertices_k[i2])) / (3.0 * dt);
        const Vector3d gamma_face = (vertices_k[i0] + vertices_k[i1] + vertices_k[i2]) / 3.0;

        const double A = face_areas[i];
        const MVector& n_maya = mesh.m_faceNormals[i];
        const Vector3d n(n_maya.x, n_maya.y, n_maya.z);

		// update fluid momentum
        const double gdot_n = gamma_dot_face.dot(n);
        l_f += prefactor * gdot_n * gamma_face.cross(n) * A;
        p_f += prefactor * gdot_n * n * A;
    }

    Vector6d data;
    data << l_f, p_f;

    return Vector6D(data);
}

Vector6D FlowIntegrator::compute_total_force(const Vector6D& velocity, double mass_body, double volume,
                                                                    double rho_fluid, double ref_area,
                                                                    double C_d, double C_l, double k_ang,
                                                                    const Vector3d& leaf_normal_world,
                                                                    bool include_drag) {
    Vector6D F = ExternalForceComputer::gravity_buoyancy(mass_body, volume, rho_fluid);

    if (include_drag) {
        F = F + ExternalForceComputer::drag_simple(velocity, rho_fluid, ref_area, C_d);
        F = F + lift(velocity, leaf_normal_world, rho_fluid, ref_area, C_l);
        F = F + angular_drag(velocity, k_ang);
    }

    return F;
}


NewtonResult FlowIntegrator::integrate_step_newton(const SE3Transform& g_k, const Vector6D& mu_k, const Matrix6d& K,
                                                   const Vector6D& mu_offset, const Vector6D& F, double dt) {
    const double h = dt;
    const Matrix6d K_inv = K.inverse();

    Vector6D Y_k(K_inv * (mu_k.data - mu_offset.data));
    Vector6D Y = Y_k;

    CayleyMap cayley;
    Matrix6d dcay_inv_neg_hYk = cayley.inverse_cayley_differential(Y_k * (-h));

    double r_norm = 0.0;
    int iter = 0;

    for (iter = 0; iter < max_newton_iters; ++iter) {
        Matrix6d dcay_inv_hY = cayley.inverse_cayley_differential(Y * h);
        Vector6d momentum = K * Y.data + mu_offset.data;

        Vector6d r = dcay_inv_hY.transpose() * momentum - dcay_inv_neg_hYk.transpose() * mu_k.data - h * F.data;
        r_norm = r.norm();

        if (r_norm < newton_tol) {
            stats.first += (iter + 1);
            stats.second += 1;

            SE3Transform g_next = g_k.compose(cayley.cayley_map(Y * h).data);
            vec6 mu_next_data = K * Y.data + mu_offset.data;
            Vector6D mu_next(mu_next_data);

            return NewtonResult{ g_next, mu_next, Y, true, iter + 1, r_norm };
        }

        const double eps = 1e-6;
        Matrix6d J = Matrix6d::Zero();

        for (int i = 0; i < 6; ++i) {
            Vector6d Y_pert_data = Y.data;
            Y_pert_data(i) += eps;
            Vector6D Y_pert(Y_pert_data);
            Matrix6d dcay_pert = cayley.inverse_cayley_differential(Y_pert * h);
            Vector6d r_pert = dcay_pert.transpose() * (K * Y_pert.data + mu_offset.data) 
                                - dcay_inv_neg_hYk.transpose() * mu_k.data - h * F.data;

            J.col(i) = (r_pert - r) / eps;
        }

        Y = Vector6D(Y.data - J.colPivHouseholderQr().solve(r));
    }

    //std::cerr << "⚠ Newton did not converge (residual: " << r_norm << ")\n";
    std::cerr << "Newton did not converge in (residual: " << r_norm << ")\n";
    stats.first  += max_newton_iters;
    stats.second += 1;

    SE3Transform g_next = g_k.compose(cayley.cayley_map(Y * h).data);
    Vector6D mu_next(K * Y.data + mu_offset.data);

    return NewtonResult{ g_next, mu_next, Y, false, max_newton_iters, r_norm };
}

SimulationResult FlowIntegrator::simulate(const SE3Transform& g0, const Vector6D& mu0, const Matrix6d& K, const Vector6D& mu_offset, 
                                          double dt, int num_steps, double mass_body, double volume, double rho_fluid, 
                                          double ref_area, double C_d, bool include_drag, bool verbose) {
	SimulationResult result;
    
    SE3Transform g = g0;
    Vector6D mu = mu0;

    result.trajectory.reserve(num_steps + 1);
    result.velocities.reserve(num_steps);
    result.forces.reserve(num_steps);
    result.convergence.reserve(num_steps);

    result.trajectory.push_back(g.t());

    for (int i = 0; i < num_steps; ++i) {

        Vector6D Y = Vector6D(K.inverse() * (mu.data - mu_offset.data));
        Vector6D F = compute_total_force(Y, mass_body, volume, rho_fluid, ref_area, C_d, include_drag);
		NewtonResult nR = integrate_step_newton(g, mu, K, mu_offset, F, dt);

        result.trajectory.push_back(nR.g_next.t());
        result.velocities.push_back(nR.mu_next.vel());
        result.forces.push_back(nR.Y.vel());
        result.convergence.push_back(nR);

    }
    double avg_iters = stats.first / stats.second;

    result.avg_newton_iters = avg_iters;

    return result;

}
