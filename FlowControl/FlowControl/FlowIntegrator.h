#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Vector6D.h"
#include "SE3Transform.h"
#include "CayleyMap.h"
#include "helpers.h"

struct NewtonResult {
    SE3Transform g_next;
    Vector6D mu_next;
    Vector6D Y;
    bool converged;
    int iterations;
	double residual;
};

struct SimulationResult {
    std::vector<Vector3d> trajectory;
    std::vector<Vector3d> velocities;
    std::vector<Vector3d> forces;
    std::vector<NewtonResult> convergence;
    double avg_newton_iters;
};

class ExternalForceComputer {
public:
    static Vector6D gravity_buoyancy(double mass_body, double volume, double rho_fluid,
                                     const Vector3d& g_vec = Vector3d(0, -9.81, 0));

    static Vector6D drag_simple(const Vector6D& velocity, double rho_fluid, double ref_area, double C_d = 0.5);
};



class FlowIntegrator {
public:

    FlowIntegrator(int max_newton_iters = 20, double newton_tol = 1e-8)
        : max_newton_iters(max_newton_iters), newton_tol(newton_tol), stats(std::pair<int, int>(0, 0)) {}

    Matrix6d compute_body_inertia(const std::vector<Vector3d>& vertices, const std::vector<double>& mass_density);

    Vector6D compute_body_momentum(const std::vector<Vector3d>& vertices_k,
                                   const std::vector<Vector3d>& vertices_k1,
                                   const std::vector<double>& mass_density,
                                   double dt);

    Vector6D compute_total_force(const Vector6D& velocity, double mass_body, double volume, double rho_fluid,
                                 double ref_area, double C_d = 0.5, bool include_drag = true);

    NewtonResult integrate_step_newton(const SE3Transform& g_k, const Vector6D& mu_k, const Matrix6d& K,
                                       const Vector6D& mu_offset, const Vector6D& F, double dt);

    SimulationResult simulate(const SE3Transform& g0, const Vector6D& mu0, const Matrix6d& K, const Vector6D& mu_offset, 
                              double dt, int num_steps, double mass_body, double volume, double rho_fluid, 
                              double ref_area, double C_d = 0.5, bool include_drag = true, bool verbose = false);

private:
    int max_newton_iters;
    double newton_tol;
	std::pair<int, int> stats;  // (total_iters, total_steps)

};