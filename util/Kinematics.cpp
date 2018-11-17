// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  Kinematics.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include "Kinematics.h"

bool solveWeightedWholebodyIK(cnoid::Position* position,
                              std::vector<cnoid::LinkPtr>& joints,
                              const std::vector<IKParam>& ik_params,
                              const double ik_threshold,
                              const size_t max_iteration,
                              const double damping)
{
    // TODO: Ordered IK
    // cnoid::VectorX lambda;

    // Set the rank of configuration space
    size_t rank_c_space = joints.size();
    if (position) rank_c_space += 6; // Whole body IK

    size_t task_dof = 0;
    for (IKParam ik_param : ik_params) {
        if (ik_param.target_type == ik_3daffine) task_dof += 6;
        else task_dof += 3;
    }

    Eigen::VectorXd IK_weight(task_dof);
    Eigen::VectorXd error = Eigen::VectorXd::Zero(task_dof);
    Eigen::VectorXd damping_vec = damping * Eigen::VectorXd::Ones(rank_c_space);
    Eigen::MatrixXd jacobian(task_dof, rank_c_space);

    // Dirty code
    for (IKParam ik_param : ik_params) {
        size_t dof_accum = 0;
        size_t dof = 3;
        if (ik_param.target_type == ik_3daffine) dof = 6;
        IK_weight.segment(dof_accum, dof) = ik_param.IK_weight;
        dof_accum += dof;
    }

    for (size_t _ = 0; _ < max_iteration; ++_) {
        size_t dof_accum = 0;
        for (IKParam ik_param : ik_params) {
            size_t dof = 3;
            if (ik_param.target_type == ik_3daffine) dof = 6;

            jacobian.block(dof_accum, 0, dof, rank_c_space) = ik_param.calcJacobian();
            // ik_param.calcJacobian(jacobian.block(dof_accum, 0, 3, rank_c_space));
            error.segment(dof_accum, dof) = ik_param.calcError();

            dof_accum += dof;
        }

        Eigen::VectorXd q_delta = (jacobian.transpose() * IK_weight.asDiagonal() * jacobian + damping_vec.diagonal()).inverse() *
            (jacobian.transpose() * IK_weight.asDiagonal() * error);

        size_t idx_c_space = 0;
        if (position) {
            position->translate(Eigen::Vector3d(q_delta[0], q_delta[1], q_delta[2]));
            Eigen::Vector3d omega_delta(q_delta[3], q_delta[4], q_delta[5]);
            position->rotate(Eigen::AngleAxisd(omega_delta.norm(), omega_delta.normalized()));
            idx_c_space += 6;
        }
        for (size_t i = 0; i < joints.size(); ++i) joints[i]->q() += q_delta[i];

        calcFowardKinematics();
        calcCenterOfMass();

        if(q_delta.squaredNorm() < ik_threshold) return true; // Completed
    }

    return false; // Not completed
}
