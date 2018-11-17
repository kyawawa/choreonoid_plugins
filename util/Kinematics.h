// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

// #include <cnoid/EigenTypes>
#include <cnoid/Jacobian>
#include <functional>

enum IKTargetType {
    ik_3daffine, // 4x4 matrix
    ik_trans,    // 3d vector
    ik_axisrot,  // 3d vector
};

struct IKParam {
    IKTargetType target_type;
    // I wanna use union but...
    cnoid::Position target_pos;
    Eigen::Vector3d target_trans;
    Eigen::Vector3d target_axisrot;

    std::function<Eigen::VectorXd()> calcError;
    // https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
    // std::function<void(Eigen::MatrixXd&)> calcJacobian;
    std::function<Eigen::MatrixXd()> calcJacobian;
    // Eigen::DiagonalMatrix<double, 3> IK_weight;
    Eigen::VectorXd IK_weight;
};

bool solveWeightedWholebodyIK(cnoid::Position* position, // Floating base
                              std::vector<cnoid::LinkPtr>& joints, // Should use reference_wrapper and double?
                              const std::vector<IKParam>& ik_params,
                              const double ik_threshold = 1e-2,
                              const size_t max_iteration = 100,
                              const double damping = 1e-1);

#endif // __KINEMATICS_H__
