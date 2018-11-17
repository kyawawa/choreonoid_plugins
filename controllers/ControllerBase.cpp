// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  ControllerBase.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include "ControllerBase.h"
#include <iostream>
#include <fstream>

namespace cnoid {

void ControllerBase::PDControl()
{
    for (size_t idx : actuation_joints_idx_) {
        double q = joints_[idx]->q(); // input
        double dq = (q - q_prev_[idx]) / dt_;
        double dq_ref = (q_ref_[idx] - q_ref_prev_[idx]) / dt_;
        joints_[idx]->u() = p_gain_[idx] * (q_ref_[idx] - q) + d_gain_[idx] * (dq_ref - dq); // output
        joints_[idx]->u() = std::max(std::min(joints_[idx]->u(), u_limit_[idx]), -u_limit_[idx]);

        q_prev_[idx] = q;
        q_ref_prev_[idx] = q_ref_[idx];
    }
}

void ControllerBase::parseOptionString(SimpleControllerIO* io)
{
    std::vector<std::string> option_string_vec = io->options();
    for (size_t i = 0; i < option_string_vec.size(); ++i) {
        // Set PD gains
        if (option_string_vec[i] == "pdgains") {
            std::ifstream ifs(option_string_vec[++i]);
            if (ifs.is_open()) {
                double gain;
                for (size_t idx : actuation_joints_idx_) {
                    if (ifs >> gain) {
                        p_gain_[idx] = gain;
                    } else {
                        std::cerr << "PD gain config file " << option_string_vec[i] << " is too short" << std::endl;
                    }
                    if (ifs >> gain) {
                        d_gain_[idx] = gain;
                    } else {
                        std::cerr << "PD gain config file " << option_string_vec[i] << " is too short" << std::endl;
                    }
                }
                std::cerr << "PD gain config file " << option_string_vec[i] << " opened" << std::endl;
            } else {
                std::cerr << "PD gain config file " << option_string_vec[i] << " can't be opened!" << std::endl;
            }
        }
        // Set Torque Limits
        else if (option_string_vec[i] == "TorqueLimits") {
            std::ifstream ifs(option_string_vec[++i]);
            if (ifs.is_open()) {
                double ulimit;
                for (size_t idx : actuation_joints_idx_) {
                    if (ifs >> ulimit) {
                        u_limit_[idx] = ulimit;
                    } else {
                        std::cerr << "Torque limit config file " << option_string_vec[i] << " is too short" << std::endl;
                    }
                }
                std::cerr << "Torque limit config file " << option_string_vec[i] << " opened" << std::endl;
            } else {
                std::cerr << "Torque limit config file " << option_string_vec[i] << " can't be opened!" << std::endl;
            }
        }
    }
}

bool ControllerBase::initialize(SimpleControllerIO* io)
{
    Body::JointAccessor joint_list = io->body()->joints();

    actuation_joints_idx_.reserve(io->body()->numJoints());
    joints_.reserve(io->body()->numJoints());
    q_ref_.reserve(io->body()->numJoints());
    q_prev_.reserve(io->body()->numJoints());
    q_ref_prev_.reserve(io->body()->numJoints());
    p_gain_.reserve(io->body()->numJoints());
    d_gain_.reserve(io->body()->numJoints());
    u_limit_.reserve(io->body()->numJoints());

    size_t joint_idx = 0;
    for (auto it = joint_list.begin(); it != joint_list.end(); ++it, ++joint_idx) {
        (*it)->setActuationMode(Link::JOINT_TORQUE);
        joints_.push_back(*it);
        q_ref_.push_back((*it)->q());
        q_prev_.push_back((*it)->q());
        q_ref_prev_.push_back((*it)->q());
        p_gain_.push_back(3000.0);
        d_gain_.push_back(200.0);
        u_limit_.push_back(500);

        // Ignore spring joints
        if ((*it)->name().find("SPRING") == std::string::npos) {
            io->enableIO(*it);
            actuation_joints_idx_.push_back(joint_idx);
        }
    }

    dt_ = io->timeStep();
    parseOptionString(io);

    return true;
}

bool ControllerBase::start()
{
    return true;
}

bool ControllerBase::control()
{
    PDControl();
    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ControllerBase)
}
