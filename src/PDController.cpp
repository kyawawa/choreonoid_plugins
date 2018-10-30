// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  PDcontroller.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

// #include <cnoid/RootItem>
#include "PDController.h"
#include <iostream>
#include <fstream>

namespace cnoid {

void PDController::PDControl()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        double q = joints_[i]->q(); // input
        double dq = (q - q_prev_[i]) / dt_;
        double dq_ref = (q_ref_[i] - q_ref_prev_[i]) / dt_;
        joints_[i]->u() = p_gain_[i] * (q_ref_[i] - q) + d_gain_[i] * (dq_ref - dq); // output
        joints_[i]->u() = std::max(std::min(joints_[i]->u(), u_limit_[i]), -u_limit_[i]);

        q_prev_[i] = q;
        q_ref_prev_[i] = q_ref_[i];
    }
}

void PDController::parseOptionString(SimpleControllerIO* io)
{
    std::vector<std::string> option_string_vec = io->options();
    for (size_t i = 0; i < option_string_vec.size(); ++i) {
        // Set PD gains
        if (option_string_vec[i] == "pdgains") {
            std::ifstream ifs(option_string_vec[++i]);
            if (ifs.is_open()) {
                double gain;
                for (size_t idx = 0; idx < joints_.size(); ++idx){
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
                for (size_t idx = 0; idx < joints_.size(); ++idx){
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

bool PDController::initialize(SimpleControllerIO* io)
{
    Body::JointAccessor joint_list = io->body()->joints();

    joints_.reserve(io->body()->numJoints());
    q_ref_.reserve(io->body()->numJoints());
    q_prev_.reserve(io->body()->numJoints());
    q_ref_prev_.reserve(io->body()->numJoints());
    p_gain_.reserve(io->body()->numJoints());
    d_gain_.reserve(io->body()->numJoints());
    u_limit_.reserve(io->body()->numJoints());

    for (auto it = joint_list.begin(); it != joint_list.end(); ++it) {
        (*it)->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(*it);
        joints_.push_back(*it);
        q_ref_.push_back((*it)->q());
        q_prev_.push_back((*it)->q());
        q_ref_prev_.push_back((*it)->q());
        p_gain_.push_back(3000.0);
        d_gain_.push_back(200.0);
        u_limit_.push_back(500);
    }

    dt_ = io->timeStep();
    // robot_name_ = io->body()->name();
    parseOptionString(io);

    return true;
}

bool PDController::start()
{
    return true;
}

bool PDController::control()
{
    PDControl();
    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController)
}
