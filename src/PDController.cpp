// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  PDcontroller.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include <cnoid/SimpleController>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <fstream>
using namespace cnoid;

class PDController : public SimpleController
{
    std::vector<Link*> joints;
    std::vector<double> q_ref;
    std::vector<double> q_prev;
    std::vector<double> q_ref_prev;
    std::vector<double> p_gain;
    std::vector<double> d_gain;
    std::vector<double> u_limit;
    double dt;

    void PDControl()
    {
        for (size_t i = 0; i < joints.size(); ++i) {
            double q = joints[i]->q(); // input
            double dq = (q - q_prev[i]) / dt;
            double dq_ref = (q_ref[i] - q_ref_prev[i]) / dt;
            joints[i]->u() = p_gain[i] * (q_ref[i] - q) + d_gain[i] * (dq_ref - dq); // output
            joints[i]->u() = std::max(std::min(joints[i]->u(), u_limit[i]), -u_limit[i]);

            q_prev[i] = q;
            q_ref_prev[i] = q_ref[i];
        }
    }

    void parseOptionString(SimpleControllerIO* io)
    {
        std::vector<std::string> option_string_vec = io->options();
        for (size_t i = 0; i < option_string_vec.size(); ++i) {
            // Set PD gains
            if (option_string_vec[i] == "pdgains") {
                std::ifstream ifs(option_string_vec[++i]);
                if (ifs.is_open()) {
                    double gain;
                    for (size_t idx = 0; idx < joints.size(); ++idx){
                        if (ifs >> gain) {
                            p_gain[idx] = gain;
                        } else {
                            std::cerr << "PD gain config file " << option_string_vec[i] << " is too short" << std::endl;
                        }
                        if (ifs >> gain) {
                            d_gain[idx] = gain;
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
                    for (size_t idx = 0; idx < joints.size(); ++idx){
                        if (ifs >> ulimit) {
                            u_limit[idx] = ulimit;
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

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        Body::JointAccessor joint_list = io->body()->joints();

        joints.reserve(io->body()->numJoints());
        q_ref.reserve(io->body()->numJoints());
        q_prev.reserve(io->body()->numJoints());
        q_ref_prev.reserve(io->body()->numJoints());
        p_gain.reserve(io->body()->numJoints());
        d_gain.reserve(io->body()->numJoints());
        u_limit.reserve(io->body()->numJoints());

        for (auto it = joint_list.begin(); it != joint_list.end(); ++it) {
            (*it)->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(*it);
            joints.push_back(*it);
            q_ref.push_back((*it)->q());
            q_prev.push_back((*it)->q());
            q_ref_prev.push_back((*it)->q());
            p_gain.push_back(3000.0);
            d_gain.push_back(200.0);
            u_limit.push_back(500);
        }
        dt = io->timeStep();
        parseOptionString(io);

        return true;
    }

    virtual bool control() override
    {
        PDControl();
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController)
