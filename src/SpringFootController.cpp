// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  SpringFootController.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include <cnoid/SimpleController>
#include <iostream>
#include <fstream>

namespace cnoid {

class SpringFootController : public SimpleController
{
public:
    bool initialize(SimpleControllerIO* io) override
    {
        Body::JointAccessor joint_list = io->body()->allJoints();

        size_t idx = 0;
        for (auto it = joint_list.begin(); it != joint_list.end(); ++it) {
            if ((*it)->name().find("SPRING") != std::string::npos) {
                (*it)->setActuationMode(Link::JOINT_TORQUE);
                io->enableIO(*it);
                joints_idx_.push_back(idx);
                joints_.push_back(*it);
                q_prev_.push_back((*it)->q());
                kp_.push_back(10000);
                kd_.push_back(1000);
                ++idx;
            }
        }

        dt_ = io->timeStep();
        parseOptionString(io);

        return true;
    }

    bool control() override
    {
        for (size_t idx : joints_idx_) {
            double q = joints_[idx]->q(); // input
            double dq = (q - q_prev_[idx]) / dt_;
            joints_[idx]->u() = -kp_[idx] * q - kd_[idx] * dq;
            q_prev_[idx] = q;
        }

        return true;
    }

private:
    std::vector<size_t> joints_idx_;
    std::vector<Link*> joints_;
    std::vector<double> q_prev_;
    std::vector<double> kp_;
    std::vector<double> kd_;
    double dt_;

    void parseOptionString(SimpleControllerIO* io)
    {
        std::vector<std::string> option_string_vec = io->options();
        for (size_t i = 0; i + 1 < option_string_vec.size(); ++i) {
            // Set spring constant
            if (option_string_vec[i] == "spring") {
                std::ifstream ifs(option_string_vec[++i]);
                if (ifs.is_open()) {
                    double spring;
                    for (size_t idx = 0; idx < joints_.size(); ++idx){
                        if (ifs >> spring) {
                            kp_[idx] = spring;
                        } else {
                            std::cerr << "Spring constant config file " << option_string_vec[i] << " is too short" << std::endl;
                        }
                        if (ifs >> spring) {
                            kd_[idx] = spring;
                        } else {
                            std::cerr << "Spring constant config file " << option_string_vec[i] << " is too short" << std::endl;
                        }
                    }
                    std::cerr << "Spring constant config file " << option_string_vec[i] << " opened" << std::endl;
                } else {
                    std::cerr << "Spring constant config file " << option_string_vec[i] << " can't be opened!" << std::endl;
                }
            }
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpringFootController)
}
