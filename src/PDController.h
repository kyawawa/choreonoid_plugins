// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __PDCONTROLLER_H__
#define __PDCONTROLLER_H__

#include <cnoid/SimpleController>
#include <cnoid/SimulatorItem>

namespace cnoid {

class PDController : public SimpleController
{
public:
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool start() override;
    virtual bool control() override;
private:
    std::vector<Link*> joints_;
    std::vector<double> q_ref_;
    std::vector<double> q_prev_;
    std::vector<double> q_ref_prev_;
    std::vector<double> p_gain_;
    std::vector<double> d_gain_;
    std::vector<double> u_limit_;
    double dt_;
    // std::string robot_name_;
    // SimulatorItemPtr simulator_item_;

    void PDControl();
    void parseOptionString(SimpleControllerIO* io);
    void resetSimulation();
};

}
#endif // __PDCONTROLLER_H__
