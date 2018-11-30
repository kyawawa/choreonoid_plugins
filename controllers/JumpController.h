// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __JUMPCONTROLLER_H__
#define __JUMPCONTROLLER_H__

#include "ControllerBase.h"
#include "../util/Interpolator.h"
#include "../util/Kinematics.h"

namespace cnoid {

class JumpController : public ControllerBase
{
public:
    virtual bool initialize(SimpleControllerIO* io) override;
    // virtual bool start() override;
    virtual bool control() override;
    JumpController() : control_count(0), is_jumping(false), ref_jumping_time(0)
    {
    };
private:
    unsigned int control_count;
    bool is_jumping;
    double ref_jumping_time;

    BodyPtr ioBody;
    enum END_EFFECTOR { RLEG, LLEG }; // Dirty
    std::vector<JointPathPtr> end_effectors; // Dirty
    std::vector<IKParam> ik_params;


    // Vector6 calcJumpCOMTrajectory(const double time);
    // std::function<Vector6(const double time)> calcRefCOMVel; // Return COM velocity and angular velocity
    bool startJump(const double height, const double squat = 0.0);
    void jumpControl(minJerkCoeff& coeff);
};

}


#endif // __JUMPCONTROLLER_H__
