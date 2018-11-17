// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  JumpController.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include "JumpController.h"
#include <iostream>
#include <functional>
#include <algorithm>
#include <iterator>

namespace {
const double G_ACC = -9.80665;
}

namespace cnoid {

bool JumpController::startJump(const double height, const double squat)
{
    control_count = 0;
    is_jumping = true;

    Vector3 startPos = ioBody->calcCenterOfMass();
    Vector3 start(startPos[2], 0, 0); // pos, vel, acc
    Vector3 finish(startPos[2] + 0.15, sqrt(2 * G_ACC * height), G_ACC);
    minJerkCoeffTime coeff_time = calcMinJerkCoeffTime(start, finish);
    minJerkCoeff coeff;
    std::copy(coeff_time.begin(), coeff_time.end() - 1, coeff.begin());

    ik_params.resize(4);
    ik_params[0].target_type = ik_trans; // COM Position
    ik_params[1].target_type = ik_axisrot; // Angular Vel
    ik_params[2].target_type = ik_3daffine; // RLEG
    ik_params[3].target_type = ik_3daffine; // LLEG

    ik_params[0].calcJacobian = []() { calcCMJacobian(ioBody, nullptr,)

    // auto calcCOM = std::bind(calcNthOrderSpline, coeff, std::placeholders::_1);
    // calc = calcNthOrderSpline(coeff); // curry

    // setCOMCalcRule(std::bind(jumpControl, coeff), T);
    return true;
}

bool JumpController::initialize(SimpleControllerIO* io)
{
    ControllerBase::initialize(io);
    ioBody = io->body();

    return true;
}

// bool JumpController::start()
// {
//     return true;
// }

void JumpController::jumpControl(minJerkCoeff& coeff)
{
    ++control_count;
    Vector3 refCOM = ioBody->centerOfMass();
    Matrix3 refCOMAng = Vector3::Zero();
    refCOM[2] = calcNthOrderSpline<5>(coeff, control_count * dt_);


}

bool JumpController::control()
{
    if (is_jumping) {
        // jumpControl();
    }
    PDControl();
    return true;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JumpController)
}
