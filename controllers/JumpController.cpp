// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  JumpController.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include <cnoid/EigenUtil>
#include "JumpController.h"
#include <iostream>
#include <functional>
#include <algorithm>
#include <iterator>

namespace {
const double G_ACC = -9.80665;
}

namespace cnoid {

bool JumpController::startJump(const double height, const double const double squat)
{
    control_count = 0;
    is_jumping = true;

    Vector3 startPos = ioBody->calcCenterOfMass();
    Vector3 start(startPos[2], 0, 0); // pos, vel, acc
    Vector3 finish(startPos[2] + 0.15, sqrt(2 * G_ACC * height), G_ACC);
    minJerkCoeffTime coeff_time = calcMinJerkCoeffTime(start, finish);
    minJerkCoeff coeff;
    std::copy(coeff_time.begin(), coeff_time.end() - 1, coeff.begin()); // Remove time from coeff_time

    ik_params.resize(4);
    ik_params[0].target_type = ik_trans; // COM Position
    ik_params[1].target_type = ik_axisrot; // Angular Vel
    ik_params[2].target_type = ik_3daffine; // RLEG
    ik_params[3].target_type = ik_3daffine; // LLEG

    ik_params[0].calcJacobian = [&]()
        {
            MatrixXd J;
            calcCMJacobian(ioBody, nullptr, J);
            return J;
        };
    ik_params[0].calcError = [&]()
        {
            return ik_params[0].target_trans - ioBody->centerOfMass();
        };

    ik_params[1].calcJacobian = [&]()
        {
            MatrixXd H;
            calcAngularMomentumJacobian(ioBody, nullptr, H);
            return H;
        };
    ik_params[1].calcError = [&]()
        {
            Vector3 tmp_P, L;
            ioBody->calcTotalMomentum(tmp_P, L);
            return ik_params[1].target_axisrot - L;
        };

    ik_params[2].calcJacobian = [&]()
        {
            MatrixXd J;
            setJacobian<0x3f, 0, 0>(*end_effectors[RLEG], end_effectors[RLEG]->endLink(), J);
            return J;
        };
    ik_params[2].calcError = [&]()
        {
            Vector6 error;
            error.head<3>() = end_effectors[RLEG]->endLink()->p() - ik_params[2].target_pos.translation();
            error.segment<3>(3) = end_effectors[RLEG]->endLink()->R() * omegaFromRot(end_effectors[RLEG]->endLink()->R().transpose() * ik_params[2].target_pos.linear());
            return error;
        };

    ik_params[3].calcJacobian = [&]()
        {
            MatrixXd J;
            setJacobian<0x3f, 0, 0>(*end_effectors[LLEG], end_effectors[LLEG]->endLink(), J);
            return J;
        };
    ik_params[3].calcError = [&]()
        {
            Vector6 error;
            error.head<3>() = end_effectors[LLEG]->endLink()->p() - ik_params[3].target_pos.translation();
            error.segment<3>(3) = end_effectors[LLEG]->endLink()->R() * omegaFromRot(end_effectors[LLEG]->endLink()->R().transpose() * ik_params[3].target_pos.linear());
            return error;
        };

    // auto calcCOM = std::bind(calcNthOrderSpline, coeff, std::placeholders::_1);
    // calc = calcNthOrderSpline(coeff); // curry

    // setCOMCalcRule(std::bind(jumpControl, coeff), T);
    return true;
}

void JumpController::jumpControl(minJerkCoeff& coeff)
{
    ++control_count;
    Vector3 refCOM = ioBody->centerOfMass();
    Vector3 refCOMAng = Vector3::Zero();
    refCOM[2] = calcNthOrderSpline<5>(coeff, control_count * dt_);

}

bool JumpController::initialize(SimpleControllerIO* io)
{
    ControllerBase::initialize(io);
    ioBody = io->body();
    end_effectors.push_back(std::make_shared<JointPath>(io->body()->rootLink(), io->body()->link("RLEG_JOINT5")));
    end_effectors.push_back(std::make_shared<JointPath>(io->body()->rootLink(), io->body()->link("LLEG_JOINT5")));

    return true;
}

// bool JumpController::start()
// {
//     return true;
// }

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
