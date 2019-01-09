// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  PDcontroller_InvertedPendulum.cpp
 * @brief
 * @author Hiroki Takeda
 */

#include <cnoid/SimpleController>
#include <cnoid/AccelerationSensor>
#include <iostream>
#include <cmath>
#include <cfloat>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <cnoid/MessageView>

using namespace boost::interprocess;
using namespace cnoid;

class PDController_InvertedPendulum : public SimpleController
{
  LinkPtr wheel;
  LinkPtr rod;
  double q_prev;

  AccelerationSensor* accelSensor;
  double theta_prev;

  double dt;

  double rod_rot_q_sum;

public:
  bool initialize(SimpleControllerIO* io) override
  {
    accelSensor = io->body()->findDevice<AccelerationSensor>("RodAccelSensor");
    io->enableInput(accelSensor);
    theta_prev = accelSensor->dv().x();

    wheel = io->body()->link("WHEEL");
    wheel->setActuationMode(Link::JOINT_TORQUE);
    io->enableIO(wheel);
    q_prev = wheel->q();

    rod = io->body()->link("ROD");
    io->enableInput(rod, SimpleControllerIO::LINK_POSITION);
    rod_rot_q_sum = 0.0;

    dt = io->timeStep();

    return true;
  }

  bool control() override
  {
    static const double q_ref = 0.0;
    static const double dq_ref = 0.0;
    static const double theta_ref = 0.0;
    static const double dtheta_ref = 0.0;

    double q = wheel->q();
    double dq = (q - q_prev) / dt;
    Vector3 dv = accelSensor->dv();
    double theta = theta_ref;
    if(dv.norm() > DBL_EPSILON)
      theta = asin((dv / dv.norm()).x()) * 180 / 3.141592;
    double dtheta = (theta - theta_prev) / dt;
    Matrix3 A; A << 1, 0, 0, 0, 0, 1, 0,-1, 0;
    Matrix3 R = rod->rotation();
    rod_rot_q_sum += AngleAxis(A*R).angle();


    shared_memory_object shm_gain{open_or_create, "Gain", read_write};
    shm_gain.truncate(1024);
    mapped_region region_gain{shm_gain, read_only};
    double *gain = static_cast<double*>(region_gain.get_address());

    wheel->u() =
      gain[0] * (q_ref - q)
      + gain[1] * (dq_ref - dq)
      + gain[2] * (theta_ref - theta)
      + gain[3] * (dtheta_ref - dtheta);

    shared_memory_object shm_eval{open_or_create, "Eval", read_write};
    shm_eval.truncate(1024);
    mapped_region region_eval{shm_eval, read_write};
    double *eval = static_cast<double*>(region_eval.get_address());
    eval[0] = rod_rot_q_sum;

    theta_prev = theta;
    q_prev = q;

    for (int i = 0; i < 4; i++)
        std::cerr << "gain[" << i << "]: " << gain[i] << std::endl;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController_InvertedPendulum)
