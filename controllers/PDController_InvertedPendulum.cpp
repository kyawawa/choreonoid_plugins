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

using namespace boost::interprocess;
using namespace cnoid;

class PDController_InvertedPendulum : public SimpleController
{
  Link* joint;
  double q_prev;
  
  AccelerationSensor* accelSensor;
  double theta_prev;

  double dt;

  double* gain;
  
public:
  bool initialize(SimpleControllerIO* io) override
  {
    accelSensor = io->body()->findDevice<AccelerationSensor>("RodAccelSensor");
    io->enableInput(accelSensor);
    theta_prev = accelSensor->dv().x();

    joint = io->body()->link("WHEEL");
    joint->setActuationMode(Link::JOINT_TORQUE);
    io->enableIO(joint);
    q_prev = joint->q();

    dt = io->timeStep();

    return true;
  }

  bool control() override
  {
    static const double q_ref = 0.0;
    static const double dq_ref = 0.0;
    static const double theta_ref = 0.0;
    static const double dtheta_ref = 0.0;

    double q = joint->q();
    double dq = (q - q_prev) / dt;
    Vector3 dv = accelSensor->dv();
    double theta = theta_ref;
    if(dv.norm() > DBL_EPSILON)
      theta = asin((dv / dv.norm()).x()) * 180 / 3.141592;
    double dtheta = (theta - theta_prev) / dt;

    shared_memory_object shdmem{open_or_create, "Gain", read_write};
    shdmem.truncate(1024);
    mapped_region region{shdmem, read_only};
    double *gain = static_cast<double*>(region.get_address());
    
    joint->u() = 
      gain[0] * (q_ref - q)
      + gain[1] * (dq_ref - dq)
      + gain[2] * (theta_ref - theta)
      + gain[3] * (dtheta_ref - dtheta);

    theta_prev = theta;
    q_prev = q;

    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController_InvertedPendulum)


