// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  optimize_inverted_pendulum.h
 * @brief
 * @author Tatsuya Ishikawa
 */

#ifndef __OPTIMIZE_INVERTED_PENDULUM_H__
#define __OPTIMIZE_INVERTED_PENDULUM_H__

#include <vector>
#include <cnoid/Referenced>
#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>

namespace cnoid {

constexpr size_t NUM_PARAMS = 2;

class OptimizeInvertedPendulum : public SimpleController, public Referenced
{
    LinkPtr wheel;
    LinkPtr rod;
    RateGyroSensorPtr gyro_sensor;
    AccelerationSensorPtr accel_sensor;
    std::vector<double> gains{NUM_PARAMS};
    double goal_time;
    unsigned int count;
    double dt;
    bool is_failed;

  public:
    bool initialize(SimpleControllerIO* io) override;
    bool start() override;
    bool control() override;
    void setGoalTime(const double _time) { goal_time = _time; }
    void setGain(const std::vector<double>& _gain) { gains = _gain; }
    bool isFailed() const;
    double calcCost() const;
};

}

#endif // __OPTIMIZE_INVERTED_PENDULUM_H__
