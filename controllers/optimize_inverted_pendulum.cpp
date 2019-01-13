// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  optimize_inverted_pendulum.cpp
 * @brief
 * @author Tatsuya Ishikawa
 */

#include <vector>
#include <cnoid/Referenced>
#include <cnoid/SimpleController>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/MessageView>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <iostream>

using namespace boost::interprocess;

namespace {
constexpr size_t NUM_PARAMS = 3;
constexpr double FAIL_PENALTY = 30;
constexpr double NAN_PENALTY = 100;
constexpr double FAIL_ANGLE = 1.2;
struct GainWithCost {
    std::vector<double> gains{1.0, 1.0, 1.0};
    double cost{0};
    int is_finished{0};
};
}

namespace cnoid {

class OptimizeInvertedPendulum : public SimpleController
{
    LinkPtr wheel;
    LinkPtr rod;
    RateGyroSensorPtr gyro_sensor;
    AccelerationSensorPtr accel_sensor;

    double wheel_q_prev;
    cnoid::Vector3 gyro_prev;
    cnoid::Vector3 gyro_w;
    double gyro_acc;
    cnoid::Position init_rod;
    cnoid::Vector3 rod_rot;

    GainWithCost opt_data;
    unsigned int count;
    double dt;

    shared_memory_object shm_gain{};
    const char* GAIN_SHM = "Gain";

  public:
    bool initialize(SimpleControllerIO* io) override
    {
        shm_gain = shared_memory_object{open_only, GAIN_SHM, read_write};

        {
            mapped_region region_gain{shm_gain, read_write};
            GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());
            for (size_t i = 0; i < opt_data.gains.size(); ++i) {
                opt_data.gains[i] = data->gains[i];
            }
            data->is_finished = 0;
        }

        count = 0;
        dt = io->timeStep();

        accel_sensor = io->body()->findDevice<AccelerationSensor>("RodAccelSensor");
        io->enableInput(accel_sensor);
        gyro_sensor = io->body()->findDevice<RateGyroSensor>("RodGyro");
        io->enableInput(gyro_sensor);

        wheel = io->body()->link("WHEEL");
        wheel->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(wheel);

        rod = io->body()->link("ROD");
        io->enableInput(rod, SimpleControllerIO::LINK_POSITION);

        wheel_q_prev = 0;
        gyro_prev.setZero();
        gyro_w.setZero();
        gyro_acc = 0;
        init_rod = rod->position();
        rod_rot.setZero();

        return true;
    }

    bool start() override
    {
        return true;
    }

    bool control() override
    {
        if (opt_data.is_finished == 0) {
            gyro_w = gyro_sensor->w();
            wheel->dq() = (wheel->q() - wheel_q_prev) / dt;

            {
                mapped_region region_gain{shm_gain, read_write};
                GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());

                data->is_finished = calcIsFinished();
                data->cost = calcCost();
            }

            rod_rot += (gyro_w + gyro_prev) * 0.5 * dt;
            wheel_q_prev = wheel->q();
            gyro_prev = gyro_w;

            wheel->u() = gyro_acc * opt_data.gains[0] + rod_rot.sum() * opt_data.gains[1] + wheel->q() * opt_data.gains[2];
            ++count;
        }

        return 0;
    }

    int calcIsFinished()
    {
        gyro_acc = (gyro_w - gyro_prev).sum() / dt;
        if (Eigen::AngleAxisd(init_rod.linear().transpose() * rod->rotation()).angle() > FAIL_ANGLE || rod->rotation().array().isNaN().any()) {
            opt_data.is_finished = -1;
        }
        return opt_data.is_finished;
    }

    /// Maximize cost
    /// If failed:
    ///     If body has NaN: cost = current time - (fail penalty + NaN penalty)
    ///     else:            cost = current time - fail penalty
    /// else:                cost = current time - (wheel velocity + sum of gyro + displacement from initial position)
    double calcCost() const
    {
        if (opt_data.is_finished == -1) {
            if (rod->rotation().array().isNaN().any()) return count * dt - (FAIL_PENALTY + NAN_PENALTY);
            return count * dt - FAIL_PENALTY;
        } else return count * dt - (std::abs(wheel->dq()) + gyro_w.cwiseAbs().sum() + (init_rod.translation() - rod->translation()).cwiseAbs().sum());
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OptimizeInvertedPendulum)
}
