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
constexpr size_t NUM_PARAMS = 2;
constexpr double FAIL_PENALTY = 20;
}

namespace cnoid {

namespace {
struct GainWithCost {
    std::vector<double> gains{1.0, 0.0};
    double cost{0};
    int is_finished{0};
};
}

class OptimizeInvertedPendulum : public SimpleController
{
    LinkPtr wheel;
    LinkPtr rod;
    RateGyroSensorPtr gyro_sensor;
    AccelerationSensorPtr accel_sensor;

    double wheel_q_prev;
    cnoid::Vector3 gyro_prev;

    GainWithCost opt_data;
    double goal_time;
    unsigned int count;
    double dt;

    shared_memory_object shm_gain{};
    const char* GAIN_SHM = "Gain";

  public:
    bool initialize(SimpleControllerIO* io) override
    {
        shm_gain = shared_memory_object{open_only, GAIN_SHM, read_write};

        accel_sensor = io->body()->findDevice<AccelerationSensor>("RodAccelSensor");
        io->enableInput(accel_sensor);
        gyro_sensor = io->body()->findDevice<RateGyroSensor>("RodGyro");
        io->enableInput(gyro_sensor);
        gyro_prev.setZero();

        wheel = io->body()->link("WHEEL");
        wheel->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(wheel);
        wheel_q_prev = 0;
        rod = io->body()->link("ROD");
        io->enableInput(rod);

        goal_time = 10.0;
        count = 0;
        dt = io->timeStep();
        return true;
    }

    bool start() override
    {
        readGain();
        mapped_region region_gain{shm_gain, read_write};
        GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());
        data->is_finished = 0;

        return true;
    }

    bool control() override
    {
        wheel->dq() = (wheel->q() - wheel_q_prev) / dt;
        // MessageView::mainInstance()->putln("q: " + std::to_string(wheel->q()));
        // MessageView::mainInstance()->putln("q_prev: " + std::to_string(wheel_q_prev));
        // MessageView::mainInstance()->putln("dq: " + std::to_string(wheel->dq()));
        // std::cerr << "q: " << wheel->q() << ", q_prev: " << wheel_q_prev << ", dq: " << wheel->dq() << std::endl;
        // std::cerr << "gyro: " << gyro_sensor->w().transpose() << std::endl;
        // std::cerr << rod->p().transpose() << std::endl;
        wheel->u() = gyro_sensor->w().sum() * opt_data.gains[0] + wheel->dq() * opt_data.gains[1];
        // wheel->u() = gyro_sensor->w().sum() * 1.0 + wheel->dq() * opt_data.gains[1];
        // wheel->u() = gyro_sensor->w().sum() * 1.0 + wheel->dq() * 1.0;

        writeData();

        wheel_q_prev = wheel->q();
        gyro_prev = gyro_sensor->w();
        ++count;
        return 0;
    }

    int calcIsFinished()
    {
        if (opt_data.is_finished == 0) {
            // if (std::abs(accel_sensor->dv().sum()) > 400.0) {
            // std::cerr << std::abs(((gyro_sensor->w() - gyro_prev) / dt).sum()) << std::endl;
            if ((gyro_sensor->w() - gyro_prev).cwiseAbs().sum() / dt > 200.0) {
                // MessageView::mainInstance()->putln("dv: " + std::to_string(accel_sensor->dv().sum()));
                opt_data.is_finished = -1;
            }
            // else if (wheel->dq() + gyro_sensor->w().sum() < 0.01) opt_data.is_finished = 1;
        }
        return opt_data.is_finished;
    }

    double calcCost() const
    {
        if (opt_data.is_finished == -1) return count * dt + FAIL_PENALTY;
        else if (count * dt < goal_time) {
            return count * dt;
        } else {
            return goal_time + std::abs(wheel->dq()) + gyro_sensor->w().cwiseAbs().sum();
        }
    }

    void readGain()
    {
        mapped_region region_gain{shm_gain, read_only};
        GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());
        opt_data.gains[0] = data->gains[0];
        opt_data.gains[1] = data->gains[1];
    }

    void writeData()
    {
        mapped_region region_gain{shm_gain, read_write};
        GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());
        data->is_finished = calcIsFinished();
        data->cost = calcCost();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OptimizeInvertedPendulum)
}
