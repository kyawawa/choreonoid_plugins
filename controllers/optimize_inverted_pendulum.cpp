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

using namespace boost::interprocess;

namespace {
constexpr size_t NUM_PARAMS = 2;
constexpr double FAIL_PENALTY = 100;
}

namespace cnoid {

namespace {
struct GainWithCost {
    std::vector<double> gains{1.0, 0.0};
    double cost{0};
    bool is_failed{false};
};
}

class OptimizeInvertedPendulum : public SimpleController
{
    LinkPtr wheel;
    LinkPtr rod;
    RateGyroSensorPtr gyro_sensor;
    AccelerationSensorPtr accel_sensor;

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

        wheel = io->body()->link("WHEEL");
        wheel->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(wheel);
        rod = io->body()->link("ROD");
        io->enableInput(rod);

        goal_time = 3.0;
        count = 0;
        dt = io->timeStep();
        return true;
    }

    bool start() override
    {
        return true;
    }

    bool control() override
    {
        wheel->u() = gyro_sensor->w().sum() * opt_data.gains[0] + wheel->dq() * opt_data.gains[1];
        writeData();
        ++count;
        return 0;
    }

    // void setGoalTime(const double _time) { goal_time = _time; }
    // void setGain(const std::vector<double>& _gain) { gains = _gain; }
    bool calcIsFailed()
    {
        opt_data.is_failed = (opt_data.is_failed || std::abs(accel_sensor->dv().sum()) > 500.0);
        return opt_data.is_failed;
    }

    double calcCost() const
    {
        if (opt_data.is_failed) return count * dt + FAIL_PENALTY;
        if (count * dt < goal_time) {
            return count * dt;
        } else {
            return goal_time + wheel->dq() + gyro_sensor->w().sum();
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
        data->is_failed = calcIsFailed();
        data->cost = calcCost();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OptimizeInvertedPendulum)
}
