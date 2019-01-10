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
constexpr double FAIL_PENALTY = 10;
constexpr double FAIL_ANGLE = 1.2;
struct GainWithCost {
    std::vector<double> gains{1.0, 0.0};
    double cost{0};
    int is_finished{0};
};
}

namespace cnoid {

template <class T> class FirstOrderLowPassFilter
{
  private:
    T prev_value;
    double cutoff_freq, dt, const_param;
  public:
    FirstOrderLowPassFilter (const double _cutoff_freq, const double _dt, const T init_value) : prev_value(init_value), dt(_dt)
    {
        setCutOffFreq(_cutoff_freq);
    };
    ~FirstOrderLowPassFilter()
    {
    };
    T passFilter (const T& value)
    {
        prev_value = 1.0 / (1 + const_param) * prev_value + const_param / (1 + const_param) * value;
        return prev_value;
    };
    void reset (const T& value) { prev_value = value; };
    void setCutOffFreq (const double f)
    {
        cutoff_freq = f;
        const_param = 2 * M_PI * cutoff_freq * dt;
    };
    double getCutOffFreq () const { return cutoff_freq; };
    T getCurrentValue () const { return prev_value; };
};


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
    cnoid::Matrix3 init_rod;
    cnoid::Vector3 rod_rot;
    std::shared_ptr<FirstOrderLowPassFilter<cnoid::Vector3>> gyro_filter;

    GainWithCost opt_data;
    double goal_time;
    unsigned int count;
    double dt;

    shared_memory_object shm_gain{};
    const char* GAIN_SHM = "Gain";

  public:
    bool initialize(SimpleControllerIO* io) override
    {
        io->body()->initializeDeviceStates();

        shm_gain = shared_memory_object{open_only, GAIN_SHM, read_write};
        mapped_region region_gain{shm_gain, read_write};
        GainWithCost* data = static_cast<GainWithCost*>(region_gain.get_address());
        opt_data.gains[0] = data->gains[0];
        opt_data.gains[1] = data->gains[1];
        data->is_finished = 0;

        goal_time = 10.0;
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
        gyro_filter = std::make_shared<FirstOrderLowPassFilter<cnoid::Vector3>>(50, dt, cnoid::Vector3::Zero());
        init_rod = rod->rotation();
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
            // gyro_w = gyro_filter->passFilter(gyro_sensor->w());
            gyro_w = gyro_sensor->w();
            // std::cerr << "gyro: " << gyro_sensor->w().transpose() << std::endl;
            wheel->dq() = (wheel->q() - wheel_q_prev) / dt;
            // std::cerr << "q: " << wheel->q() << ", q_prev: " << wheel_q_prev << ", dq: " << wheel->dq() << std::endl;
            // std::cerr << "gyro: " << gyro_sensor->w().transpose() << std::endl;
            // std::cerr << Eigen::AngleAxisd(init_rod.transpose() * rod->rotation()).angle() << std::endl;
            // std::cerr << rod->p().transpose() << std::endl;
            // wheel->u() = gyro_sensor->w().sum() * opt_data.gains[0] + wheel->dq() * opt_data.gains[1];
            // wheel->u() = gyro_sensor->w().sum() * 1.0 + wheel->dq() * opt_data.gains[1];
            // wheel->u() = gyro_sensor->w().sum() * 1.0 + wheel->dq() * 1.0;

            writeData();

            rod_rot += (gyro_w + gyro_prev) * 0.5 * dt;
            wheel_q_prev = wheel->q();
            gyro_prev = gyro_w;

            wheel->u() = gyro_acc * opt_data.gains[0] + rod_rot.sum() * opt_data.gains[1];
            // wheel->u() = 0;
            ++count;
        }

        return 0;
    }

    int calcIsFinished()
    {
        // if (std::abs(accel_sensor->dv().sum()) > 400.0) {
        // std::cerr << std::abs(((gyro_sensor->w() - gyro_prev) / dt).sum()) << std::endl;
        // std::cerr << gyro_sensor->w().transpose() << std::endl;
        gyro_acc = (gyro_w - gyro_prev).sum() / dt;
        // std::cerr << Eigen::AngleAxisd(init_rod.transpose() * rod->rotation()).angle() << std::endl;
        // std::cerr << rod->rotation() << std::endl;
        // std::cerr << "acc:  " << gyro_acc << std::endl;
        if (Eigen::AngleAxisd(init_rod.transpose() * rod->rotation()).angle() > FAIL_ANGLE || rod->rotation().array().isNaN().any()) {
            // std::cerr << "gyro: " << gyro_sensor->w().transpose() << std::endl;
            // std::cerr << "prev: " << gyro_prev.transpose() << std::endl;
            // std::cerr << "acc:  " << gyro_acc << std::endl;
            opt_data.is_finished = -1;
        }
        // else if (wheel->dq() + gyro_sensor->w().sum() < 0.01) opt_data.is_finished = 1;
        return opt_data.is_finished;
    }

    double calcCost() const
    {
        if (opt_data.is_finished == -1) {
            if (rod->rotation().array().isNaN().any()) return (goal_time - count * dt) + FAIL_PENALTY + 100;
            return (goal_time - count * dt) + FAIL_PENALTY;
        } else return (goal_time - count * dt) + std::abs(wheel->dq()) + gyro_w.cwiseAbs().sum();
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

        int tmp_is_finished = calcIsFinished();
        data->cost = calcCost();
        data->is_finished = tmp_is_finished;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OptimizeInvertedPendulum)
}
