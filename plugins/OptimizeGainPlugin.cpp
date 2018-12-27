// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  ResetSimulation.cpp
 * @brief Reset the simulation at regular intervals
 * @author Tatsuya Ishikawa
 */

#include <cnoid/Plugin>
#include <cnoid/RootItem>
#include <cnoid/SimulatorItem>
#include <cnoid/MessageView>
#include <cnoid/Timer>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <string>

using namespace boost::interprocess;
using namespace cnoid;

class OptimizeGainPlugin : public Plugin
{
    SimulatorItemPtr simulator_item_;
    Timer reset_timer_;
    double reset_interval_;
public:

    OptimizeGainPlugin() : Plugin("OptimizeGain")
    {
    }

    virtual bool initialize() override
    {
        reset_timer_.sigTimeout().connect([this]() { resetSimulation(); });
        reset_timer_.setInterval(100); // 100ms
        reset_timer_.start();
        reset_interval_ = 3; // sec
        return true;
    }

private:

    void resetSimulation()
    {
        if (simulator_item_) {
            if (simulator_item_->simulationTime() > reset_interval_) {

                shared_memory_object shm_gain{open_or_create, "Gain", read_write};
                shm_gain.truncate(1024);
                mapped_region region_gain{shm_gain, read_write};
                double *gain = static_cast<double*>(region_gain.get_address());
                for (int i = 0; i < 4; i++)
                    gain[i] = 0.01 * i;

                shared_memory_object shm_eval{open_or_create, "Eval", read_write};
                shm_eval.truncate(1024);
                mapped_region region_eval{shm_eval, read_only};
                double *eval = static_cast<double*>(region_eval.get_address());
                for (int i = 0; i < 1; i++)
                    MessageView::mainInstance()->putln(std::string("eval: ") + std::to_string(eval[i]));

                simulator_item_->startSimulation(true);
            }
        } else {
            ItemPtr robot_item = RootItem::instance()->findItem("InvertedPendulum");
            simulator_item_ = SimulatorItem::findActiveSimulatorItemFor(robot_item);
            if (!simulator_item_) {
                // MessageView::mainInstance()->putln("[OptimizeGainPlugin] Simulator item can't be found. Stop reset_timer_");
                // reset_timer_.stop();
            }
        }
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(OptimizeGainPlugin)
