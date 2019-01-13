// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  OptimizeGainPlugin.cpp
 * @brief Reset the simulation at regular intervals and optimize gains with nlopt
 * @author Hiroki Takeda
 */

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <random>
#include <iostream>
#include <fstream>

#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/SpinBox>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SimulatorItem>
#include <cnoid/SimulationBar>
#include <cnoid/ItemTreeView>
#include <cnoid/Timer>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <nlopt.hpp>

using namespace boost::interprocess;
using namespace cnoid;

namespace {
constexpr size_t NUM_PARAMS = 3;
struct GainWithCost {
    std::vector<double> gains{1.0, 1.0, 1.0};
    double cost{0};
    int is_finished{0};
};
std::mutex mtx;
}

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data);

class OptimizeGainPlugin : public Plugin
{
    std::mt19937 mt{};
    std::uniform_real_distribution<double> uni_dist{-10, 10};

    double goal_time;
    double stop_threshold;

    SimulatorItemPtr simulator_item_;
    BodyItemPtr body_item;

    const char* GAIN_SHM = "Gain";
    shared_memory_object shm_gain{};
    mapped_region region_gain{};

    GainWithCost opt_data;
    nlopt::opt opt{};
    std::thread opt_thread;

    cnoid::Timer reset_timer_;
    cnoid::Timer force_timer_;
    bool will_reset_simulation = false;
    cnoid::Signal<void()> sigRequestResetSimulation_;

    std::ofstream outputfile{"cost.txt"};

  public:
    OptimizeGainPlugin() : Plugin("OptimizeGain")
    {
    }

    bool initialize() override
    {
        mt = std::mt19937{std::random_device{}()};

        shm_gain.remove(GAIN_SHM);
        shm_gain = shared_memory_object{create_only, GAIN_SHM, read_write};
        shm_gain.truncate(1024);
        region_gain = mapped_region{shm_gain, read_write};
        GainWithCost* shm_data = static_cast<GainWithCost*>(region_gain.get_address());
        std::memcpy(shm_data, &opt_data, sizeof(opt_data));

        goal_time = 10.0;
        stop_threshold = 0.5;
        opt = nlopt::opt(nlopt::GN_ISRES, NUM_PARAMS);

        std::vector<double> lb(NUM_PARAMS, -100);
        opt.set_lower_bounds(lb);
        std::vector<double> ub(NUM_PARAMS, 100);
        opt.set_upper_bounds(ub);

        opt.set_max_objective(costFuncWrapper, this);
        opt.set_stopval(goal_time - stop_threshold);

        cnoid::RootItem::instance()->sigItemAdded().connect([this](Item* _item) {
                SimulatorItemPtr itemptr = dynamic_cast<cnoid::SimulatorItem*>(_item);
                if (itemptr) {
                    this->simulator_item_ = itemptr;
                    reset_timer_.sigTimeout().connect([this]() {
                            if (will_reset_simulation) {
                                this->simulator_item_->startSimulation(true);
                                will_reset_simulation = false;
                            }
                        });
                    reset_timer_.setInterval(20); // 20ms

                    sigRequestResetSimulation_.connect([this]() {
                            mtx.lock();
                            will_reset_simulation = true;
                            mtx.unlock();
                        });

                    force_timer_.sigTimeout().connect([this]() {
                            addExternalForceToRod(uni_dist(mt));
                        });
                    force_timer_.setInterval(50);
                }
            });

        std::unique_ptr<ToolBar> bar = std::make_unique<ToolBar>(this->name());
        bar->setVisibleByDefault(true);

        ToolButton* button = bar->addToggleButton("StartOptimization");
        button->sigToggled().connect([this](bool checked) {
                if (checked) {
                    body_item = ItemTreeView::instance()->selectedItem<BodyItem>();
                    if (!body_item || body_item->body()->isStaticModel()) {
                        putMessage("Toggle button with checking robot body item");
                        return;
                    }

                    if (!simulator_item_->isActive()) {
                        SimulationBar::instance()->startSimulation(true);
                    }
                    this->reset_timer_.start();
                    this->force_timer_.start();
                    opt_thread = std::thread([this]() { this->runNLOPT(); });
                } else {
                    this->reset_timer_.stop();
                    this->force_timer_.stop();
                    if (opt_thread.joinable()) opt_thread.detach();
                }
            });

        {
            std::unique_ptr<SpinBox> timeSpin = std::make_unique<SpinBox>();
            timeSpin->setMaximum(1000);
            timeSpin->setValue(goal_time);
            timeSpin->sigValueChanged().connect([this](const int value) {
                    this->goal_time = value;
                    opt.set_stopval(this->goal_time - stop_threshold);
                });
            bar->addWidget(new QLabel("GoalTime"));
            bar->addWidget(timeSpin.release());
        }
        {
            std::unique_ptr<SpinBox> distSpin = std::make_unique<SpinBox>();
            distSpin->setMaximum(10000);
            distSpin->setValue(10);
            distSpin->sigValueChanged().connect([this](const int value) {
                    uni_dist = std::uniform_real_distribution<double>(-value, value);
                });
            bar->addWidget(new QLabel("Disturbance"));
            bar->addWidget(distSpin.release());
        }
        {
            std::unique_ptr<DoubleSpinBox> threSpin = std::make_unique<DoubleSpinBox>();
            threSpin->setMaximum(1000);
            threSpin->setValue(stop_threshold);
            threSpin->sigValueChanged().connect([this](const double value) {
                    this->stop_threshold = value;
                    this->opt.set_stopval(this->goal_time - stop_threshold);
                });
            bar->addWidget(new QLabel("StopThre"));
            bar->addWidget(threSpin.release());
        }

        addToolBar(bar.release());
        return true;
    }

    bool finalize() override
    {
        shm_gain.remove(GAIN_SHM);
        opt_thread.detach();
        outputfile.close();

        return true;
    }

    cnoid::SignalProxy<void()> sigRequestResetSimulation() { return sigRequestResetSimulation_; }

    void addExternalForceToRod(const double force_x, const double time = 0.1)
    {
        const cnoid::Vector3 force(force_x, 0, 0);
        simulator_item_->setExternalForce(body_item, body_item->body()->link("ROD"), cnoid::Vector3::Zero(), force, time);
    }

    double costFunc(const std::vector<double> &gains, std::vector<double> &grad, void *data)
    {
        GainWithCost* shm_data = static_cast<GainWithCost*>(region_gain.get_address());
        for (size_t i = 0; i < gains.size(); ++i) {
            shm_data->gains[i] = gains[i];
        }
        shm_data->is_finished = 0;
        sigRequestResetSimulation_();

        while ((simulator_item_->simulationTime() < goal_time && shm_data->is_finished == 0) || will_reset_simulation) {
            // putMessage("cost: " + std::to_string(shm_data->cost));
            std::this_thread::sleep_for(std::chrono::microseconds(2));
        }
        putMessage("gains: " + std::to_string(opt_data.gains[0]) + ", " + std::to_string(opt_data.gains[1]) + ", " + std::to_string(opt_data.gains[2]));
        putMessage("cost: " + std::to_string(shm_data->cost));
        outputfile << shm_data->cost << " ";
        return shm_data->cost;
    }

  private:

    void putMessage(const std::string& msg) // const // plugin->name() is not const function
    {
        MessageView::mainInstance()->putln("[" + std::string(this->name()) + "Plugin] " + msg);
    }

    void runNLOPT()
    {
        double minimum;
        nlopt::result result = opt.optimize(opt_data.gains, minimum);
        std::cerr << "NLOPT finished" << std::endl;
        std::cerr << "result: " << result << std::endl;
        std::cerr << "minimized cost: " << minimum << std::endl;
        std::cerr << "gains: ";
        for (const auto gain : opt_data.gains) std::cerr << gain << ", ";
        std::cerr << std::endl;
        outputfile.flush();
    }
};

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data)
{
    reinterpret_cast<OptimizeGainPlugin*>(data)->costFunc(gains, grad, data);
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(OptimizeGainPlugin)
