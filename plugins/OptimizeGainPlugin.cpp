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

#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SimulatorItem>
#include <cnoid/SimulationBar>
#include <cnoid/ItemTreeView>
#include <cnoid/Timer>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <nlopt.hpp>

// #include "../controllers/optimize_inverted_pendulum.h"

using namespace boost::interprocess;
using namespace cnoid;

namespace {
constexpr size_t NUM_PARAMS = 2;
struct GainWithCost {
    std::vector<double> gains{1.0, 0.0};
    double cost{0};
    int is_finished{0};
};
std::mutex mtx;
}

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data);

class OptimizeGainPlugin : public Plugin
{
    std::mt19937 mt{};
    std::uniform_real_distribution<double> uni_dist{-40, 40};

    SimulatorItemPtr simulator_item_;
    double goal_time;
    BodyItemPtr body_item;

    const char* GAIN_SHM = "Gain";
    shared_memory_object shm_gain{};
    mapped_region region_gain{};

    GainWithCost opt_data;
    nlopt::opt opt{};
    std::thread opt_thread;

    cnoid::Timer reset_timer_;
    bool will_reset_simulation = false;
    cnoid::Signal<void()> sigRequestResetSimulation_;

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
        opt = nlopt::opt(nlopt::GN_ISRES, NUM_PARAMS);
        // opt = nlopt::opt(nlopt::LN_PRAXIS, NUM_PARAMS);

        std::vector<double> lb = {-30, -30};
        opt.set_lower_bounds(lb);
        std::vector<double> ub = {30, 30};
        opt.set_upper_bounds(ub);

        // opt.set_xtol_rel(1e-10);
        // opt.set_ftol_abs(1e-10);
        opt.set_min_objective(costFuncWrapper, this);
        opt.set_stopval(5.0);

        cnoid::RootItem::instance()->sigItemAdded().connect([this](Item* _item) {
                SimulatorItemPtr itemptr = dynamic_cast<cnoid::SimulatorItem*>(_item);
                if (itemptr) {
                    this->simulator_item_ = itemptr;
                    reset_timer_.sigTimeout().connect([this]() {
                            mtx.lock();
                            if (will_reset_simulation) {
                                putMessage("Sig Start");
                                this->simulator_item_->startSimulation(true);
                                addExternalForceToRod(uni_dist(mt));
                                will_reset_simulation = false;
                            }
                            mtx.unlock();
                        });
                    reset_timer_.setInterval(20); // 20ms
                    sigRequestResetSimulation_.connect([this]() {
                            putMessage("Sig Reset");
                            mtx.lock();
                            will_reset_simulation = true;
                            mtx.unlock();
                        });
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
                        addExternalForceToRod(uni_dist(mt));
                    }
                    this->reset_timer_.start();
                    opt_thread = std::thread([this]() { this->runNLOPT(); });
                    // if (!simulator_item_->isActive()) SimulationBar::instance()->startSimulation(true);
                    //     if (findControllerItem()) {
                    //         opt_thread = std::thread([this]() { this->runNLOPT(); });
                    //         // manage_nlopt = std::thread([this]() {
                    //         //         opt_thread.join();
                    //         //         putMessage("NLOPT Completed");
                    //         //         simulator_item_->startSimulation(true);
                    //         //         findControllerItem();
                    //         //     });
                    //     }
                    // }
                } else {
                    this->reset_timer_.stop();
                    if (opt_thread.joinable()) opt_thread.detach();
                }
            });
        // button->sigToggled().connect([this](bool checked) {
        //         if (checked) {
        //             SimpleControllerItemPtr controller_item = ItemTreeView::instance()->selectedItem<SimpleControllerItem>();
        //             if (controller_item) {
        //                 SimulationBar::instance()->startSimulation(true);
        //                 auto controller_ptr = dynamic_cast<OptimizeInvertedPendulum*>(controller_item->controller());
        //                 if (controller_ptr) {
        //                     // Initialize NLOPT
        //                     this->inverted_pendulum_ = controller_ptr;
        //                     return;
        //                 }
        //             }
        //             putMessage("Toggle button with checking optimize controller");
        //         }
        //     });

        addToolBar(bar.release());
        return true;
    }

    bool finalize() override
    {
        shm_gain.remove(GAIN_SHM);
        opt_thread.detach();
        return true;
    }

    cnoid::SignalProxy<void()> sigRequestResetSimulation() { return sigRequestResetSimulation_; }

    void addExternalForceToRod(const double force_x)
    {
        const cnoid::Vector3 force(force_x, 0, 0);
        simulator_item_->setExternalForce(body_item, body_item->body()->link("ROD"), cnoid::Vector3::Zero(), force, 0.1);
        // body_item->body()->link("ROD")->addExternalForce(force, cnoid::Vector3::Zero());
    }

    double costFunc(const std::vector<double> &gains, std::vector<double> &grad, void *data)
    {
        GainWithCost* shm_data = static_cast<GainWithCost*>(region_gain.get_address());
        shm_data->gains[0] = gains[0];
        shm_data->gains[1] = gains[1];
        shm_data->is_finished = 0;
        sigRequestResetSimulation_();

        while ((simulator_item_->simulationTime() < goal_time && shm_data->is_finished == 0) || will_reset_simulation) {
            // putMessage("cost: " + std::to_string(shm_data->cost));
            std::this_thread::sleep_for(std::chrono::microseconds(2));
        }
        putMessage("time: " + std::to_string(simulator_item_->simulationTime()));
        putMessage("finished: " + std::to_string(shm_data->is_finished));
        if (will_reset_simulation) putMessage("will_reset_simulation");
        putMessage("gains: " + std::to_string(opt_data.gains[0]) + ", " + std::to_string(opt_data.gains[1]));
        putMessage("cost: " + std::to_string(shm_data->cost));
        return shm_data->cost;
    }

  private:

    void putMessage(const std::string& msg) // const
    {
        MessageView::mainInstance()->putln("[" + std::string(this->name()) + "Plugin] " + msg);
    }

    void runNLOPT()
    {
        double minimum;
        nlopt::result result = opt.optimize(opt_data.gains, minimum);
        putMessage("result: " + std::to_string(result));
        putMessage("minumum: " + std::to_string(minimum));
        putMessage("gains: " + std::to_string(opt_data.gains[0]) + ", " + std::to_string(opt_data.gains[1]));
    }
};

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data)
{
    reinterpret_cast<OptimizeGainPlugin*>(data)->costFunc(gains, grad, data);
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(OptimizeGainPlugin)
