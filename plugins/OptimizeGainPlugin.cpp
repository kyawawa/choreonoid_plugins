// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  OptimizeGainPlugin.cpp
 * @brief Reset the simulation at regular intervals and optimize gains with nlopt
 * @author Hiroki Takeda
 */

#include <string>
#include <vector>
#include <thread>

#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SimpleControllerItem>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/SimulationBar>
#include <cnoid/ItemTreeView>

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
    bool is_failed{false};
};
}

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data);

class OptimizeGainPlugin : public Plugin
{
    SimulatorItemPtr simulator_item_;
    double goal_time;
    BodyItemPtr body_item;

    const char* GAIN_SHM = "Gain";
    shared_memory_object shm_gain{};
    mapped_region region_gain{};

    GainWithCost opt_data;
    nlopt::opt opt{};
    std::thread opt_thread;

    cnoid::Signal<void()> sigRequestResetSimulation_;

  public:
    OptimizeGainPlugin() : Plugin("OptimizeGain")
    {
    }

    bool initialize() override
    {
        shm_gain.remove(GAIN_SHM);
        shm_gain = shared_memory_object{create_only, GAIN_SHM, read_write};
        shm_gain.truncate(1024);
        region_gain = mapped_region{shm_gain, read_write};
        GainWithCost* shm_data = static_cast<GainWithCost*>(region_gain.get_address());
        std::memcpy(shm_data, &opt_data, sizeof(opt_data));

        goal_time = 3.0;
        opt = nlopt::opt(nlopt::GN_ISRES, NUM_PARAMS);
        // opt_gains = {1.0, 0.0};
        // opt_data.gains = {1.0, 0.0};

        std::vector<double> lb = {-100, -100};
        opt.set_lower_bounds(lb);
        std::vector<double> ub = {100, 100};
        opt.set_upper_bounds(ub);

        opt.set_xtol_rel(1e-4);
        opt.set_min_objective(costFuncWrapper, this);

        cnoid::RootItem::instance()->sigItemAdded().connect([this](Item* _item) {
                SimulatorItemPtr itemptr = dynamic_cast<cnoid::SimulatorItem*>(_item);
                if (itemptr) {
                    this->simulator_item_ = itemptr;
                    sigRequestResetSimulation_.connect([this]() { this->simulator_item_->startSimulation(true); });
                }
            });

        std::unique_ptr<ToolBar> bar = std::make_unique<ToolBar>(this->name());
        bar->setVisibleByDefault(true);

        ToolButton* button = bar->addToggleButton("StartOptimization");
        button->sigToggled().connect([this](bool checked) {
                if (checked) {
                    if (!simulator_item_->isActive()) SimulationBar::instance()->startSimulation(true);
                    opt_thread = std::thread([this]() { this->runNLOPT(); });
                    // if (!simulator_item_->isActive()) SimulationBar::instance()->startSimulation(true);
                    // body_item = ItemTreeView::instance()->selectedItem<BodyItem>();
                    // if (body_item) {
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
                    // putMessage("Toggle button with checking robot body item");
                } else opt_thread.detach();
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
        return true;
    }

    cnoid::SignalProxy<void()> sigRequestResetSimulation() { return sigRequestResetSimulation_; }

    // bool findControllerItem() {
    //     Item* child = body_item->childItem();
    //     while (child) {
    //         SimpleControllerItemPtr controller_item = dynamic_cast<SimpleControllerItem*>(child);
    //         if (controller_item) {
    //             auto controller_ptr = dynamic_cast<OptimizeInvertedPendulum*>(controller_item->controller());
    //             if (controller_ptr) {
    //                 // Initialize NLOPT
    //                 this->inverted_pendulum_ = controller_ptr;
    //                 return true;
    //             }
    //         }
    //         child = child->nextItem();
    //     }
    //     putMessage("Can't find OptimizeInvertedPendulum controller");
    //     return false;
    // }

    double costFunc(const std::vector<double> &gains, std::vector<double> &grad, void *data)
    {
        GainWithCost* shm_data = static_cast<GainWithCost*>(region_gain.get_address());
        shm_data->gains[0] = gains[0];
        shm_data->gains[1] = gains[1];
        // simulator_item_->startSimulation(true);
        sigRequestResetSimulation_();

        if (shm_data->is_failed) {
            putMessage("failed");
        } else {
            putMessage("not failed");
        }
        while (simulator_item_->simulationTime() < goal_time && !(shm_data->is_failed)) {
            std::this_thread::sleep_for(std::chrono::microseconds(20));
        }

        shm_data->is_failed = false;
        putMessage("cost: " + std::to_string(shm_data->cost) + ", gains[0]: " + std::to_string(gains[0]) + ", gains[1]: " + std::to_string(gains[1]));
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
        putMessage("minumum: " + std::to_string(minimum));
        putMessage("gains: " + std::to_string(opt_data.gains[0]) + ", " + std::to_string(opt_data.gains[1]));
    }
};

double costFuncWrapper(const std::vector<double> &gains, std::vector<double> &grad, void *data)
{
    reinterpret_cast<OptimizeGainPlugin*>(data)->costFunc(gains, grad, data);
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(OptimizeGainPlugin)
