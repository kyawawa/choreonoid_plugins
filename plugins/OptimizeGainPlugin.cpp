// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  OptimizeGainPlugin.cpp
 * @brief Reset the simulation at regular intervals and optimize gains with nlopt
 * @author Hiroki Takeda
 */

#include <string>

#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SimulatorItem>
#include <cnoid/ItemTreeView>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include <nlopt.hpp>

using namespace boost::interprocess;
using namespace cnoid;

class OptimizeGainPlugin : public Plugin
{
    SimulatorItemPtr simulator_item_;
    shared_memory_object shm_gain{};
    shared_memory_object shm_eval{};
    const char* GAIN_SHM = "Gain";
    const char* EVAL_SHM = "Eval";

    nlopt::opt optim{};

  public:
    OptimizeGainPlugin() : Plugin("OptimizeGain")
    {
        require("Body");
    }

    bool initialize() override
    {
        shm_gain.remove(GAIN_SHM);
        shm_eval.remove(EVAL_SHM);
        shm_gain = shared_memory_object{create_only, GAIN_SHM, read_write};
        shm_gain.truncate(1024);
        shm_eval = shared_memory_object{create_only, EVAL_SHM, read_write};
        shm_eval.truncate(1024);

        cnoid::RootItem::instance()->sigItemAdded().connect([this](Item* _item) {
                SimulatorItemPtr itemptr = dynamic_cast<cnoid::SimulatorItem*>(_item);
                if (itemptr) this->simulator_item_ = itemptr;
            });

        std::unique_ptr<ToolBar> bar = std::make_unique<ToolBar>(this->name());
        bar->setVisibleByDefault(true);
        ToolButton* button = bar->addToggleButton("StartOptimization");
        button->sigToggled().connect([this](bool checked) {
                if (checked) {
                    BodyItemPtr body_item = ItemTreeView::instance()->selectedItem<BodyItem>();
                    if (body_item) {
                        if (body_item->body()->isStaticModel()) {
                            putMessage("Please select non-static model");
                            return;
                        }
                        putMessage("Body Found!!");
                        // startNLOPT();
                    } else {
                        putMessage("Selected item isn't BodyItem");
                        return;
                    }
                }
            });
        addToolBar(bar.release());

        return true;
    }

    bool finalize() override
    {
        shm_gain.remove(GAIN_SHM);
        shm_eval.remove(EVAL_SHM);
        return true;
    }

  private:

    void putMessage(const std::string&& msg) // const
    {
        MessageView::mainInstance()->putln("[" + std::string(this->name()) + "Plugin] " + msg);
    }

    // void startNLOPT()
    // {
    //     optim = nlopt::opt(nlopt::GN_ISRES, 1); // TODO
    //     std::vector<double> lb(2);
    //     lb[0] = -100; lb[1] = -100;
    //     opt.set_lower_bounds(lb);

    //     std::vector<double> ub(2);
    //     ub[0] = 100; ub[1] = 100;
    //     opt.set_upper_bounds(ub);

    //     opt.set_min_objective(myfunc, NULL);
    // }

    void evalNLOPT()
    {
        mapped_region region_gain{shm_gain, read_write};
        double *gain = static_cast<double*>(region_gain.get_address());
        for (int i = 0; i < 4; i++)
            gain[i] = 0.01 * i;

        mapped_region region_eval{shm_eval, read_only};
        double *eval = static_cast<double*>(region_eval.get_address());
        for (int i = 0; i < 1; i++)
            MessageView::mainInstance()->putln(std::string("eval: ") + std::to_string(eval[i]));
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(OptimizeGainPlugin)
