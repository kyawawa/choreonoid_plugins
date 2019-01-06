// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  ResetSimulation.cpp
 * @brief Reset the simulation at regular intervals
 * @author Tatsuya Ishikawa
 */

#include <cnoid/MessageView>
#include <cnoid/ToolBar>
#include <cnoid/SpinBox>

#include "ResetSimulationPlugin.h"
#include "../util/util.h"

using namespace cnoid;

bool ResetSimulationPlugin::initialize()
{
    reset_timer_.sigTimeout().connect([this]() { this->resetSimulation(); });
    reset_timer_.setInterval(100); // 100ms
    reset_interval_ = 3.0;

    std::unique_ptr<ToolBar> bar = std::make_unique<ToolBar>("ResetSimulation");
    bar->setVisibleByDefault(true);

    ToolButton* button = bar->addToggleButton("ResetSimulation");
    button->sigToggled().connect([this](bool checked) {
            if (checked) this->reset_timer_.start();
            else this->reset_timer_.stop();
        });

    std::unique_ptr<SpinBox> timeSpin = std::make_unique<SpinBox>();
    timeSpin->setMaximum(1000000);
    timeSpin->setValue(reset_interval_);
    timeSpin->sigValueChanged().connect([this](const int value) { this->reset_interval_ = value; });
    bar->addWidget(timeSpin.release());

    addToolBar(bar.release());
    return true;
}

void ResetSimulationPlugin::resetSimulation()
{
    if (simulator_item_) {
        if (simulator_item_->simulationTime() > reset_interval_) {
            sigResetSimulation_();
            simulator_item_->startSimulation(true);
        }
    } else {
        simulator_item_ = findSimulatorItem("AISTSimulator");
        if (!simulator_item_) {
            MessageView::mainInstance()->putln("[ResetSimulationPlugin] Simulator item can't be found. Stop reset_timer_");
            reset_timer_.stop();
        }
    }
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ResetSimulationPlugin)
