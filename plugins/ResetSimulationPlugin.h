// -*- mode: C++; coding: utf-8-unix; -*-

/**
 * @file  ResetSimulationPlugin.h
 * @author Tatsuya Ishikawa
 */

#ifndef __RESETSIMULATIONPLUGIN_H__
#define __RESETSIMULATIONPLUGIN_H__

#include <cnoid/Plugin>
#include <cnoid/PluginManager>
#include <cnoid/Signal>
#include <cnoid/SimulatorItem>
#include <cnoid/Timer>

class ResetSimulationPlugin : public cnoid::Plugin
{
    cnoid::SimulatorItemPtr simulator_item_;
    cnoid::Timer reset_timer_;
    double reset_interval_;
    cnoid::Signal<void()> sigResetSimulation_;

public:
    ResetSimulationPlugin() : Plugin("ResetSimulation")
    {
    }

    bool initialize() override;
    cnoid::SignalProxy<void()> sigResetSimulation() { return sigResetSimulation_; }

private:
    void resetSimulation();
};

ResetSimulationPlugin* findResetSimulation()
{
    return dynamic_cast<ResetSimulationPlugin*>(cnoid::PluginManager::instance()->findPlugin("ResetSimulation"));
}

#endif // __RESETSIMULATIONPLUGIN_H__
