// -*- mode: C++; coding: utf-8-unix; -*-

#ifndef __CHOREONOID_PLUGINS_UTIL_H__
#define __CHOREONOID_PLUGINS_UTIL_H__

#include <string>
#include <cnoid/RootItem>
#include <cnoid/SimulatorItem>

cnoid::SimulatorItem* findSimulatorItem(const std::string& simulator_name = "AISTSimulator")
{
    return dynamic_cast<cnoid::SimulatorItem*>(cnoid::RootItem::instance()->findItem(simulator_name));
}

#endif // __CHOREONOID_PLUGINS_UTIL_H__
