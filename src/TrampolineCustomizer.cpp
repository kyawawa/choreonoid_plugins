// // -*- mode: C++; coding: utf-8-unix; -*-

// /**
//  * @file   Customizer.cpp
//  * @brief  Trampoline spring / damping simulation
//  * @author Shin'ichiro Nakaoka
//  * @editor Tatsuya Ishikawa
//  */

#define CNOID_BODY_CUSTOMIZER
#ifdef CNOID_BODY_CUSTOMIZER
#include <cnoid/BodyCustomizerInterface>
#else
#include <BodyCustomizerInterface.h>
#endif

#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>
#include "yaml-cpp/yaml.h"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif /* Windows */

using namespace cnoid;

namespace {
BodyInterface* bodyInterface = 0;
BodyCustomizerInterface bodyCustomizerInterface;

struct Customizer
{
    double* pUpperJointPosition;
    double* pUpperJointVelocity;
    double* pUpperJointForce;
    double kp;
    double kd;
};

const char** getTargetModelNames()
{
    char *rname = getenv("CHOREONOID_ROBOT");
    if (rname != NULL) {
        std::cerr << "[TrampolineCustomizer] CHOREONID_ROBOT =" << rname << std::endl;
    }
    static const char* names[] = {"TrampolineSpringModel", rname, 0};
    return names;
}

BodyCustomizerHandle create(BodyHandle bodyHandle, const char* modelName)
{
    // Customizer* customizer = new Customizer();
    Customizer* customizer = 0;
    customizer = new Customizer;
    int upperJointIndex = bodyInterface->getLinkIndexFromName(bodyHandle, "UPPER");

    customizer->pUpperJointPosition = bodyInterface->getJointValuePtr(bodyHandle, upperJointIndex);
    customizer->pUpperJointVelocity = bodyInterface->getJointVelocityPtr(bodyHandle, upperJointIndex);
    customizer->pUpperJointForce = bodyInterface->getJointForcePtr(bodyHandle, upperJointIndex);
    customizer->kp = 2500;
    customizer->kd = 5.0;

    if (char* conf_file = std::getenv("CUSTOMIZER_CONF_PATH")) {
        std::ifstream ifs(conf_file);
        if (ifs.is_open()) {
            YAML::Node spring = YAML::LoadFile(conf_file);
            customizer->kp = spring["trampoline"]["kp"].as<double>();
            customizer->kd = spring["trampoline"]["kd"].as<double>();
            std::cerr << "[TrampolineCustomizer] kp is set to " << customizer->kp << " and kd is set to " << customizer->kd << std::endl;
        }
    }

    return static_cast<BodyCustomizerHandle>(customizer);
}

void destroy(BodyCustomizerHandle customizerHandle)
{
    Customizer* customizer = static_cast<Customizer*>(customizerHandle);
    if (customizer) {
        delete customizer;
    }
}

void setVirtualJointForces(BodyCustomizerHandle customizerHandle)
{
    Customizer* c = static_cast<Customizer*>(customizerHandle);
    *c->pUpperJointForce = -c->kp * *c->pUpperJointPosition - c->kd * *c->pUpperJointVelocity;
}
}

extern "C" DLL_EXPORT
cnoid::BodyCustomizerInterface* getHrpBodyCustomizerInterface(cnoid::BodyInterface* bodyInterface_)
{
    bodyInterface = bodyInterface_;

    bodyCustomizerInterface.version = cnoid::BODY_CUSTOMIZER_INTERFACE_VERSION;
    bodyCustomizerInterface.getTargetModelNames = getTargetModelNames;
    bodyCustomizerInterface.create = create;
    bodyCustomizerInterface.destroy = destroy;
    bodyCustomizerInterface.initializeAnalyticIk = NULL;
    bodyCustomizerInterface.calcAnalyticIk = NULL;
    bodyCustomizerInterface.setVirtualJointForces = setVirtualJointForces;

    return &bodyCustomizerInterface;
}
