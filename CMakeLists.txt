cmake_minimum_required(VERSION 2.8.3)
project(choreonoid_plugins)

find_package(catkin COMPONENTS
  hrpsys_choreonoid_tutorials
  hrpsys_ros_bridge_tutorials
  jvrc_models
  jsk_models
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include(FindPkgConfig)
pkg_check_modules(cnoid-plugin REQUIRED choreonoid-body-plugin)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
pkg_check_modules(NLOPT REQUIRED nlopt)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS # TODO
  LIBRARIES # TODO
)

include_directories(${cnoid-plugin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR} ${NLOPT_INCLUDE_DIRS})
link_directories(${cnoid-plugin_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS} ${NLOPT_LIBRARY_DIRS})

### nlopt ###
set(target NloptTutorial)
add_executable(${target} src/tutorial.cpp)
target_link_libraries(${target} ${NLOPT_LIBRARIES})

### Customizer ###
set(target CnoidTrampolineCustomizer)
add_library(${target} customizers/TrampolineCustomizer.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES} ${YAML_CPP_LIBRARIES})
set_target_properties(${target} PROPERTIES
  COMPILE_FLAG "-fPIC"
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

### Controller ###
set(target CnoidPDController)
add_library(${target} SHARED controllers/PDController.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES})
set_target_properties(${target} PROPERTIES
  COMPILE_FLAG "-fPIC"
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

set(target CnoidSpringFootController)
add_library(${target} SHARED controllers/SpringFootController.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES})
set_target_properties(${target} PROPERTIES
  COMPILE_FLAG "-fPIC"
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

set(target CnoidPDController_InvertedPendulum)
add_library(${target} SHARED controllers/PDController_InvertedPendulum.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES})
set_target_properties(${target} PROPERTIES
  COMPILE_FLAG "-fPIC"
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

### Plugin ###
set(target CnoidResetSimulationPlugin)
add_library(${target} SHARED plugins/ResetSimulationPlugin.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES})
set_target_properties(${target} PROPERTIES
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

set(target CnoidCloneItemPlugin)
add_library(${target} SHARED plugins/CloneItemPlugin.cpp)
target_link_libraries(${target} ${cnoid-plugin_LIBRARIES})
set_target_properties(${target} PROPERTIES
  PREFIX "lib"
  SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

### cnoid ###
configure_file(${PROJECT_SOURCE_DIR}/world/SampleRobotTrampoline.cnoid.in ${PROJECT_SOURCE_DIR}/world/SampleRobotTrampoline.cnoid @ONLY)

### JAXON ###
if(${hrpsys_choreonoid_tutorials_FOUND} AND ${jvrc_models_FOUND})
  set(JVRC_MODELS_DIR ${jvrc_models_SOURCE_PREFIX})
  set(PDGAINS_YAML_FILE ${hrpsys_choreonoid_tutorials_SOURCE_PREFIX}/models/JAXON_JVRC.PDgains_sim.dat)
  set(TORQUE_LIMITS_FILE ${PROJECT_SOURCE_DIR}/config/JAXONTorqueLimits.dat)
  configure_file(${PROJECT_SOURCE_DIR}/world/JAXONFloor.cnoid.in ${PROJECT_SOURCE_DIR}/world/JAXONFloor.cnoid @ONLY)
  configure_file(${PROJECT_SOURCE_DIR}/world/JAXONTrampoline.cnoid.in ${PROJECT_SOURCE_DIR}/world/JAXONTrampoline.cnoid @ONLY)
else()
  message(WARNING "Cound not find hrpsys_choreonoid_tutorials and jvrc_models")
endif()

### CHIDORI ###
if (${jsk_models_FOUND})
  file(READ models/LLEG_SPRING_FOOT.wrl LLEG_SPRING_FOOT)
  file(READ models/RLEG_SPRING_FOOT.wrl RLEG_SPRING_FOOT)
  set(CHIDORI_MODEL_DIR ${jsk_models_SOURCE_PREFIX}/CHIDORI)
  configure_file(${PROJECT_SOURCE_DIR}/models/CHIDORImain_springfoot.wrl.in ${PROJECT_SOURCE_DIR}/models/CHIDORImain_springfoot.wrl @ONLY)

  set(PDGAINS_YAML_FILE ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/models/CHIDORI.PDgains_sim.dat)
  set(TORQUE_LIMITS_FILE ${PROJECT_SOURCE_DIR}/config/CHIDORITorqueLimits.dat)
  set(SPRING_CONF_FILE ${PROJECT_SOURCE_DIR}/config/FOOT_SPRING.dat)
  configure_file(${PROJECT_SOURCE_DIR}/world/CHIDORIFloor.cnoid.in ${PROJECT_SOURCE_DIR}/world/CHIDORIFloor.cnoid @ONLY)
else()
  message(WARNING "Cound not find jsk_models")
endif()
