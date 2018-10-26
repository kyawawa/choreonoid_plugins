cmake_minimum_required(VERSION 2.8.3)
project(choreonoid_plugins)

find_package(catkin REQUIRED COMPONENTS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include(FindPkgConfig)
pkg_check_modules(cnoid-plugin choreonoid-body-plugin)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS # TODO
  INCLUDE_DIRS # TODO
  LIBRARIES # TODO
)

if(${cnoid-plugin_FOUND})
  include_directories(${cnoid-plugin_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDEDIR})
  link_directories(${cnoid-plugin_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

  add_library(TrampolineCustomizer src/TrampolineCustomizer.cpp)
  target_link_libraries(TrampolineCustomizer ${cnoid-plugin_LIBRARIES} ${YAML_CPP_LIBRARIES})
  set_target_properties(TrampolineCustomizer PROPERTIES
    COMPILE_FLAG "-fPIC"
    PREFIX ""
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
