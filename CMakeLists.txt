cmake_minimum_required(VERSION 3.10.2)
PROJECT(clean_robot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++14 -pthread")
set(thrid_party_path ${CMAKE_CURRENT_SOURCE_DIR}/thrid_party)
set(Glog_LIBS ${thrid_party_path}/glog/libglog.so)
set(Yaml_LIBS ${thrid_party_path}/yaml/libyaml-cpp.so.0.7.0)
set(Qp_LIBS ${thrid_party_path}/qp/libqp_event.so)
message(STATUS "Yaml_LIBS : ${Yaml_LIBS}")
message(STATUS "Glog_LIBS : ${Glog_LIBS}")
file(GLOB srcs
        activeThread/*.cc
        modules/sensor/*.cc
        common/*.cc
        modules/control/*.cc
        modules/room/*.cc
        activeApi/*.cc
        modules/virtualLineCheck/*.cc
        modules/navigation/jps/*.cc
        modules/navigation/*.cc
        modules/map/*.cc
        modules/chains/*.cc
        modules/cover/*.cc
        )
file(GLOB ros_suport_files
        ros/*.cc
        )
find_package(catkin REQUIRED COMPONENTS roscpp tf)
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLOG REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(activeThread thrid_party/qp thrid_party/event modules ./)

add_executable(${PROJECT_NAME}_node main_pc.cpp ${srcs} ${ros_suport_files})
target_link_libraries(${PROJECT_NAME}_node pthread ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Glog_LIBS} ${GFLAGS_LIBRARIES}  ${Qp_LIBS} yaml-cpp)
