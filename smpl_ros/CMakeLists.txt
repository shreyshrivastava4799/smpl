cmake_minimum_required(VERSION 2.8.12)
project(smpl_ros)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(
    catkin
    REQUIRED
    COMPONENTS
        roscpp
        moveit_core
        visualization_msgs)

find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
        smpl_ros
    CATKIN_DEPENDS moveit_core roscpp visualization_msgs
    DEPENDS Boost smpl)

add_definitions(-DSV_PACKAGE_NAME="smpl_ros")

add_library(
    smpl_ros
    src/debug/visualizer_ros.cpp
    src/debug/marker_conversions.cpp
    src/ros/propagation_distance_field.cpp)
target_include_directories(
    smpl_ros
    SYSTEM PUBLIC
        include
        ${roscpp_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})
target_link_libraries(
    smpl_ros
    PUBLIC
        ${roscpp_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        smpl::smpl)

install(
    DIRECTORY include/smpl_ros/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS smpl_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
