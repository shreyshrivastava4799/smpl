cmake_minimum_required(VERSION 2.8.12)
project(smpl_ros_interface)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(
    catkin
    REQUIRED
    COMPONENTS
        moveit_msgs eigen_conversions leatherman trajectory_msgs)

find_package(smpl REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES smpl_ros_interface
    CATKIN_DEPENDS moveit_msgs
    DEPENDS smpl)

add_library(smpl_ros_interface src/factories.cpp src/planner_interface.cpp)
target_compile_definitions(
    smpl_ros_interface
    PRIVATE
        -DSV_PACKAGE_NAME="smpl_ros_interface")
target_include_directories(
    smpl_ros_interface
    SYSTEM PUBLIC
        include
        ${moveit_msgs_INCLUDE_DIRS})
target_include_directories(
    smpl_ros_interface
    SYSTEM PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${eigen_conversions_INCLUDE_DIRS}
        ${leatherman_INCLUDE_DIRS}
        ${trajectory_msgs_INCLUDE_DIRS})
target_link_libraries(
    smpl_ros_interface
    PUBLIC
        ${moveit_msgs_LIBRARIES} smpl::smpl)
target_link_libraries(
    smpl_ros_interface
    PRIVATE
        ${eigen_conversions_LIBRARIES}
        ${leatherman_LIBRARIES}
        ${trajectory_msgs_LIBRARIES})

install(
    DIRECTORY include/smpl_ros_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS smpl_ros_interface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
