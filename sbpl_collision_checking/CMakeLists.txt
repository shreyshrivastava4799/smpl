cmake_minimum_required(VERSION 2.8)
project(sbpl_collision_checking)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        eigen_conversions
        geometric_shapes
        geometry_msgs
        leatherman
        moveit_core
        moveit_msgs
        octomap_msgs
        roscpp
        smpl_ros
        sensor_msgs
        shape_msgs
        urdf
        visualization_msgs)

find_package(octomap REQUIRED)
find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
        sbpl_collision_checking
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        geometry_msgs
        leatherman
        moveit_core
        moveit_msgs
        octomap_msgs
        roscpp
        smpl_ros
        sensor_msgs
        shape_msgs
        urdf
        visualization_msgs
    DEPENDS
        Boost
        OCTOMAP
        smpl)

add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${OCTOMAP_INCLUDE_DIRS})

add_library(
    sbpl_collision_checking
    src/attached_bodies_collision_model.cpp
    src/attached_bodies_collision_state.cpp
    src/base_collision_models.cpp
    src/base_collision_states.cpp
    src/collision_model_config.cpp
    src/collision_operations.cpp
    src/collision_space.cpp
    src/conversions.cpp
    src/robot_collision_model.cpp
    src/robot_motion_collision_model.cpp
    src/robot_collision_state.cpp
    src/self_collision_model.cpp
    src/shape_visualization.cpp
    src/types.cpp
    src/voxel_operations.cpp
    src/world_collision_detector.cpp
    src/world_collision_model.cpp)
target_link_libraries(sbpl_collision_checking ${catkin_LIBRARIES} smpl::smpl)

install(
    DIRECTORY include/sbpl_collision_checking/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS sbpl_collision_checking
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
