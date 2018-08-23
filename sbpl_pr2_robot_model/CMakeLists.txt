cmake_minimum_required(VERSION 2.8)
project(sbpl_pr2_robot_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        leatherman
        pr2_arm_kinematics
        roscpp
        sbpl_kdl_robot_model)

find_package(orocos_kdl REQUIRED)

find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_pr2_robot_model
    CATKIN_DEPENDS
        pr2_arm_kinematics
        sbpl_kdl_robot_model)

add_library(
    sbpl_pr2_robot_model
    src/sbpl_math.cpp
    src/orientation_solver.cpp
    src/pr2_kdl_robot_model.cpp
    src/ubr1_kdl_robot_model.cpp)
target_include_directories(sbpl_pr2_robot_model PUBLIC include)
target_include_directories(sbpl_pr2_robot_model SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(sbpl_pr2_robot_model SYSTEM PUBLIC ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(sbpl_pr2_robot_model ${catkin_LIBRARIES})
target_link_libraries(sbpl_pr2_robot_model ${orocos_kdl_LIBRARIES})
target_link_libraries(sbpl_pr2_robot_model smpl::smpl)
target_compile_options(sbpl_pr2_robot_model PUBLIC -std=c++11)

install(
    DIRECTORY include/sbpl_pr2_robot_model/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS sbpl_pr2_robot_model
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

