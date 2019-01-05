cmake_minimum_required(VERSION 2.8)
project(sbpl_kdl_robot_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(
    catkin
    REQUIRED
    COMPONENTS
        eigen_conversions
        kdl_parser
        leatherman
        roscpp
        smpl_urdf_robot_model
        urdf)

find_package(orocos_kdl REQUIRED)

find_package(smpl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_kdl_robot_model
    CATKIN_DEPENDS
        roscpp
        smpl_urdf_robot_model
    DEPENDS
        orocos_kdl
        smpl)

add_library(sbpl_kdl_robot_model src/kdl_robot_model.cpp)

target_include_directories(sbpl_kdl_robot_model PUBLIC include)
target_include_directories(
    sbpl_kdl_robot_model
    SYSTEM PUBLIC
        ${orocos_kdl_INCLUDE_DIRS}
        ${smpl_urdf_robot_model_INCLUDE_DIRS})
target_include_directories(
    sbpl_kdl_robot_model
    SYSTEM PRIVATE
        ${eigen_conversions_INCLUDE_DIRS}
        ${kdl_parser_INCLUDE_DIRS}
        ${leatherman_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${urdf_INCLUDE_DIRS})

target_link_libraries(
    sbpl_kdl_robot_model
    PUBLIC
        ${orocos_kdl_LIBRARIES}
        ${smpl_urdf_robot_model_LIBRARIES}
        smpl::smpl)
target_link_libraries(
    sbpl_kdl_robot_model
    PRIVATE
        ${eigen_conversions_LIBRARIES}
        ${kdl_parser_LIBRARIES}
        ${leatherman_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${urdf_LIBRARIES})

add_executable(test_kdl src/test_kdl_robot_model.cpp)
target_include_directories(
    test_kdl
    PRIVATE
        ${leatherman_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS})
target_link_libraries(
    test_kdl
    PRIVATE
        sbpl_kdl_robot_model
        ${leatherman_LIBRARIES}
        ${roscpp_LIBRARIES})

install(
    DIRECTORY include/sbpl_kdl_robot_model/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS sbpl_kdl_robot_model test_kdl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
