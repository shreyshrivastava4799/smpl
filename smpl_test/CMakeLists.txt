cmake_minimum_required(VERSION 2.8)
project(smpl_test)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

find_package(Eigen3 REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        smpl_ros
        eigen_conversions
        kdl_conversions
        octomap_msgs
#        geometry_msgs
        leatherman
        smpl_ros
#        moveit_msgs
#        moveit_ros_planning
        roscpp
        sbpl_collision_checking
        sbpl_kdl_robot_model
        sbpl_pr2_robot_model
        smpl_ompl_interface
        visualization_msgs)

find_package(OMPL REQUIRED)
find_package(smpl REQUIRED)
find_package(urdfdom REQUIRED)

catkin_package()

add_definitions(-DSV_PACKAGE_NAME="smpl_test")

###################
# smpl-only tests #
###################

add_executable(visualizer_test src/visualizer_test.cpp)
target_link_libraries(visualizer_test PRIVATE smpl::smpl)

add_executable(csv_parser_test src/csv_parser_test.cpp)
target_link_libraries(csv_parser_test PRIVATE smpl::smpl)

add_executable(xytheta src/xytheta.cpp)
target_link_libraries(xytheta PRIVATE smpl::smpl)

add_executable(distance_map_test src/distance_map_test.cpp)
target_link_libraries(distance_map_test PRIVATE smpl::smpl)

install(
    TARGETS visualizer_test csv_parser_test xytheta distance_map_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############################
# smpl-and-boost-only tests #
#############################

add_executable(heap_test src/heap_test.cpp)
target_include_directories(heap_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(heap_test PRIVATE ${Boost_LIBRARIES} smpl::smpl)

add_executable(egraph_test src/egraph_test.cpp)
target_include_directories(egraph_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(egraph_test PRIVATE ${Boost_LIBRARIES} smpl::smpl)

add_executable(octree_test src/octree_tests.cpp)
target_include_directories(octree_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(octree_test PRIVATE ${Boost_LIBRARIES} smpl::smpl)

add_executable(sparse_grid_test src/sparse_grid_test.cpp)
target_link_libraries(sparse_grid_test ${Boost_LIBRARIES} smpl::smpl)

add_executable(sparse_binary_grid_test src/sparse_binary_grid_test.cpp)
target_include_directories(
        sparse_binary_grid_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(
        sparse_binary_grid_test PRIVATE ${Boost_LIBRARIES} smpl::smpl)

install(
    TARGETS heap_test egraph_test octree_test sparse_grid_test sparse_binary_grid_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

########################
# catkin package tests #
########################

add_executable(debug_vis_demo src/debug_vis_demo.cpp)
target_include_directories(
    debug_vis_demo SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS} ${smpl_ros_INCLUDE_DIRS})
target_link_libraries(
    debug_vis_demo PRIVATE ${roscpp_LIBRARIES} ${smpl_ros_LIBRARIES} smpl::smpl)

#add_executable(callPlanner src/call_planner.cpp src/collision_space_scene.cpp)
#target_include_directories(
#    callPlanner
#    SYSTEM PRIVATE
#        ${eigen_conversions_INCLUDE_DIRS}
#        ${leatherman_INCLUDE_DIRS}
#        ${moveit_msgs_INCLUDE_DIRS}
#        ${octomap_msgs_INCLUDE_DIRS}
#        ${roscpp_INCLUDE_DIRS}
#        ${kdl_conversions_INCLUDE_DIRS}
#        ${sbpl_collision_checking_INCLUDE_DIRS}
#        ${sbpl_kdl_robot_model_INCLUDE_DIRS}
#        ${smpl_ros_INCLUDE_DIRS}
#        ${visualization_msgs_INCLUDE_DIRS})
#target_link_libraries(
#    callPlanner
#    PRIVATE
#        ${eigen_conversions_LIBRARIES}
#        ${leatherman_LIBRARIES}
#        ${moveit_msgs_LIBRARIES}
#        ${octomap_msgs_LIBRARIES}
#        ${roscpp_LIBRARIES}
#        ${kdl_conversions_LIBRARIES}
#        ${sbpl_collision_checking_LIBRARIES}
#        ${sbpl_kdl_robot_model_LIBRARIES}
#        ${smpl_ros_LIBRARIES}
#        ${visualization_msgs_LIBRARIES}
#        smpl::smpl)

add_library(test_scenario src/test_scenario.cpp src/collision_space_scene.cpp)
target_compile_options(test_scenario PRIVATE -Wfatal-errors)
target_include_directories(
    test_scenario
    SYSTEM PUBLIC
        ${sbpl_pr2_robot_model_INCLUDE_DIRS}
        ${moveit_msgs_INCLUDE_DIRS}
        ${eigen_conversions_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sbpl_collision_checking_INCLUDE_DIRS}
        ${sbpl_kdl_robot_model_INCLUDE_DIRS}
        ${smpl_ros_INCLUDE_DIRS})
target_link_libraries(
    test_scenario
    PUBLIC
    ${sbpl_pr2_robot_model_LIBRARIES}
    ${moveit_msgs_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${sbpl_collision_checking_LIBRARIES}
    ${sbpl_kdl_robot_model_LIBRARIES}
    smpl::smpl
    ${smpl_ros_LIBRARIES})

add_executable(test_ara_bfs_jsl src/test_ara_bfs_jsl.cpp)
target_include_directories(test_ara_bfs_jsl SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
target_link_libraries(test_ara_bfs_jsl PRIVATE ${roscpp_LIBRARIES} smpl::smpl test_scenario)

add_executable(test_ara_bfs_wsl src/test_ara_bfs_wsl.cpp)
target_include_directories(test_ara_bfs_wsl SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
target_link_libraries(test_ara_bfs_wsl PRIVATE ${roscpp_LIBRARIES} smpl::smpl test_scenario)

add_executable(call_ompl_planner src/call_ompl_planner.cpp src/collision_space_scene.cpp)
target_include_directories(
    call_ompl_planner
    SYSTEM PRIVATE
        ${OMPL_INCLUDE_DIRS}
        ${eigen_conversions_INCLUDE_DIRS}
        ${kdl_conversions_INCLUDE_DIRS}
        ${leatherman_INCLUDE_DIRS}
        ${octomap_msgs_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sbpl_collision_checking_INCLUDE_DIRS}
        ${sbpl_kdl_robot_model_INCLUDE_DIRS}
        ${smpl_ros_INCLUDE_DIRS}
        ${smpl_ompl_interface_INCLUDE_DIRS}
        ${urdfdom_INCLUDE_DIRS})
target_link_libraries(
    call_ompl_planner
        ${OMPL_LIBRARIES}
        ${eigen_conversions_LIBRARIES}
        ${kdl_conversions_LIBRARIES}
        ${leatherman_LIBRARIES}
        ${octomap_msgs_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${sbpl_collision_checking_LIBRARIES}
        ${smpl_ompl_interface_LIBRARIES}
        ${sbpl_kdl_robot_model_LIBRARIES}
        ${smpl_ros_LIBRARIES}
        ${urdfdom_LIBRARIES}
        smpl::smpl)

add_executable(occupancy_grid_test src/occupancy_grid_test.cpp)
target_include_directories(
    occupancy_grid_test
    SYSTEM PRIVATE
        ${roscpp_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        ${smpl_ros_INCLUDE_DIRS})
target_link_libraries(
    occupancy_grid_test PRIVATE
        ${roscpp_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        ${smpl_ros_LIBRARIES}
        smpl::smpl)

install(
    TARGETS debug_vis_demo call_ompl_planner occupancy_grid_test #callPlanner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
