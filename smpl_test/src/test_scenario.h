#ifndef SMPL_TEST_TEST_SCENARIO_H
#define SMPL_TEST_TEST_SCENARIO_H

// system includes
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/occupancy_grid.h>
#include <smpl_ros/debug/visualizer_ros.h>

// project includes
#include "collision_space_scene.h"

// A common test scenario composed of:
// (1) A scene (list of objects in an environment)
// (2) A robot (kinematic object in the scene that we're planning for)
// (3) An initial state for the robot
// (4) A planning model (description of the space we're planning in)
// (5) A planning-model-oriented collision detector
struct TestScenario
{
    ros::NodeHandle nh;
    ros::NodeHandle ph;

    smpl::VisualizerROS visualizer;

    CollisionSpaceScene scene;

    moveit_msgs::RobotState start_state;

    smpl::KDLRobotModel planning_model;

    smpl::OccupancyGrid grid;
    smpl::collision::CollisionSpace collision_model;

    TestScenario();
};

bool InitTestScenario(TestScenario* test);

#endif
