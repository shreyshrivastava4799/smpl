// standard includes
#include <chrono>
#include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/search/arastar.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h> // TODO: for visualization that should be part of JointStateGoal

// project includes
#include "test_scenario.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ara_bfs_jsl");
    ros::NodeHandle ph("~");

    auto scenario = TestScenario();
    InitTestScenario(&scenario);

    auto resolutions = std::vector<double>(
            scenario.planning_model->jointVariableCount(),
            smpl::to_radians(1.0));

    auto actions = smpl::ManipulationActionSpace();
    auto cost_fun = smpl::UniformCostFunction();
    auto graph = smpl::ManipLattice();

    if (!graph.Init(
            scenario.planning_model.get(),
            &scenario.collision_model,
            resolutions,
            &actions,
            &cost_fun))
    {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    if (!actions.Init(&graph)) {
        SMPL_ERROR("Failed to initialize Manipulation Action Space");
        return 1;
    }

    auto mprim_filename = std::string();
    if (!ph.getParam("graph/mprim_filename", mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return 1;
    }

    if (!actions.Load(mprim_filename)) {
        SMPL_ERROR("Failed to load motion primitives");
        return 1;
    }

    actions.EnableLongMotions(true);
    actions.EnableIKMotionXYZ(false);
    actions.EnableIKMotionRPY(false);
    actions.EnableIKMotionXYZRPY(true);

    actions.SetLongMotionThreshold(0.4);
    actions.SetIKMotionXYZRPYThreshold(0.10); // 0.2 in call_planner

    if (!cost_fun.Init(&graph)) {
        SMPL_ERROR("Failed to initialize Uniform Cost Function");
        return 1;
    }

    cost_fun.cost_per_action = 1000;

    auto heuristic = smpl::JointDistHeuristic();
    if (!heuristic.Init(&graph)) {
        SMPL_ERROR("Failed to initialize BFS Heuristic");
        return 1;
    }

    auto* h = (smpl::Heuristic*)&heuristic;
    if (!graph.UpdateHeuristics(&h, 1)) {
        SMPL_ERROR("Failed to associate BFS Heuristic with Manip Lattice");
        return 1;
    }

    auto search = smpl::ARAStar();
    if (!search.Init(&graph, &heuristic)) {
        SMPL_ERROR("Failed to initialize ARA*");
        return 1;
    }

    search.SetInitialEps(100.0);
    search.SetSearchMode(false);
    search.SetAllowPartialSolutions(false);
    search.SetTargetEpsilon(1.0);
    search.SetDeltaEpsilon(1.0);
    search.SetImproveSolution(false);
    search.SetBoundExpansions(true);

    auto goal = smpl::JointStateGoal();
    if (!goal.Init(&graph)) {
        SMPL_ERROR("Failed to initialize the goal");
        return 1;
    }

    /////////////////////////
    // Set the start state //
    /////////////////////////

    auto start_state = smpl::RobotState();
    auto success = false;
    std::tie(start_state, success) = MakeRobotState(
            &scenario.start_state, scenario.planning_model.get());
    if (!success) return 1;

    auto start_state_id = graph.GetStateID(start_state);

    if (!graph.UpdateStart(start_state_id) ||
        !heuristic.UpdateStart(start_state_id) ||
        !search.UpdateStart(start_state_id))
    {
        SMPL_ERROR("Failed to update the start state");
        return 1;
    }

    /////////////////////
    // Update the goal //
    /////////////////////

    // right arm tuck pose
    double goal_state[] =
    {
        /* r_shoulder_pan_joint */ -0.023593,
        /* r_shoulder_lift_joint */ 1.10728,
        /* r_upper_arm_roll_joint */ -1.55669,
        /* r_forearm_roll_joint */ -1.4175,
        /* r_elbow_flex_joint */ -2.12441,
        /* r_wrist_flex_joint */ -1.8417,
        /* r_wrist_roll_joint */ 0.21436,
    };

    // left arm tuck pose
    // -1.68339, -1.73434, 1.24853, 0.06024, 1.78907, -0.0962141, -0.0864407,

    auto smpl_goal_state = std::vector<double>(
            goal_state,
            goal_state + sizeof(goal_state) / sizeof(goal_state[0]));
    goal.SetGoalState(smpl_goal_state);

    auto goal_tolerance =
    {
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
        smpl::to_radians(1.0),
    };

    goal.SetGoalTolerance(goal_tolerance);

    auto full_state_vis = scenario.planning_model->GetVisualization(smpl_goal_state);
    auto id = (int32_t)0;
    for (auto& marker : full_state_vis) {
        marker.frame_id = "odom_combined";
        marker.ns = "joint_goal";
        marker.color = smpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
        marker.id = id++;
    }

    SV_SHOW_INFO_NAMED("joint_goal", full_state_vis);

    if (!graph.UpdateGoal(&goal) ||
        !heuristic.UpdateGoal(&goal) ||
        !search.UpdateGoal(&goal))
    {
        SMPL_ERROR("Failed to update the goal");
        return 1;
    }

    ////////////////////////////
    // Finally, plan the path //
    ////////////////////////////

    auto time_params = smpl::TimeoutCondition();
    time_params.bounded = true;
    time_params.improve = false;
    time_params.type = smpl::TimeoutCondition::TIME;
//    time_params.type = smpl::TimeoutCondition::EXPANSIONS;
    time_params.max_expansions_init = 200000;
    time_params.max_expansions = 2000;
    time_params.max_allowed_time_init = std::chrono::seconds(30);
    time_params.max_allowed_time = std::chrono::seconds(1);
    auto solution = std::vector<int>();
    auto cost = 0;
    auto found = (bool)search.Replan(time_params, &solution, &cost);

    if (!found) {
        SMPL_ERROR("Failed to find path after %d expansions in %f seconds", search.GetNumExpansions(), search.GetElapsedTime());
        return 1;
    }

    SMPL_INFO("Found path after %d expansions in %f seconds", search.GetNumExpansions(), search.GetElapsedTime());

    auto path = std::vector<smpl::RobotState>();
    if (!graph.ExtractPath(solution, path)) {
        SMPL_ERROR("Failed to extract path");
        return 1;
    }

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    return AnimateSolution(&scenario, scenario.planning_model.get(), &path);
}
