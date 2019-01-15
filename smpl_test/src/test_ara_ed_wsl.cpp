// standard includes
#include <chrono>
#include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/search/arastar.h>

// project includes
#include "test_scenario.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ara_ed_wsl");
    ros::NodeHandle ph("~");

    auto scenario = TestScenario();
    InitTestScenario(&scenario);

    auto resolutions = smpl::WorkspaceProjectionParams();
    resolutions.res_x = 0.02;
    resolutions.res_y = 0.02;
    resolutions.res_z = 0.02;
    resolutions.R_count = 24;   // 15 degree
    resolutions.P_count = 13;   // 15 degree
    resolutions.Y_count = 24;   // 15 degree
    resolutions.free_angle_res = { smpl::to_radians(15.0) };

    auto actions = smpl::SimpleWorkspaceLatticeActionSpace();

    // TODO: implement cost functions for workspace lattice

    auto graph = smpl::WorkspaceLattice();

    if (!graph.Init(
            scenario.planning_model.get(),
            &scenario.collision_model,
            resolutions,
            &actions))
    {
        SMPL_ERROR("Failed to initialize Workspace Lattice");
        return 1;
    }

    // TODO: weird name, and inconsistent with Manip Lattice's ActionSpace
    if (!InitSimpleWorkspaceLatticeActions(&graph, &actions)) {
        SMPL_ERROR("Failed to initialize Simple Workspace Lattice Action Space");
        return 1;
    }

    // NOTE: SimpleWorkspaceLatticeActionSpace really isn't that simple. It
    // includes an adaptive ik motion that is automatically enabled
    actions.m_ik_amp_enabled = true;
    actions.m_ik_amp_thresh = 0.05;

    auto heuristic = smpl::EuclidDistHeuristic();
    if (!heuristic.Init(&graph)) {
        SMPL_ERROR("Failed to initialize Euclid Dist Heuristic");
        return 1;
    }

    auto* h = (smpl::Heuristic*)&heuristic;
    if (!graph.UpdateHeuristics(&h, 1)) {
        SMPL_ERROR("Failed to associate Euclid Dist Heuristic with Manip Lattice");
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

    auto goal = smpl::PoseGoal();
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

    double goal_vals[6];
    ph.param("goal/x",      goal_vals[0], 0.0);
    ph.param("goal/y",      goal_vals[1], 0.0);
    ph.param("goal/z",      goal_vals[2], 0.0);
    ph.param("goal/roll",   goal_vals[3], 0.0);
    ph.param("goal/pitch",  goal_vals[4], 0.0);
    ph.param("goal/yaw",    goal_vals[5], 0.0);

    goal.pose = smpl::MakeAffine(
            goal_vals[0], goal_vals[1], goal_vals[2],
            goal_vals[5], goal_vals[4], goal_vals[3]);
    goal.tolerance.xyz[0] = goal.tolerance.xyz[1] = goal.tolerance.xyz[2] = 0.015;
    goal.tolerance.rpy[0] = goal.tolerance.rpy[1] = goal.tolerance.rpy[2] = smpl::to_radians(3.0);

    SV_SHOW_INFO_NAMED("pose_goal", goal.GetVisualization("odom_combined"));

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

    SMPL_WARN("PLAN!");

    auto time_params = smpl::ARAStar::TimeParameters();
    time_params.bounded = true;
    time_params.improve = false;
    time_params.type = smpl::ARAStar::TimeParameters::TIME;
    // time_params.type = smpl::ARAStar::TimeParameters::EXPANSIONS;
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

    SMPL_INFO("Animate path");

    return AnimateSolution(&scenario, scenario.planning_model.get(), &path);
}
