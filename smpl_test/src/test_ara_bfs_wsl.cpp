// standard includes
#include <chrono>
#include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/search/arastar.h>

// project includes
#include "test_scenario.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ara_bfs_wsl");
    ros::NodeHandle ph("~");

    auto scenario = TestScenario();
    InitTestScenario(&scenario);

    // some cool factors of 360. choose one of these for roll and yaw
    // m degrees/disc = n discrete values
    // 1    value  = 360   degrees
    // 2    values = 180   degrees
    // 3    values = 120   degrees
    // 4    values = 90    degrees
    // 5    values = 72    degrees
    // 6    values = 60    degrees
    // 8    values = 45    degrees
    // 9    values = 40    degrees
    // 10   values = 36    degrees
    // 12   values = 30    degrees
    // 15   values = 24    degrees
    // 18   values = 20    degrees
    // 20   values = 18    degrees
    // 24   values = 15    degrees
    // 30   values = 12    degrees
    // 36   values = 10    degrees
    // 40   values = 9     degrees
    // 45   values = 8     degrees
    // 60   values = 6     degrees
    // 72   values = 5     degrees
    // 90   values = 4     degrees
    // 120  values = 3     degrees
    // 180  values = 2     degrees
    // 360  values = 1     degree

    // some cool factors of 180. choose one of these for pitch and add 1
    // 1    value  = 180   degrees
    // 2    values = 90    degrees
    // 3    values = 60    degrees
    // 4    values = 45    degrees
    // 5    values = 36    degrees
    // 6    values = 30    degrees
    // 9    values = 20    degrees
    // 10   values = 18    degrees
    // 12   values = 30    degrees
    // 15   values = 15    degrees
    // 18   values = 10    degrees
    // 20   values = 9     degrees
    // 30   values = 6     degrees
    // 36   values = 5     degrees
    // 45   values = 4     degrees
    // 60   values = 3     degrees
    // 90   values = 2     degrees
    // 180  values = 1     degrees

    // TODO: initialize and configure
    auto resolutions = smpl::WorkspaceProjectionParams();
    resolutions.res_x = 0.02;
    resolutions.res_y = 0.02;
    resolutions.res_z = 0.02;
#if 0
    // 30 degree
    resolutions.R_count = 12;
    resolutions.P_count = 7;
    resolutions.Y_count = 12;
    resolutions.free_angle_res = { smpl::to_radians(30.0) };
#elif 1
    // 15 degree
    resolutions.R_count = 24;
    resolutions.P_count = 13;
    resolutions.Y_count = 24;
    resolutions.free_angle_res = { smpl::to_radians(15.0) };
#elif 0
    // 10 degree
    resolutions.R_count = 36;
    resolutions.P_count = 19;
    resolutions.Y_count = 36;
    resolutions.free_angle_res = { smpl::to_radians(10.0) };
#else
    // 5 degree
    resolutions.R_count = 72;
    resolutions.P_count = 37;
    resolutions.Y_count = 72;
    resolutions.free_angle_res = { smpl::to_radians(5.0) };
#endif

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

    auto heuristic = smpl::BFSHeuristic();
    if (!heuristic.Init(&graph, &scenario.grid)) {
        SMPL_ERROR("Failed to initialize BFS Heuristic");
        return 1;
    }

    heuristic.SetInflationRadius(0.04);
    heuristic.SetCostPerCell(100);

    // TODO: this is kinda dumb to have to remember to do this
    heuristic.SyncGridAndBFS();
    SV_SHOW_DEBUG(heuristic.GetWallsVisualization());

    auto* h = (smpl::Heuristic*)&heuristic;
    if (!graph.UpdateHeuristics(&h, 1)) {
        SMPL_ERROR("Failed to associate BFS Heuristic with Workspace Lattice");
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
    // similar to call_planner tolerance of (0.015, 0.05)
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

    SV_SHOW_DEBUG_NAMED("bfs_values", heuristic.GetValuesVisualization());

    ////////////////////////////
    // Finally, plan the path //
    ////////////////////////////

    SMPL_WARN("PLAN!");

    auto time_params = smpl::TimeoutCondition();
    time_params.bounded = true;
    time_params.improve = false;
    time_params.type = smpl::TimeoutCondition::TIME;
    // time_params.type = smpl::TimeoutCondition::EXPANSIONS;
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
