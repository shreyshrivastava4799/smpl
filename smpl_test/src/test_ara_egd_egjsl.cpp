// standard includes
#include <chrono>
#include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/search/arastar.h>

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
    auto graph = smpl::ManipLatticeEGraph();

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

    if (!actions.Init(graph.GetJointSpaceLattice())) {
        SMPL_ERROR("Failed to initialize Manipulation Action Space");
        return 1;
    }

    auto mprim_filename = std::string();
    if (!ph.getParam("planning/mprim_filename", mprim_filename)) {
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

    if (!cost_fun.Init(graph.GetJointSpaceLattice())) {
        SMPL_ERROR("Failed to initialize Uniform Cost Function");
        return 1;
    }

    cost_fun.cost_per_action = 1000;

    std::string egraphs_directory;
    if (!ph.getParam("egraphs_directory", egraphs_directory)) {
        SMPL_ERROR("Failed to retrieve 'egraphs_directory' from the param server");
        return 1;
    }

    if (!graph.LoadExperienceGraph(egraphs_directory)) {
        SMPL_ERROR("Failed to load experience graph");
        return 1;
    }

    auto heuristic = smpl::DijkstraEGraphHeuristic3D();
    if (!heuristic.Init(&graph, &scenario.grid)) {
        SMPL_ERROR("Failed to initialize BFS Heuristic");
        return 1;
    }

    heuristic.SetInflationRadius(0.04);
    heuristic.SetEGraphWeight(5.0);
//    heuristic.SetCostPerCell(100);

    // TODO: this is kinda dumb to have to remember to do this
    // ...additionally, it would be nice if these names were uniform across
    // bfs and dijkstra variants?
    heuristic.SyncGridAndDijkstra();
    SV_SHOW_DEBUG(heuristic.GetWallsVisualization());

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

    auto time_params = smpl::ARAStar::TimeParameters();
    time_params.bounded = true;
    time_params.improve = false;
    time_params.type = smpl::ARAStar::TimeParameters::TIME;
//    time_params.type = smpl::ARAStar::TimeParameters::EXPANSIONS;
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
