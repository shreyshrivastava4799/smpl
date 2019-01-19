// standard includes
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/search/search.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/console/console.h>

// project includes
#include "test_scenario.h"
#include "factories.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_planner");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    /////////////////////////////
    // Initialize the scenario //
    /////////////////////////////

    auto scenario = TestScenario();
    if (!InitTestScenario(&scenario, nh, ph)) {
        SMPL_ERROR("Failed to initialize test scenario");
        return 1;
    }

    //////////////////////////
    // Initialize the graph //
    //////////////////////////

    auto graph_nh = ros::NodeHandle(ph, "graph");
    auto graph_type = std::string();
    if (!graph_nh.getParam("type", graph_type)) {
        return 1;
    }
    SMPL_INFO("Graph type: %s", graph_type.c_str());

    auto graph = std::unique_ptr<smpl::DiscreteSpace>();
    if (graph_type == "manip_lattice") {
        graph = MakeManipLattice(
                scenario.planning_model.get(),
                &scenario.collision_model,
                graph_nh);
    } else if (graph_type == "workspace_lattice") {
        graph = MakeWorkspaceLattice(
                scenario.planning_model.get(),
                &scenario.collision_model,
                graph_nh);
    } else if (graph_type == "manip_lattice_egraph") {
        graph = MakeManipLatticeEGraph(
                scenario.planning_model.get(),
                &scenario.collision_model,
                graph_nh);
    } else {
        SMPL_ERROR("Unrecognized graph type '%s'", graph_type.c_str());
        return 1;
    }
    if (graph == NULL) {
        SMPL_ERROR("Failed to create graph");
        return 1;
    }

    //////////////////////////////
    // Initialize the Heuristic //
    //////////////////////////////

    auto heuristic_nh = ros::NodeHandle(ph, "heuristic");
    auto heuristic_type = std::string();
    if (!heuristic_nh.getParam("type", heuristic_type)) {
        return 1;
    }
    SMPL_INFO("Heuristic type: %s", heuristic_type.c_str());

    auto heuristic = std::unique_ptr<smpl::Heuristic>();
    if (heuristic_type == "bfs") {
        heuristic = MakeBFSHeuristic(graph.get(), heuristic_nh);
    } else if (heuristic_type == "euclid_dist") {
        heuristic = MakeEuclidDistHeuristic(graph.get(), heuristic_nh);
    } else if (heuristic_type == "joint_dist") {
        heuristic = MakeJointDistHeuristic(graph.get(), heuristic_nh);
    } else if (heuristic_type == "dijkstra_egraph_3d") {
        heuristic = MakeDijkstraEGraph3DHeuristic(
                graph.get(), heuristic_nh);
    } else if (heuristic_type == "multi_frame_bfs") {
        heuristic = MakeMultiFrameBFSHeuristic(graph.get(), heuristic_nh);
    } else {
        SMPL_ERROR("Unrecognized heuristic type '%s'", heuristic_type.c_str());
        return 1;
    }
    if (heuristic == NULL) {
        SMPL_ERROR("Failed to create heuristic");
        return 1;
    }

    /////////////////////////////////////
    // Associate Heuristics with Graph //
    /////////////////////////////////////

    auto* h = heuristic.get();
    if (!graph->UpdateHeuristics(&h, 1)) {
        SMPL_ERROR("Failed to associate heuristic with graph");
        return 1;
    }

    ///////////////////////////
    // Initialize the Search //
    ///////////////////////////

    auto search_nh = ros::NodeHandle(ph, "search");
    auto search_type = std::string();
    if (!search_nh.getParam("type", search_type)) {
        return 1;
    }
    SMPL_INFO("Search type: %s", search_type.c_str());

    auto search = std::unique_ptr<smpl::Search>();
    if (search_type == "arastar") {
        search = MakeARAStar(graph.get(), heuristic.get(), search_nh);
    } else if (search_type == "smhastar") {
    } else {
        SMPL_ERROR("Unrecognized search type '%s'", search_type.c_str());
        return 1;
    }
    if (search == NULL) {
        SMPL_ERROR("Failed to create search");
        return 1;
    }

    /////////////////////////
    // Set the start state //
    /////////////////////////

    auto* rps = graph->GetExtension<smpl::RobotPlanningSpace>();
    if (rps == NULL) {
        SMPL_ERROR("Discrete Space must support RobotPlanningSpace Interface");
        return 1;
    }

    auto start_state = smpl::RobotState();
    auto success = false;
    std::tie(start_state, success) = MakeRobotState(
            &scenario.start_state, scenario.planning_model.get());
    if (!success) return 1;

    auto start_state_id = rps->GetStateID(start_state);

    if (!graph->UpdateStart(start_state_id) ||
        !heuristic->UpdateStart(start_state_id) ||
        !search->UpdateStart(start_state_id))
    {
        SMPL_ERROR("Failed to update the start state");
        return 1;
    }

    /////////////////////
    // Update the goal //
    /////////////////////

    auto goal_nh = ros::NodeHandle(ph, "goal");
    auto goal_type = std::string();
    if (!goal_nh.getParam("type", goal_type)) {
        return 1;
    }
    SMPL_INFO("Goal type: %s", goal_type.c_str());

    auto goal = std::unique_ptr<smpl::GoalConstraint>();
    if (goal_type == "pose") {
        goal = MakePoseGoal(graph.get(), goal_nh);
    } else if (goal_type == "joint_state") {
        goal = MakeJointStateGoal(graph.get(), goal_nh);
    } else {
        SMPL_ERROR("Unrecognized goal type '%s'", goal_type.c_str());
        return 1;
    }

    if (goal == NULL) {
        SMPL_ERROR("Failed to construct goal");
        return 1;
    }

    if (!graph->UpdateGoal(goal.get()) ||
        !heuristic->UpdateGoal(goal.get()) ||
        !search->UpdateGoal(goal.get()))
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
    auto found = (bool)search->Replan(time_params, &solution, &cost);

    if (!found) {
        SMPL_ERROR("Failed to find path after %d expansions in %f seconds", search->GetNumExpansions(), search->GetElapsedTime());
        return 1;
    }

    SMPL_INFO("Found path after %d expansions in %f seconds", search->GetNumExpansions(), search->GetElapsedTime());

    auto path = std::vector<smpl::RobotState>();
    if (!rps->ExtractPath(solution, path)) {
        SMPL_ERROR("Failed to extract path");
        return 1;
    }

#if 0
    WritePathCSV(&scenario.planning_model, &path, "trajectory.csv");
#endif

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    return AnimateSolution(&scenario, scenario.planning_model.get(), &path);
}
