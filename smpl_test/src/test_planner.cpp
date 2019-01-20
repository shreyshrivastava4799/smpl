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

auto MakeHeuristic(
    const std::string& type,
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto heuristic = std::unique_ptr<smpl::Heuristic>();
    if (type == "bfs") {
        heuristic = MakeBFSHeuristic(graph, nh);
    } else if (type == "euclid_dist") {
        heuristic = MakeEuclidDistHeuristic(graph, nh);
    } else if (type == "joint_dist") {
        heuristic = MakeJointDistHeuristic(graph, nh);
    } else if (type == "dijkstra_egraph_3d") {
        heuristic = MakeDijkstraEGraph3DHeuristic(graph, nh);
    } else if (type == "multi_frame_bfs") {
        heuristic = MakeMultiFrameBFSHeuristic(graph, nh);
    } else {
        SMPL_ERROR("Unrecognized heuristic type '%s'", type.c_str());
    }
    return heuristic;
}

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

    auto num_heuristics = 0;
    ph.getParam("num_heuristics", num_heuristics);

    auto heuristics = std::vector<std::unique_ptr<smpl::Heuristic>>();
    if (num_heuristics == 0) {
        auto heuristic_nh = ros::NodeHandle(ph, "heuristic");
        auto heuristic_type = std::string();
        if (!heuristic_nh.getParam("type", heuristic_type)) {
            return 1;
        }
        SMPL_INFO("Heuristic type: %s", heuristic_type.c_str());

        auto heuristic = MakeHeuristic(heuristic_type, graph.get(), heuristic_nh);
        if (heuristic == NULL) {
            SMPL_ERROR("Failed to create heuristic");
            return 1;
        }
        heuristics.push_back(std::move(heuristic));
    } else {
        SMPL_INFO("Create %d heuristics", num_heuristics);
        for (auto i = 0; i < num_heuristics; ++i) {
            auto heuristic_nh = ros::NodeHandle(ph, "heuristic_" + std::to_string(i));
            auto heuristic_type = std::string();
            if (!heuristic_nh.getParam("type", heuristic_type)) {
                return 1;
            }
            SMPL_INFO("Heuristic type: %s", heuristic_type.c_str());

            auto heuristic = MakeHeuristic(heuristic_type, graph.get(), heuristic_nh);
            if (heuristic == NULL) {
                SMPL_ERROR("Failed to create heuristic");
                return 1;
            }
            heuristics.push_back(std::move(heuristic));
        }
    }

    /////////////////////////////////////
    // Associate Heuristics with Graph //
    /////////////////////////////////////

    auto p_heuristics = std::vector<smpl::Heuristic*>();
    for (auto& h : heuristics) {
        p_heuristics.push_back(h.get());
    }
    if (!graph->UpdateHeuristics(p_heuristics.data(), (int)p_heuristics.size())) {
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

    auto* h_first = p_heuristics[0];

    auto search = std::unique_ptr<smpl::Search>();
    if (search_type == "arastar") {
        search = MakeARAStar(graph.get(), h_first, search_nh);
    } else if (search_type == "smhastar") {
        search = MakeSMHAStar(
                graph.get(),
                p_heuristics[0],
                &p_heuristics[1],
                (int)p_heuristics.size() - 1,
                search_nh);
    } else if (search_type == "fmhastar") {
        search = MakeFMHAStar(
                graph.get(),
                p_heuristics[0],
                &p_heuristics[1],
                (int)p_heuristics.size() - 1,
                search_nh);
    } else if (search_type == "umhastar") {
        search = MakeUMHAStar(
                graph.get(),
                p_heuristics[0],
                &p_heuristics[1],
                (int)p_heuristics.size() - 1,
                search_nh);
    } else if (search_type == "mhastarpp") {
        search = MakeMHAStarPP(
                graph.get(),
                p_heuristics[0],
                &p_heuristics[1],
                (int)p_heuristics.size() - 1,
                search_nh);
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

    if (!graph->UpdateStart(start_state_id)) {
        SMPL_ERROR("Failed to update start in the graph");
        return 1;
    }
    for (auto& h : heuristics) {
        if (!h->UpdateStart(start_state_id)) {
            SMPL_ERROR("Failed to update start in a heuristic");
            return 1;
        }
    }
    if (!search->UpdateStart(start_state_id)) {
        SMPL_ERROR("Failed to update start in the search");
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

    if (!graph->UpdateGoal(goal.get())) {
        SMPL_ERROR("Failed to update goal in the graph");
        return 1;
    }
    for (auto& h : heuristics) {
        if (!h->UpdateGoal(goal.get())) {
            SMPL_ERROR("Failed to update goal in a heuristic");
            return 1;
        }
    }
    if (!search->UpdateGoal(goal.get())) {
        SMPL_ERROR("Failed to update goal in the search");
        return 1;
    }

    ////////////////////////////
    // Finally, plan the path //
    ////////////////////////////

    auto time_params = smpl::TimeoutCondition();
    time_params.bounded = true;
    time_params.improve = false;
//    time_params.type = smpl::TimeoutCondition::TIME;
    time_params.type = smpl::TimeoutCondition::EXPANSIONS;
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
