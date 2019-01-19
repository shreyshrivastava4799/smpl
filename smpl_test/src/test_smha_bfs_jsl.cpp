// standard includes
#include <chrono>
#include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/search/search.h>
#include <smpl/heuristic/bfs_heuristic.h>

// project includes
#include "test_scenario.h"

#include "factories.h"

// Init(DiscreteSpace)
// Init(DiscreteSpace, CollisionChecker)
// Init(DiscreteSpace, ProjectToPoint, OccupancyGrid)

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ara_bfs_jsl");
    ros::NodeHandle ph("~");

    auto scenario = TestScenario();
    InitTestScenario(&scenario);

    auto graph = MakeManipLattice(
            scenario.planning_model.get(),
            &scenario.collision_model,
            ros::NodeHandle(ph, "graph"));
    if (graph == NULL) {
        return 1;
    }

    auto heur_0 = MakeBFSHeuristic(graph.get(), ros::NodeHandle(ph, "h_0"));
    if (heur_0 == NULL) {
        return 1;
    }

    auto heur_1 = MakeBFSHeuristic(graph.get(), ros::NodeHandle(ph, "h_1"));
    if (heur_1 == NULL) {
        return 1;
    }

    smpl::Heuristic* h[2] = { heur_0.get(), heur_1.get() };
    if (!graph->UpdateHeuristics(h, 2)) {
        SMPL_ERROR("Failed to associate BFS Heuristic with Manip Lattice");
        return 1;
    }

    auto search = MakeSMHAStar(graph.get(), h[0], &h[1], 1, ph);
    if (search == NULL) {
        return 1;
    }

    /////////////////////////
    // Set the start state //
    /////////////////////////

    auto* rps = graph->GetExtension<smpl::RobotPlanningSpace>();
    if (rps == NULL) {
        return 1;
    }

    auto start_state = smpl::RobotState();
    auto success = false;
    std::tie(start_state, success) = MakeRobotState(
            &scenario.start_state, scenario.planning_model.get());
    if (!success) return 1;

    auto start_state_id = rps->GetStateID(start_state);

    if (!graph->UpdateStart(start_state_id) ||
        !heur_0->UpdateStart(start_state_id) ||
        !heur_1->UpdateStart(start_state_id) ||
        !search->UpdateStart(start_state_id))
    {
        SMPL_ERROR("Failed to update the start state");
        return 1;
    }

    /////////////////////
    // Update the goal //
    /////////////////////

    auto goal = MakePoseGoal(graph.get(), ph);
    if (goal == NULL) {
        return 1;
    }

    if (!graph->UpdateGoal(goal.get()) ||
        !heur_0->UpdateGoal(goal.get()) ||
        !heur_1->UpdateGoal(goal.get()) ||
        !search->UpdateGoal(goal.get()))
    {
        SMPL_ERROR("Failed to update the goal");
        return 1;
    }

    SV_SHOW_DEBUG_NAMED(
            "bfs_values",
            dynamic_cast<smpl::BFSHeuristic*>(heur_0.get())->GetValuesVisualization());

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

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    return AnimateSolution(&scenario, scenario.planning_model.get(), &path);
}
