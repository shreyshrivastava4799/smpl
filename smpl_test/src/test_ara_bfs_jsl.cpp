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
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/search/arastar.h>
#include <smpl/stl/memory.h>

// project includes
#include "test_scenario.h"

// Factory function to construct, initialize, and configure a ManipLattice.
auto MakeManipLattice(
    smpl::RobotModel* model,
    smpl::CollisionChecker* checker,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::DiscreteSpace>
{
    struct SimpleManipLattice : smpl::ManipLattice
    {
        std::unique_ptr<smpl::ActionSpace> actions;
        std::unique_ptr<smpl::CostFunction> cost_function;
    };

    //////////////////////////////////
    // Initialize the DiscreteSpace //
    //////////////////////////////////

    auto graph = smpl::make_unique<SimpleManipLattice>();

    auto resolutions = std::vector<double>(
            model->jointVariableCount(),
            smpl::to_radians(1.0));

    auto actions = smpl::make_unique<smpl::ManipulationActionSpace>();

    auto cost_function = smpl::make_unique<smpl::UniformCostFunction>();

    if (!graph->Init(
            model, checker, resolutions, actions.get(), cost_function.get()))
    {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return NULL;
    }

    /////////////////////////////////
    // Initialize the Action Space //
    /////////////////////////////////

    if (!actions->Init(graph.get())) {
        SMPL_ERROR("Failed to initialize Manipulation Action Space");
        return NULL;
    }

    auto mprim_filename = std::string();
    if (!nh.getParam("planning/mprim_filename", mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return NULL;
    }

    if (!actions->Load(mprim_filename)) {
        SMPL_ERROR("Failed to load motion primitives");
        return NULL;
    }

    actions->EnableLongMotions(true);
    actions->EnableIKMotionXYZ(false);
    actions->EnableIKMotionRPY(false);
    actions->EnableIKMotionXYZRPY(true);

    actions->SetLongMotionThreshold(0.4);
    actions->SetIKMotionXYZRPYThreshold(0.10); // 0.2 in call_planner

    //////////////////////////////////
    // Initialize the Cost Function //
    //////////////////////////////////

    if (!cost_function->Init(graph.get())) {
        SMPL_ERROR("Failed to initialize Uniform Cost Function");
        return NULL;
    }

    cost_function->cost_per_action = 1000;

    graph->actions = std::move(actions);
    graph->cost_function = std::move(cost_function);

    return std::move(graph);
}

// Init(DiscreteSpace)
// Init(DiscreteSpace, CollisionChecker)
// Init(DiscreteSpace, ProjectToPoint, OccupancyGrid)

auto MakeBFSHeuristic(smpl::DiscreteSpace* graph, const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto* checker = graph->GetCollisionChecker();
    auto* grid_accessor = checker->GetExtension<smpl::IOccupancyGridAccessor>();
    if (grid_accessor == NULL) {
        return NULL;
    }

    auto heuristic = smpl::make_unique<smpl::BFSHeuristic>();
    if (!heuristic->Init(graph, grid_accessor->GetGrid())) {
        SMPL_ERROR("Failed to initialize BFS Heuristic");
        return NULL;
    }

    heuristic->SetInflationRadius(0.04);
    heuristic->SetCostPerCell(100);

    // TODO: this is kinda dumb to have to remember to do this
    heuristic->SyncGridAndBFS();
    SV_SHOW_DEBUG(heuristic->GetWallsVisualization());

    return std::move(heuristic);
}

auto MakeARAStar(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* heuristic,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::ARAStar>();
    if (!search->Init(graph, heuristic)) {
        SMPL_ERROR("Failed to initialize ARA*");
        return NULL;
    }

    search->SetInitialEps(100.0);
    search->SetSearchMode(false);
    search->SetAllowPartialSolutions(false);
    search->SetTargetEpsilon(1.0);
    search->SetDeltaEpsilon(1.0);
    search->SetImproveSolution(false);
    search->SetBoundExpansions(true);

    return std::move(search);
}

auto MakePoseGoal(smpl::DiscreteSpace* graph, const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::GoalConstraint>
{
    auto goal = smpl::make_unique<smpl::PoseGoal>();

    if (!goal->Init(graph)) {
        return NULL;
    }

    double goal_vals[6];
    nh.param("goal/x",      goal_vals[0], 0.0);
    nh.param("goal/y",      goal_vals[1], 0.0);
    nh.param("goal/z",      goal_vals[2], 0.0);
    nh.param("goal/roll",   goal_vals[3], 0.0);
    nh.param("goal/pitch",  goal_vals[4], 0.0);
    nh.param("goal/yaw",    goal_vals[5], 0.0);

    goal->pose = smpl::MakeAffine(
            goal_vals[0], goal_vals[1], goal_vals[2],
            goal_vals[5], goal_vals[4], goal_vals[3]);

    // similar to call_planner tolerance of (0.015, 0.05)
    goal->tolerance.xyz[0] = goal->tolerance.xyz[1] = goal->tolerance.xyz[2] = 0.015;
    goal->tolerance.rpy[0] = goal->tolerance.rpy[1] = goal->tolerance.rpy[2] = smpl::to_radians(3.0);

    SV_SHOW_INFO_NAMED("pose_goal", goal->GetVisualization("odom_combined"));

    return std::move(goal);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "test_ara_bfs_jsl");
    ros::NodeHandle ph("~");

    auto scenario = TestScenario();
    InitTestScenario(&scenario);

    auto graph = MakeManipLattice(
            scenario.planning_model.get(), &scenario.collision_model, ph);
    if (graph == NULL) {
        return 1;
    }

    auto heuristic = MakeBFSHeuristic(graph.get(), ph);
    if (heuristic == NULL) {
        return 1;
    }

    auto* h = heuristic.get();
    if (!graph->UpdateHeuristics(&h, 1)) {
        SMPL_ERROR("Failed to associate BFS Heuristic with Manip Lattice");
        return 1;
    }

    auto search = MakeARAStar(graph.get(), heuristic.get(), ph);
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
        !heuristic->UpdateStart(start_state_id) ||
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
        !heuristic->UpdateGoal(goal.get()) ||
        !search->UpdateGoal(goal.get()))
    {
        SMPL_ERROR("Failed to update the goal");
        return 1;
    }

    SV_SHOW_DEBUG_NAMED(
            "bfs_values",
            dynamic_cast<smpl::BFSHeuristic*>(heuristic.get())->GetValuesVisualization());

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
