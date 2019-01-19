#include "factories.h"

#include <utility>

#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/action_space.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>
#include <smpl/search/smhastar.h>
#include <smpl/stl/memory.h>

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
    if (!nh.getParam("mprim_filename", mprim_filename)) {
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

auto MakeBFSHeuristic(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
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

    double inflation_radius;
    int cost_per_cell;

    if (!nh.getParam("inflation_radius", inflation_radius)) {
        return false;
    }

    if (!nh.getParam("cost_per_cell", cost_per_cell)) {
        return false;
    }

    heuristic->SetInflationRadius(inflation_radius);
    heuristic->SetCostPerCell(cost_per_cell);

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
    search->SetAllowPartialSolutions(false);
    search->SetTargetEpsilon(1.0);
    search->SetDeltaEpsilon(1.0);

    return std::move(search);
}

auto MakeSMHAStar(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* anchor,
    smpl::Heuristic** heurs,
    int num_heuristics,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::SMHAStar>();
    if (!Init(search.get(), graph, anchor, heurs, num_heuristics)) {
        SMPL_ERROR("Failed to initialize SMHA*");
        return NULL;
    }

    SetInitialEps(search.get(), 100.0);
    SetInitialMHAEps(search.get(), 5.0);
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

