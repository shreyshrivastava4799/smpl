#include "factories.h"

#include <utility>

#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/action_space.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>
#include <smpl/search/smhastar.h>
#include <smpl/search/fmhastar.h>
#include <smpl/search/umhastar.h>
#include <smpl/search/mhastarpp.h>
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

    // TODO: read discretization from config
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

    auto enable_long_motions = false;
    auto enable_ik_motion_xyz = false;
    auto enable_ik_motion_rpy = false;
    auto enable_ik_motion_xyzrpy = false;
    auto long_motions_thresh = 0.0;
    auto ik_motion_xyz_thresh = 0.0;
    auto ik_motion_rpy_thresh = 0.0;
    auto ik_motion_xyzrpy_thresh = 0.0;
    auto mprim_filename = std::string();

    nh.getParam("use_long_dist_mprims", enable_long_motions);
    nh.getParam("use_xyz_snap_mprim", enable_ik_motion_xyz);
    nh.getParam("use_rpy_snap_mprim", enable_ik_motion_rpy);
    nh.getParam("use_xyzrpy_snap_mprim", enable_ik_motion_xyzrpy);
    nh.getParam("short_dist_mprims_thresh", long_motions_thresh);
    nh.getParam("xyz_snap_dist_thresh", ik_motion_xyz_thresh);
    nh.getParam("rpy_snap_dist_thresh", ik_motion_rpy_thresh);
    nh.getParam("xyzrpy_snap_dist_thresh", ik_motion_xyzrpy_thresh);

    SMPL_INFO_STREAM("use_long_dist_mprims: " << enable_long_motions);
    SMPL_INFO_STREAM("use_xyz_snap_mprim: " << enable_ik_motion_xyz);
    SMPL_INFO_STREAM("use_rpy_snap_mprim: " << enable_ik_motion_rpy);
    SMPL_INFO_STREAM("use_xyzrpy_snap_mprim: " << enable_ik_motion_xyzrpy);
    SMPL_INFO_STREAM("short_dist_mprims_thresh: " << long_motions_thresh);
    SMPL_INFO_STREAM("xyz_snap_dist_thresh: " << ik_motion_xyz_thresh);
    SMPL_INFO_STREAM("rpy_snap_dist_thresh: " << ik_motion_rpy_thresh);
    SMPL_INFO_STREAM("xyzrpy_snap_dist_thresh: " << ik_motion_xyzrpy_thresh);

    if (!nh.getParam("mprim_filename", mprim_filename)) {
        SMPL_ERROR("Failed to read param 'mprim_filename' from the param server");
        return NULL;
    }

    SMPL_INFO_STREAM("mprim_filename: " << mprim_filename);

    if (!actions->Load(mprim_filename)) {
        SMPL_ERROR("Failed to load motion primitives");
        return NULL;
    }

    actions->EnableLongMotions(enable_long_motions);
    actions->EnableIKMotionXYZ(enable_ik_motion_xyz);
    actions->EnableIKMotionRPY(enable_ik_motion_rpy);
    actions->EnableIKMotionXYZRPY(enable_ik_motion_xyzrpy);

    actions->SetLongMotionThreshold(long_motions_thresh);
    actions->SetIKMotionXYZThreshold(ik_motion_xyz_thresh);
    actions->SetIKMotionRPYThreshold(ik_motion_rpy_thresh);
    actions->SetIKMotionXYZRPYThreshold(ik_motion_xyzrpy_thresh);

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

auto MakeWorkspaceLattice(
    smpl::RobotModel* model,
    smpl::CollisionChecker* checker,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::DiscreteSpace>
{
    struct SimpleWorkspaceLattice : smpl::WorkspaceLattice
    {
        std::unique_ptr<smpl::WorkspaceLatticeActionSpace> actions;
    };

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

    nh.getParam("res_x", resolutions.res_x);
    nh.getParam("res_y", resolutions.res_y);
    nh.getParam("res_z", resolutions.res_z);
    nh.getParam("num_roll_angles", resolutions.R_count);
    nh.getParam("num_pitch_angles", resolutions.P_count);
    nh.getParam("num_yaw_angles", resolutions.Y_count);
    nh.getParam("res_free_angles", resolutions.free_angle_res);

    SMPL_INFO_STREAM("res_x: " << resolutions.res_x);
    SMPL_INFO_STREAM("res_y: " << resolutions.res_y);
    SMPL_INFO_STREAM("res_z: " << resolutions.res_z);
    SMPL_INFO_STREAM("num_roll_angles: " << resolutions.R_count);
    SMPL_INFO_STREAM("num_pitch_angles: " << resolutions.P_count);
    SMPL_INFO_STREAM("num_yaw_angles: " << resolutions.Y_count);
    SMPL_INFO_STREAM("res_free_angles " << resolutions.free_angle_res);

    auto actions = smpl::make_unique<smpl::SimpleWorkspaceLatticeActionSpace>();

    // TODO: implement cost functions for workspace lattice

    auto graph = smpl::make_unique<SimpleWorkspaceLattice>();

    if (!graph->Init(model, checker, resolutions, actions.get())) {
        SMPL_ERROR("Failed to initialize Workspace Lattice");
        return NULL;
    }

    // TODO: weird name, and inconsistent with Manip Lattice's ActionSpace
    if (!InitSimpleWorkspaceLatticeActions(graph.get(), actions.get())) {
        SMPL_ERROR("Failed to initialize Simple Workspace Lattice Action Space");
        return NULL;
    }

    // NOTE: SimpleWorkspaceLatticeActionSpace really isn't that simple. It
    // includes an adaptive ik motion that is automatically enabled
    actions->m_ik_amp_enabled = true;
    actions->m_ik_amp_thresh = 0.05;

    graph->actions = std::move(actions);

    return std::move(graph);
}

auto MakeManipLatticeEGraph(
    smpl::RobotModel* model,
    smpl::CollisionChecker* checker,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::DiscreteSpace>
{
    struct SimpleManipLatticeEGraph : smpl::ManipLatticeEGraph
    {
        std::unique_ptr<smpl::ActionSpace> actions;
        std::unique_ptr<smpl::CostFunction> cost_function;
    };

    auto graph = smpl::make_unique<SimpleManipLatticeEGraph>();

    auto resolutions = std::vector<double>(
            model->jointVariableCount(),
            smpl::to_radians(1.0));

    auto actions = smpl::make_unique<smpl::ManipulationActionSpace>();
    auto cost_fun = smpl::make_unique<smpl::UniformCostFunction>();

    if (!graph->Init(
            model,
            checker,
            resolutions,
            actions.get(),
            cost_fun.get()))
    {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return NULL;
    }

    if (!actions->Init(graph->GetJointSpaceLattice())) {
        SMPL_ERROR("Failed to initialize Manipulation Action Space");
        return NULL;
    }

    auto mprim_filename = std::string();
    if (!nh.getParam("mprim_filename", mprim_filename)) {
        SMPL_ERROR("Failed to read param 'mprim_filename' from the param server");
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

    if (!cost_fun->Init(graph->GetJointSpaceLattice())) {
        SMPL_ERROR("Failed to initialize Uniform Cost Function");
        return NULL;
    }

    cost_fun->cost_per_action = 1000;

    std::string egraphs_directory;
    if (!nh.getParam("egraphs_directory", egraphs_directory)) {
        SMPL_ERROR("Failed to retrieve 'egraphs_directory' from the param server");
        return NULL;
    }

    if (!graph->LoadExperienceGraph(egraphs_directory)) {
        SMPL_ERROR("Failed to load experience graph");
        return NULL;
    }

    graph->actions = std::move(actions);
    graph->cost_function = std::move(cost_fun);

    return std::move(graph);
}

auto MakeJointDistHeuristic(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto heuristic = smpl::make_unique<smpl::JointDistHeuristic>();
    if (!heuristic->Init(graph)) {
        SMPL_ERROR("Failed to initialize Joint Dist Heuristic");
        return NULL;
    }
    return std::move(heuristic);
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

    auto inflation_radius = 0.0;
    auto cost_per_cell = 1;

    nh.getParam("inflation_radius", inflation_radius);
    nh.getParam("cost_per_cell", cost_per_cell);

    SMPL_INFO_STREAM("inflation_radius: " << inflation_radius);
    SMPL_INFO_STREAM("cost_per_cell: " << cost_per_cell);

    heuristic->SetInflationRadius(inflation_radius);
    heuristic->SetCostPerCell(cost_per_cell);

    // TODO: this is kinda dumb to have to remember to do this after
    // SetInflationRadius
    heuristic->SyncGridAndBFS();
    SV_SHOW_DEBUG_NAMED("bfs_walls", heuristic->GetWallsVisualization());

    return std::move(heuristic);
}

auto MakeMultiFrameBFSHeuristic(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto* checker = graph->GetCollisionChecker();
    auto* grid_accessor = checker->GetExtension<smpl::IOccupancyGridAccessor>();
    if (grid_accessor == NULL) {
        return NULL;
    }

    auto heuristic = smpl::make_unique<smpl::MultiFrameBFSHeuristic>();
    if (!heuristic->Init(graph, grid_accessor->GetGrid())) {
        SMPL_ERROR("Failed to initialize BFS Heuristic");
        return NULL;
    }

    auto inflation_radius = 0.0;
    auto offset_x = 0.0;
    auto offset_y = 0.0;
    auto offset_z = 0.0;
    auto cost_per_cell = 1;

    nh.getParam("inflation_radius", inflation_radius);
    nh.getParam("cost_per_cell", cost_per_cell);
    nh.getParam("offset/x", offset_x);
    nh.getParam("offset/y", offset_y);
    nh.getParam("offset/z", offset_z);

    SMPL_INFO_STREAM("inflation_radius: " << inflation_radius);
    SMPL_INFO_STREAM("cost_per_cell: " << cost_per_cell);
    SMPL_INFO_STREAM("offset/x: " << offset_x);
    SMPL_INFO_STREAM("offset/y: " << offset_y);
    SMPL_INFO_STREAM("offset/z: " << offset_z);

    heuristic->SetInflationRadius(inflation_radius);
    heuristic->SetCostPerCell(cost_per_cell);
    heuristic->SetOffset(offset_x, offset_y, offset_z);

    // TODO: this is kinda dumb to have to remember to do this after
    // SetInflationRadius
    heuristic->SyncGridAndBFS();
    SV_SHOW_DEBUG(heuristic->GetWallsVisualization());

    return std::move(heuristic);
}

auto MakeEuclidDistHeuristic(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto heuristic = smpl::make_unique<smpl::EuclidDistHeuristic>();

    if (!heuristic->Init(graph)) {
        SMPL_ERROR("Failed to initialize Euclid Dist Heuristic");
        return NULL;
    }

    return std::move(heuristic);
}

auto MakeDijkstraEGraph3DHeuristic(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Heuristic>
{
    auto* checker = graph->GetCollisionChecker();
    auto* grid_accessor = checker->GetExtension<smpl::IOccupancyGridAccessor>();
    if (grid_accessor == NULL) {
        return NULL;
    }

    auto heuristic = smpl::make_unique<smpl::DijkstraEGraphHeuristic3D>();
    if (!heuristic->Init(graph, grid_accessor->GetGrid())) {
        SMPL_ERROR("Failed to initialize Dijkstra EGraph 3D Heuristic");
        return NULL;
    }

    auto inflation_radius = 0.0;
    auto cost_per_cell = 1;
    auto egraph_weight = 1.0;

    nh.getParam("inflation_radius", inflation_radius);
    nh.getParam("cost_per_cell", cost_per_cell);
    nh.getParam("egraph_weight", egraph_weight);

    SMPL_INFO_STREAM("inflation_radius: " << inflation_radius);
    SMPL_INFO_STREAM("cost_per_cell: " << cost_per_cell);
    SMPL_INFO_STREAM("egraph_weight: " << egraph_weight);

    heuristic->SetInflationRadius(inflation_radius);
    // TODO:
//    heuristic->SetCostPerCell(cost_per_cell);
    heuristic->SetEGraphWeight(egraph_weight);

    // TODO: this is kinda dumb to have to remember to do this
    // ...additionally, it would be nice if these names were uniform across
    // bfs and dijkstra variants?
    heuristic->SyncGridAndDijkstra();
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
    if (!Init(search.get(), graph, heuristic)) {
        SMPL_ERROR("Failed to initialize ARA*");
        return NULL;
    }

    auto initial_eps = 1.0;
    auto allow_partial_solutions = false;
    auto target_eps = 1.0;
    auto delta_eps = 1.0;

    nh.getParam("initial_eps", initial_eps);
    nh.getParam("allow_partial_solutions", allow_partial_solutions);
    nh.getParam("target_eps", target_eps);
    nh.getParam("delta_eps", delta_eps);

    SMPL_INFO_STREAM("initial_eps: " << initial_eps);
    SMPL_INFO_STREAM("allow_partial_solutions: " << allow_partial_solutions);
    SMPL_INFO_STREAM("target_eps: " << target_eps);
    SMPL_INFO_STREAM("delta_eps: " << delta_eps);

    SetInitialEps(search.get(), initial_eps);
    SetAllowPartialSolutions(search.get(), allow_partial_solutions);
    SetTargetEps(search.get(), target_eps);
    SetDeltaEps(search.get(), delta_eps);

    return std::move(search);
}

auto MakeSMHAStar(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* anchor,
    smpl::Heuristic** heurs,
    int num_heurs,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::SMHAStar>();
    if (!Init(search.get(), graph, anchor, heurs, num_heurs)) {
        SMPL_ERROR("Failed to initialize SMHA*");
        return NULL;
    }

    auto w_heur_init = 1.0;
    auto w_anchor_init = 1.0;

    nh.getParam("w_heur_init", w_heur_init);
    nh.getParam("w_anchor_init", w_anchor_init);

    SetInitialEps(search.get(), w_heur_init);
    SetInitialMHAEps(search.get(), w_anchor_init);
    return std::move(search);
}

auto MakeFMHAStar(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* anchor,
    smpl::Heuristic** heurs,
    int num_heurs,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::FMHAStar>();
    if (!Init(search.get(), graph, anchor, heurs, num_heurs)) {
        SMPL_ERROR("Failed to initialize SMHA*");
        return NULL;
    }

    auto w_heur_init = 1.0;
    auto anchor_expansion_freq = 1;

    nh.getParam("w_heur_init", w_heur_init);
    nh.getParam("anchor_expansion_freq", anchor_expansion_freq);

    SetInitialEps(search.get(), w_heur_init);
    SetTargetEps(search.get(), 1.0);
    SetDeltaEps(search.get(), 1.0);
    SetAnchorExpansionFreq(search.get(), anchor_expansion_freq);

    return std::move(search);
}

auto MakeUMHAStar(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* anchor,
    smpl::Heuristic** heurs,
    int num_heurs,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::UMHAStar>();
    if (!Init(search.get(), graph, anchor, heurs, num_heurs)) {
        SMPL_ERROR("Failed to initialize SMHA*");
        return NULL;
    }

    auto w_heur_init = 1.0;
    auto anchor_expansion_freq = 1;

    nh.getParam("w_heur_init", w_heur_init);
    nh.getParam("anchor_expansion_freq", anchor_expansion_freq);

    SetInitialEps(search.get(), w_heur_init);
    SetTargetEps(search.get(), 1.0);
    SetDeltaEps(search.get(), 1.0);
    SetAnchorExpansionFreq(search.get(), anchor_expansion_freq);

    return std::move(search);
}

auto MakeMHAStarPP(
    smpl::DiscreteSpace* graph,
    smpl::Heuristic* anchor,
    smpl::Heuristic** heurs,
    int num_heurs,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::Search>
{
    auto search = smpl::make_unique<smpl::MHAStarPP>();
    if (!Init(search.get(), graph, anchor, heurs, num_heurs)) {
        SMPL_ERROR("Failed to initialize SMHA*");
        return NULL;
    }

    auto w_heur_init = 1.0;
    auto anchor_expansion_freq = 1;

    nh.getParam("w_heur_init", w_heur_init);
    nh.getParam("anchor_expansion_freq", anchor_expansion_freq);

    SetInitialEps(search.get(), w_heur_init);
    SetTargetEps(search.get(), 1.0);
    SetDeltaEps(search.get(), 1.0);
    SetAnchorExpansionFreq(search.get(), anchor_expansion_freq);

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
    if (!nh.getParam("x",      goal_vals[0])) return NULL;
    if (!nh.getParam("y",      goal_vals[1])) return NULL;
    if (!nh.getParam("z",      goal_vals[2])) return NULL;
    if (!nh.getParam("roll",   goal_vals[3])) return NULL;
    if (!nh.getParam("pitch",  goal_vals[4])) return NULL;
    if (!nh.getParam("yaw",    goal_vals[5])) return NULL;

    goal->pose = smpl::MakeAffine(
            goal_vals[0], goal_vals[1], goal_vals[2],
            goal_vals[5], goal_vals[4], goal_vals[3]);

    // similar to call_planner tolerance of (0.015, 0.05)
    goal->tolerance.xyz[0] = goal->tolerance.xyz[1] = goal->tolerance.xyz[2] = 0.015;
    goal->tolerance.rpy[0] = goal->tolerance.rpy[1] = goal->tolerance.rpy[2] = smpl::to_radians(3.0);

    SV_SHOW_INFO_NAMED("pose_goal", goal->GetVisualization("odom_combined"));

    return std::move(goal);
}

auto MakeJointStateGoal(
    smpl::DiscreteSpace* graph,
    const ros::NodeHandle& nh)
    -> std::unique_ptr<smpl::GoalConstraint>
{
    auto goal = smpl::make_unique<smpl::JointStateGoal>();
    if (!goal->Init(graph)) {
        SMPL_ERROR("Failed to initialize the goal");
        return NULL;
    }

    auto goal_state = smpl::RobotState();
    if (!nh.getParam("state", goal_state)) {
        return NULL;
    }

    goal->SetGoalState(goal_state);

    auto goal_tolerance = std::vector<double>();
    if (!nh.getParam("tolerance", goal_tolerance)) {
        return NULL;
    }

    goal->SetGoalTolerance(goal_tolerance);

    auto* model = graph->GetRobotModel();

    auto full_state_vis = model->GetVisualization(goal_state);
    auto id = (int32_t)0;
    for (auto& marker : full_state_vis) {
        marker.frame_id = "odom_combined";
        marker.ns = "joint_goal";
        marker.color = smpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
        marker.id = id++;
    }

    SV_SHOW_INFO_NAMED("joint_goal", full_state_vis);
    return std::move(goal);
}
