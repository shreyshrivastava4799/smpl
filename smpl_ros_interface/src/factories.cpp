#include <smpl_ros_interface/factories.h>

#define WORKING_DEV 0

// system includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/adaptive_workspace_lattice.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/simple_workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice_egraph.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#if WORKING_DEV
#include <smpl/search/adaptive_planner.h>
#endif
#include <smpl/search/arastar.h>
#if WORKING_DEV
#include <smpl/search/awastar.h>
#include <smpl/search/experience_graph_planner.h>
#endif
#include <smpl/stl/memory.h>

namespace smpl {

static const char* PI_LOGGER = "simple";

struct ManipulationActionSpaceParams
{
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_long_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double long_dist_mprims_thresh;
};

// Lookup parameters for ManipulationActionSpace, setting reasonable defaults
// for missing parameters. Return false if any required parameter is not found.
static
bool GetManipulationActionSpaceParams(
    ManipulationActionSpaceParams& params,
    const PlanningParams& pp)
{
    if (!pp.getParam("mprim_filename", params.mprim_filename)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return false;
    }

    pp.param("use_multiple_ik_solutions", params.use_multiple_ik_solutions, false);

    pp.param("use_xyz_snap_mprim", params.use_xyz_snap_mprim, false);
    pp.param("use_rpy_snap_mprim", params.use_rpy_snap_mprim, false);
    pp.param("use_xyzrpy_snap_mprim", params.use_xyzrpy_snap_mprim, false);
    pp.param("use_long_dist_mprims", params.use_long_dist_mprims, false);

    pp.param("xyz_snap_dist_thresh", params.xyz_snap_thresh, 0.0);
    pp.param("rpy_snap_dist_thresh", params.rpy_snap_thresh, 0.0);
    pp.param("xyzrpy_snap_dist_thresh", params.xyzrpy_snap_thresh, 0.0);
    pp.param("long_dist_mprims_thresh", params.long_dist_mprims_thresh, 0.0);
    return true;
}

template <class T>
static auto ParseMapFromString(const std::string& s)
    -> std::unordered_map<std::string, T>
{
    auto map = std::unordered_map<std::string, T>();
    std::istringstream ss(s);
    auto key = std::string();
    auto value = T();
    while (ss >> key >> value) {
        map.insert(std::make_pair(key, value));
    }
    return map;
}

// Return true if the variable is part of a multi-dof joint. Optionally store
// the name of the joint and the local name of the variable.
bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name,
    std::string* local_name)
{
    auto slash_pos = name.find_last_of('/');
    if (slash_pos != std::string::npos) {
        if (joint_name != NULL) {
            *joint_name = name.substr(0, slash_pos);
        }
        if (local_name != NULL) {
            *local_name = name.substr(slash_pos + 1);
        }
        return true;
    } else {
        return false;
    }
}

static
void PrintActionSpace(const smpl::ManipulationActionSpace& actions)
{
    SMPL_INFO("Action Space:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES; ++i) {
        auto prim_type = (smpl::MotionPrimitive::Type)i;
        if (actions.IsMotionTypeEnabled(prim_type)) {
            SMPL_INFO("  %s enabled @ %0.3f",
                    to_cstring(prim_type),
                    actions.GetMotionTypeThreshold(prim_type));
        } else {
            SMPL_INFO("  %s disabled", to_cstring(prim_type));
        }
    }

    SMPL_INFO("short distance motion primitives:");
    for (auto& prim : actions.m_short_dist_mprims) {
        SMPL_INFO_STREAM("  " << prim.action);
    }
    SMPL_INFO("long distance motion primitives:");
    for (auto& prim : actions.m_long_dist_mprims) {
        SMPL_INFO_STREAM("  " << prim.action);
    }
}

auto MakeManipLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    auto resolutions = std::vector<double>(robot->jointVariableCount());

    auto disc_string = std::string();
    if (!params.getParam("discretization", disc_string)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return NULL;
    }

    auto disc = ParseMapFromString<double>(disc_string);
    SMPL_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        auto joint_name = std::string();
        auto local_name = std::string();
        if (IsMultiDOFJointVariable(vname, &joint_name, &local_name)) {
            // adjust variable name if a variable of a multi-dof joint
            auto mdof_vname = joint_name + "_" + local_name;
            auto dit = disc.find(mdof_vname);
            if (dit == end(disc)) {
                SMPL_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return NULL;
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == end(disc)) {
                SMPL_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return NULL;
            }
            resolutions[vidx] = dit->second;
        }

        SMPL_DEBUG_NAMED(PI_LOGGER, "resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    auto action_params = ManipulationActionSpaceParams();
    if (!GetManipulationActionSpaceParams(action_params, params)) {
        return NULL;
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLattice and
    // ManipulationActionSpace
    struct SimpleManipLattice : public ManipLattice {
        ManipulationActionSpace actions;
        UniformCostFunction cost_fun;
    };

    auto space = make_unique<SimpleManipLattice>();

    if (!space->Init(robot, checker, resolutions, &space->actions, &space->cost_fun)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice");
        return NULL;
    }

    if (!space->actions.Init(space.get())) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manipulation Action Space");
        return NULL;
    }

    if (grid) {
        space->SetVisualizationFrameId(grid->getReferenceFrame());
    }

    auto& actions = space->actions;
    actions.EnableMultipleIKSolutions(action_params.use_multiple_ik_solutions);
    actions.EnableIKMotionXYZ(action_params.use_xyz_snap_mprim);
    actions.EnableIKMotionRPY(action_params.use_rpy_snap_mprim);
    actions.EnableIKMotionXYZRPY(action_params.use_xyzrpy_snap_mprim);
    actions.EnableLongMotions(action_params.use_long_dist_mprims);
    actions.SetIKMotionXYZThreshold(action_params.xyz_snap_thresh);
    actions.SetIKMotionRPYThreshold(action_params.rpy_snap_thresh);
    actions.SetIKMotionXYZRPYThreshold(action_params.xyzrpy_snap_thresh);
    actions.SetLongMotionThreshold(action_params.long_dist_mprims_thresh);

    if (!actions.Load(action_params.mprim_filename)) {
        SMPL_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return NULL;
    }

    PrintActionSpace(actions);

    if (!space->cost_fun.Init(space.get())) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Failed to initialize Uniform Cost Function");
        return NULL;
    }
    space->cost_fun.cost_per_action = 1000;

    return std::move(space);
}

auto MakeManipLatticeEGraph(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params.getParam("discretization", disc_string)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return NULL;
    }
    auto disc = ParseMapFromString<double>(disc_string);
    SMPL_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == end(disc)) {
            SMPL_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return NULL;
        }
        resolutions[vidx] = dit->second;
    }

    auto action_params = ManipulationActionSpaceParams();
    if (!GetManipulationActionSpaceParams(action_params, params)) {
        return NULL; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLatticeEgraph and
    // ManipulationActionSpace
    struct SimpleManipLatticeEgraph : public ManipLatticeEGraph
    {
        ManipulationActionSpace actions;
        UniformCostFunction cost_fun;
    };

    auto space = make_unique<SimpleManipLatticeEgraph>();

    if (!space->Init(robot, checker, resolutions, &space->actions, &space->cost_fun)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Egraph");
        return NULL;
    }

    if (!space->actions.Init(&space->m_lattice)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return NULL;
    }

    auto& actions = space->actions;
    actions.EnableLongMotions(action_params.use_long_dist_mprims);
    actions.EnableMultipleIKSolutions(action_params.use_multiple_ik_solutions);
    actions.EnableIKMotionXYZ(action_params.use_xyz_snap_mprim);
    actions.EnableIKMotionRPY(action_params.use_rpy_snap_mprim);
    actions.EnableIKMotionXYZRPY(action_params.use_xyzrpy_snap_mprim);
    actions.SetIKMotionXYZThreshold(action_params.xyz_snap_thresh);
    actions.SetIKMotionRPYThreshold(action_params.rpy_snap_thresh);
    actions.SetIKMotionXYZRPYThreshold(action_params.xyzrpy_snap_thresh);
    actions.SetLongMotionThreshold(action_params.long_dist_mprims_thresh);
    if (!actions.Load(action_params.mprim_filename)) {
        SMPL_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return NULL;
    }

    if (!space->cost_fun.Init(&space->m_lattice)) {
        SMPL_ERROR("Failed to initialize Cost Function");
        return NULL;
    }
    space->cost_fun.cost_per_action = 1000;

    auto egraph_path = std::string();
    if (params.getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->LoadExperienceGraph(egraph_path);
    } else {
        SMPL_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

static
bool GetWorkspaceLatticeParams(
    const PlanningParams& params,
    const RobotModel* robot,
    const IRedundantManipulator* rmi,
    WorkspaceProjectionParams* wsp)
{
    auto disc_string = std::string();
    if (!params.getParam("discretization", disc_string)) {
        SMPL_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return false;
    }

    auto disc = ParseMapFromString<double>(disc_string);
    SMPL_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu variables", disc.size());

    auto extract_disc = [&](const char* name, double* d)
    {
        auto it = disc.find(name);
        if (it == end(disc)) {
            SMPL_ERROR("Missing discretization for variable '%s'", name);
            return false;
        }

        *d = it->second;
        return true;
    };

    double res_R, res_P, res_Y;
    if (!extract_disc("fk_pos_x", &wsp->res_x) ||
        !extract_disc("fk_pos_y", &wsp->res_y) ||
        !extract_disc("fk_pos_z", &wsp->res_z) ||
        !extract_disc("fk_rot_r", &res_R) ||
        !extract_disc("fk_rot_p", &res_P) ||
        !extract_disc("fk_rot_y", &res_Y))
    {
        return false;
    }

    wsp->R_count = (int)std::round(2.0 * M_PI / res_R);
    wsp->P_count = (int)std::round(M_PI / res_P) + 1; // TODO: force discrete values at -pi/2, 0, and pi/2
    wsp->Y_count = (int)std::round(2.0 * M_PI / res_Y);

    wsp->free_angle_res.resize(rmi->redundantVariableCount());
    for (int i = 0; i < rmi->redundantVariableCount(); ++i) {
        auto rvi = rmi->redundantVariableIndex(i);
        auto name = robot->getPlanningJoints()[rvi];
        std::string joint_name, local_name;
        if (IsMultiDOFJointVariable(name, &joint_name, &local_name)) {
            name = joint_name + "_" + local_name;
        }
        if (!extract_disc(name.c_str(), &wsp->free_angle_res[i])) {
            return false;
        }
    }

    return true;
}

auto MakeWorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    SMPL_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");

    auto* rmi = robot->GetExtension<IRedundantManipulator>();
    if (rmi == NULL) {
        SMPL_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return NULL;
    }

    auto wsp = WorkspaceProjectionParams();
    if (!GetWorkspaceLatticeParams(params, robot, rmi, &wsp)) {
        return NULL;
    }

    struct SimpleWorkspaceLattice : public WorkspaceLattice
    {
        SimpleWorkspaceLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleWorkspaceLattice>();
    if (!space->Init(robot, checker, wsp, &space->actions)) {
        SMPL_ERROR("Failed to initialize Workspace Lattice");
        return NULL;
    }

    if (!InitSimpleWorkspaceLatticeActions(space.get(), &space->actions)) {
        return NULL;
    }

    space->SetVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

auto MakeWorkspaceLatticeEGraph(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    SMPL_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice E-Graph");

    auto* rmi = robot->GetExtension<IRedundantManipulator>();
    if (rmi == NULL) {
        SMPL_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return NULL;
    }

    auto wsp = WorkspaceProjectionParams();
    if (!GetWorkspaceLatticeParams(params, robot, rmi, &wsp)) {
        return NULL;
    }

    struct SimpleWorkspaceLatticeEGraph : public WorkspaceLatticeEGraph
    {
        SimpleWorkspaceLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleWorkspaceLatticeEGraph>();
    if (!space->Init(robot, checker, wsp, &space->actions)) {
        SMPL_ERROR("Failed to initialize Workspace Lattice");
        return NULL;
    }

    if (!InitSimpleWorkspaceLatticeActions(&space->m_lattice, &space->actions)) {
        return NULL;
    }

    space->m_lattice.SetVisualizationFrameId(grid->getReferenceFrame());

    auto egraph_path = std::string();
    if (params.getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->LoadExperienceGraph(egraph_path);
    } else {
        SMPL_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

auto MakeAdaptiveWorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>
{
    SMPL_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
    auto wsp = WorkspaceProjectionParams();
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 36; //360;
    wsp.P_count = 19; //180 + 1;
    wsp.Y_count = 36; //360;

    auto* rmi = robot->GetExtension<IRedundantManipulator>();
    if (rmi == NULL) {
        SMPL_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return NULL;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), to_radians(1.0));

    auto space = make_unique<AdaptiveWorkspaceLattice>();
    if (!space->Init(robot, checker, wsp, grid)) {
        SMPL_ERROR("Failed to initialize Workspace Lattice");
        return NULL;
    }

    return std::move(space);
}

auto MakeMultiFrameBFSHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<MultiFrameBFSHeuristic>();
    h->SetCostPerCell(params.cost_per_cell);
    double inflation_radius;
    params.param("bfs_inflation_radius", inflation_radius, 0.0);
    h->SetInflationRadius(inflation_radius);
    if (!h->Init(space, grid)) {
        return NULL;
    }

    double offset_x, offset_y, offset_z;
    params.param("offset_x", offset_x, 0.0);
    params.param("offset_y", offset_y, 0.0);
    params.param("offset_z", offset_z, 0.0);
    h->setOffset(offset_x, offset_y, offset_z);

    return std::move(h);
}

auto MakeBFSHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<BfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    double inflation_radius;
    params.param("bfs_inflation_radius", inflation_radius, 0.0);
    h->setInflationRadius(inflation_radius);
    if (!h->init(space, grid)) {
        return NULL;
    }
    return std::move(h);
};

auto MakeEuclidDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<EuclidDistHeuristic>();

    if (!h->init(space)) {
        return NULL;
    }

    double wx, wy, wz, wr;
    params.param("x_coeff", wx, 1.0);
    params.param("y_coeff", wy, 1.0);
    params.param("z_coeff", wz, 1.0);
    params.param("rot_coeff", wr, 1.0);

    h->setWeightX(wx);
    h->setWeightY(wy);
    h->setWeightZ(wz);
    h->setWeightRot(wr);
    return std::move(h);
};

auto MakeJointDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<JointDistHeuristic>();
    if (!h->init(space)) {
        return NULL;
    }
    return std::move(h);
};

auto MakeDijkstraEgraphHeuristic3D(
    RobotPlanningSpace* space,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<DijkstraEgraphHeuristic3D>();

//    h->setCostPerCell(params.cost_per_cell);
    double inflation_radius;
    params.param("bfs_inflation_radius", inflation_radius, 0.0);
    h->setInflationRadius(inflation_radius);
    if (!h->init(space, grid)) {
        return NULL;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);

    return std::move(h);
};

auto MakeJointDistEGraphHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    struct JointDistEGraphHeuristic : public GenericEgraphHeuristic {
        JointDistHeuristic jd;
    };

    auto h = make_unique<JointDistEGraphHeuristic>();
    if (!h->init(space, &h->jd)) {
        return NULL;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);
    return std::move(h);
};

auto MakeARAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<ARAStar>(space, heuristic);

    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    params.param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (params.getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (params.getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (params.getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (params.getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (params.getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (params.getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    return std::move(search);
}

#if WORKING_DEV
auto MakeAWAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AWAStar>(space, heuristic);
    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    return std::move(search);
}
#endif

auto MakeMHAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    struct MHAPlannerAdapter : public MHAPlanner {
        std::vector<Heuristic*> heuristics;

        MHAPlannerAdapter(
            DiscreteSpaceInformation* space,
            Heuristic* anchor,
            Heuristic** heurs,
            int hcount)
        :
            MHAPlanner(space, anchor, heurs, hcount)
        { }
    };

    std::vector<Heuristic*> heuristics;
    heuristics.push_back(heuristic);

    auto search = make_unique<MHAPlannerAdapter>(
            space, heuristics[0], &heuristics[0], heuristics.size());

    search->heuristics = std::move(heuristics);

    double mha_eps;
    params.param("epsilon_mha", mha_eps, 1.0);
    search->set_initial_mha_eps(mha_eps);

    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    params.param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    return std::move(search);
}

#if WORKING_DEV
auto MakeLARAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    auto forward_search = true;
    auto search = make_unique<LazyARAPlanner>(space, forward_search);
    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    bool search_mode;
    params.param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);
    return std::move(search);
}
#endif

#if WORKING_DEV
auto MakeEGWAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<ExperienceGraphPlanner>(space, heuristic);

    double epsilon;
    params.param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    return std::move(search);
}
#endif

#if WORKING_DEV
auto MakePADAStar(
    RobotPlanningSpace* space,
    RobotHeuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AdaptivePlanner>(space, heuristic);

    double epsilon_plan;
    params.param("epsilon_plan", epsilon_plan, 1.0);
    search->set_plan_eps(epsilon_plan);

    double epsilon_track;
    params.param("epsilon_track", epsilon_track, 1.0);
    search->set_track_eps(epsilon_track);

    AdaptivePlanner::TimeParameters tparams;
    tparams.planning.bounded = true;
    tparams.planning.improve = false;
    tparams.planning.type = ARAStar::TimeParameters::TIME;
    tparams.planning.max_allowed_time_init = clock::duration::zero();
    tparams.planning.max_allowed_time = clock::duration::zero();

    tparams.tracking.bounded = true;
    tparams.tracking.improve = false;
    tparams.tracking.type = ARAStar::TimeParameters::TIME;
    tparams.tracking.max_allowed_time_init = std::chrono::seconds(5);
    tparams.tracking.max_allowed_time = clock::duration::zero();

    search->set_time_parameters(tparams);

    return std::move(search);
}
#endif

} // namespace smpl

