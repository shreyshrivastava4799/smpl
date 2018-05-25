////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/ros/planner_interface.h>

// standard includes
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/planners/mhaplanner.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/angles.h>
#include <smpl/post_processing.h>
#include <smpl/time.h>
#include <smpl/types.h>

#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

#include <smpl/graph/adaptive_workspace_lattice.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/manip_lattice_egraph.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/workspace_lattice_egraph.h>

#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>

#include <smpl/search/adaptive_planner.h>
#include <smpl/search/arastar.h>
#include <smpl/search/experience_graph_planner.h>
#include <smpl/search/awastar.h>

namespace sbpl {
namespace motion {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

const char* PI_LOGGER = "simple";

// Return true if the variable is part of a multi-dof joint. Optionally store
// the name of the joint and the local name of the variable.
static
bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name = NULL,
    std::string* local_name = NULL)
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

struct ManipLatticeActionSpaceParams
{
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;
};

// Lookup parameters for ManipLatticeActionSpace, setting reasonable defaults
// for missing parameters. Return false if any required parameter is not found.
static
bool GetManipLatticeActionSpaceParams(
    ManipLatticeActionSpaceParams& params,
    const PlanningParams& pp)
{
    if (!pp.getParam("mprim_filename", params.mprim_filename)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return false;
    }

    pp.param("use_multiple_ik_solutions", params.use_multiple_ik_solutions, false);

    pp.param("use_xyz_snap_mprim", params.use_xyz_snap_mprim, false);
    pp.param("use_rpy_snap_mprim", params.use_rpy_snap_mprim, false);
    pp.param("use_xyzrpy_snap_mprim", params.use_xyzrpy_snap_mprim, false);
    pp.param("use_short_dist_mprims", params.use_short_dist_mprims, false);

    pp.param("xyz_snap_dist_thresh", params.xyz_snap_thresh, 0.0);
    pp.param("rpy_snap_dist_thresh", params.rpy_snap_thresh, 0.0);
    pp.param("xyzrpy_snap_dist_thresh", params.xyzrpy_snap_thresh, 0.0);
    pp.param("short_dist_mprims_thresh", params.short_dist_mprims_thresh, 0.0);
    return true;
}

template <class T>
static auto ParseMapFromString(const std::string& s)
    -> std::unordered_map<std::string, T>
{
    std::unordered_map<std::string, T> map;
    std::istringstream ss(s);
    std::string key;
    T value;
    while (ss >> key >> value) {
        map.insert(std::make_pair(key, value));
    }
    return map;
}

static
auto MakeManipLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }

    auto disc = ParseMapFromString<double>(disc_string);
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        std::string joint_name, local_name;
        if (IsMultiDOFJointVariable(vname, &joint_name, &local_name)) {
            // adjust variable name if a variable of a multi-dof joint
            auto mdof_vname = joint_name + "_" + local_name;
            auto dit = disc.find(mdof_vname);
            if (dit == end(disc)) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == end(disc)) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        }

        ROS_DEBUG_NAMED(PI_LOGGER, "resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr;
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLattice and
    // ManipLatticeActionSpace
    struct SimpleManipLattice : public ManipLattice {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLattice>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    if (grid) {
        space->setVisualizationFrameId(grid->getReferenceFrame());
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);

    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "Action Set:");
    for (auto ait = actions.begin(); ait != actions.end(); ++ait) {
        ROS_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_cstring(ait->type));
        if (ait->type == MotionPrimitive::SNAP_TO_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == MotionPrimitive::LONG_DISTANCE ||
            ait->type == MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "    action: " << ait->action);
        }
    }

    return std::move(space);
}

static
auto MakeManipLatticeEGraph(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }
    auto disc = ParseMapFromString<double>(disc_string);
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == end(disc)) {
            ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return nullptr;
        }
        resolutions[vidx] = dit->second;
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLatticeEgraph and
    // ManipLatticeActionSpace
    struct SimpleManipLatticeEgraph : public ManipLatticeEgraph {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLatticeEgraph>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR("Failed to initialize Manip Lattice Egraph");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR("Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);
    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    std::string egraph_path;
    if (params->getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->loadExperienceGraph(egraph_path);
    } else {
        ROS_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

static
bool GetWorkspaceLatticeParams(
    const PlanningParams* params,
    const RobotModel* robot,
    const RedundantManipulatorInterface* rmi,
    WorkspaceLatticeBase::Params* wsp)
{
    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return false;
    }

    auto disc = ParseMapFromString<double>(disc_string);
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu variables", disc.size());

    auto extract_disc = [&](const char* name, double* d)
    {
        auto it = disc.find(name);
        if (it == end(disc)) {
            ROS_ERROR("Missing discretization for variable '%s'", name);
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

static
auto MakeWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return NULL;
    }

    WorkspaceLatticeBase::Params wsp;
    if (!GetWorkspaceLatticeParams(params, robot, rmi, &wsp)) {
        return NULL;
    }

    auto space = make_unique<WorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return NULL;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

static
auto MakeWorkspaceLatticeEGraph(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice E-Graph");

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return NULL;
    }

    WorkspaceLatticeBase::Params wsp;
    if (!GetWorkspaceLatticeParams(params, robot, rmi, &wsp)) {
        return NULL;
    }

    auto space = make_unique<WorkspaceLatticeEGraph>();
    if (!space->init(robot, checker, params, wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    std::string egraph_path;
    if (params->getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->loadExperienceGraph(egraph_path);
    } else {
        ROS_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

static
auto MakeAdaptiveWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 36; //360;
    wsp.P_count = 19; //180 + 1;
    wsp.Y_count = 36; //360;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));

    auto space = make_unique<AdaptiveWorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp, grid)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    return std::move(space);
}

static
auto MakeMultiFrameBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<MultiFrameBfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }

    double offset_x, offset_y, offset_z;
    params.param("offset_x", offset_x, 0.0);
    params.param("offset_y", offset_y, 0.0);
    params.param("offset_z", offset_z, 0.0);
    h->setOffset(offset_x, offset_y, offset_z);

    return std::move(h);
}

static
auto MakeBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<BfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
};

static
auto MakeEuclidDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<EuclidDistHeuristic>();

    if (!h->init(space)) {
        return nullptr;
    }

    double wx, wy, wz, wr;
    params.param("x_coeff", wx, 1.0);
    params.param("y_coeff", wy, 1.0);
    params.param("z_coeff", wz, 1.0);
    params.param("rot_coeff", wr, 1.0);

    h->setWeightX(wx);
    h->setWeightY(wx);
    h->setWeightZ(wx);
    h->setWeightRot(wx);
    return std::move(h);
};

static
auto MakeJointDistHeuristic(RobotPlanningSpace* space)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<JointDistHeuristic>();
    if (!h->init(space)) {
        return nullptr;
    }
    return std::move(h);
};

static
auto MakeDijkstraEgraphHeuristic3D(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<DijkstraEgraphHeuristic3D>();

//    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);

    return std::move(h);
};

static
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
        return nullptr;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);
    return std::move(h);
};

static
auto MakeARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<ARAStar>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    return std::move(search);
}

static
auto MakeAWAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AWAStar>(space, heuristic);
    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    return std::move(search);
}

static
auto MakeMHAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
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
    space->params()->param("epsilon_mha", mha_eps, 1.0);
    search->set_initial_mha_eps(mha_eps);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    return std::move(search);
}

static
auto MakeLARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto forward_search = true;
    auto search = make_unique<LazyARAPlanner>(space, forward_search);
    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);
    return std::move(search);
}

static
auto MakeEGWAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<ExperienceGraphPlanner>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    return std::move(search);
}

static
auto MakePADAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AdaptivePlanner>(space, heuristic);

    double epsilon_plan;
    space->params()->param("epsilon_plan", epsilon_plan, 1.0);
    search->set_plan_eps(epsilon_plan);

    double epsilon_track;
    space->params()->param("epsilon_track", epsilon_track, 1.0);
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

static
bool HasVisibilityConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.visibility_constraints.empty();
}

static
bool HasJointConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.joint_constraints.empty();
}

static
bool HasPositionConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.position_constraints.empty();
}

static
bool HasOrientationConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.orientation_constraints.empty();
}

// a set of constraints representing a goal pose for a single link
static
bool IsPoseConstraint(const moveit_msgs::Constraints& constraints)
{
    return !HasVisibilityConstraints(constraints) &&
            !HasJointConstraints(constraints) &&
            HasPositionConstraints(constraints) &&
            HasOrientationConstraints(constraints) &&
            constraints.position_constraints.size() == 1 &&
            constraints.orientation_constraints.size() == 1 &&
            constraints.position_constraints.front().link_name ==
                    constraints.orientation_constraints.front().link_name;

};

// a set of constraints representing a goal position for a single link
static
bool IsPositionConstraint(const moveit_msgs::Constraints& constraints)
{
    return !HasVisibilityConstraints(constraints) &&
            !HasJointConstraints(constraints) &&
            HasPositionConstraints(constraints) &&
            !HasOrientationConstraints(constraints) &&
            constraints.position_constraints.size() == 1;
};

// a set of constraints representing a joint state goal
static
bool IsJointStateConstraint(const moveit_msgs::Constraints& constraints)
{
    return !HasVisibilityConstraints(constraints) &&
        HasJointConstraints(constraints) &&
        !HasPositionConstraints(constraints) &&
        !HasOrientationConstraints(constraints);
};

// a set of constraints representing multiple potential goal poses for a
// single link
static
bool IsMultiPoseGoal(const GoalConstraints& goal_constraints)
{
    // check multiple constraints
    if (goal_constraints.size() < 2) {
        return false;
    }

    // check all pose constraints
    for (auto& constraints : goal_constraints) {
        if (!IsPoseConstraint(constraints)) {
            return false;
        }
    }

    // check all pose constraints for the same link
    auto& link_name = goal_constraints.front().position_constraints.front().link_name;
    for (auto& constraints : goal_constraints) {
        if (link_name != constraints.position_constraints.front().link_name) {
            return false;
        }
    }

    // TODO: assert that target point on the link is the same for all constraints?

    return true;
}

static
bool IsPoseGoal(const GoalConstraints& goal_constraints)
{
    return goal_constraints.size() == 1 &&
            IsPoseConstraint(goal_constraints.front());
}

static
bool IsPositionGoal(const GoalConstraints& goal_constraints)
{
    return goal_constraints.size() == 1 &&
         IsPositionConstraint(goal_constraints.front());
}

static
bool IsJointStateGoal(const GoalConstraints& goal_constraints)
{
    return goal_constraints.size() == 1 &&
            IsJointStateConstraint(goal_constraints.front());
}

PlannerInterface::PlannerInterface(
    RobotModel* robot,
    CollisionChecker* checker,
    OccupancyGrid* grid)
:
    m_robot(robot),
    m_checker(checker),
    m_grid(grid),
    m_fk_iface(nullptr),
    m_params(),
    m_initialized(false),
    m_pspace(),
    m_heuristics(),
    m_planner(),
    m_sol_cost(INFINITECOST),
    m_planner_id()
{
    if (m_robot) {
        m_fk_iface = m_robot->getExtension<ForwardKinematicsInterface>();
    }

    ////////////////////////////////////
    // Setup Planning Space Factories //
    ////////////////////////////////////

    m_space_factories["manip"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLattice(m_grid, r, c, p);
    };

    m_space_factories["manip_lattice_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLatticeEGraph(m_grid, r, c, p);
    };

    m_space_factories["workspace"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeWorkspaceLattice(m_grid, r, c, p);
    };

    m_space_factories["workspace_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeWorkspaceLatticeEGraph(m_grid, r, c, p);
    };

    m_space_factories["adaptive_workspace_lattice"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeAdaptiveWorkspaceLattice(m_grid, r, c, p);
    };

    ///////////////////////////////
    // Setup Heuristic Factories //
    ///////////////////////////////

    m_heuristic_factories["mfbfs"] = [this](RobotPlanningSpace* space) {
        return MakeMultiFrameBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["bfs"] = [this](RobotPlanningSpace* space) {
        return MakeBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["euclid"] = [this](RobotPlanningSpace* space) {
        return MakeEuclidDistHeuristic(space, m_params);
    };

    m_heuristic_factories["joint_distance"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistHeuristic(space);
    };

    m_heuristic_factories["bfs_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeDijkstraEgraphHeuristic3D(space, m_grid, m_params);
    };

    m_heuristic_factories["joint_distance_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistEGraphHeuristic(space, m_params);
    };

    /////////////////////////////
    // Setup Planner Factories //
    /////////////////////////////

    m_planner_factories["arastar"] = MakeARAStar;
    m_planner_factories["awastar"] = MakeAWAStar;
    m_planner_factories["mhastar"] = MakeMHAStar;
    m_planner_factories["larastar"] = MakeLARAStar;
    m_planner_factories["egwastar"] = MakeEGWAStar;
    m_planner_factories["padastar"] = MakePADAStar;
}

PlannerInterface::~PlannerInterface()
{
}

bool PlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO_NAMED(PI_LOGGER, "initialize arm planner interface");

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Frame: %s", params.planning_frame.c_str());

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Link Sphere Radius: %0.3f", params.planning_link_sphere_radius);

    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");

    if (!checkConstructionArgs()) {
        return false;
    }

    if (!checkParams(params)) {
        return false;
    }

    m_params = params;

    m_grid->setReferenceFrame(m_params.planning_frame);

    m_initialized = true;

    ROS_INFO_NAMED(PI_LOGGER, "initialized arm planner interface");
    return m_initialized;
}

bool PlannerInterface::checkConstructionArgs() const
{
    if (!m_robot) {
        ROS_ERROR("Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_checker) {
        ROS_ERROR("Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_grid) {
        ROS_ERROR("Occupancy Grid given to Arm Planner Interface must be non-null");
        return false;
    }

    return true;
}

bool PlannerInterface::solve(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // TODO: lazily reinitialize planner when algorithm changes
    if (!reinitPlanner(req.planner_id)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    res.trajectory_start = planning_scene.robot_state;
    ROS_INFO_NAMED(PI_LOGGER, "Allowed Time (s): %0.3f", req.allowed_planning_time);

    auto then = clock::now();

    if (!setGoal(req.goal_constraints)) {
        ROS_ERROR("Failed to set goal");
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    std::vector<RobotState> path;
    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    ROS_DEBUG_NAMED(PI_LOGGER, "planner path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    /////////////////////
    // smooth the path //
    /////////////////////

    postProcessPath(path);
    SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));

    ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    convertJointVariablePathToJointTrajectory(path, res.trajectory);
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    if (!m_params.plan_output_dir.empty()) {
        writePath(res.trajectory_start, res.trajectory);
    }

    profilePath(res.trajectory.joint_trajectory);
//    removeZeroDurationSegments(traj);

    res.planning_time = to_seconds(clock::now() - then);
    return true;
}

bool PlannerInterface::checkParams(
    const PlanningParams& params) const
{
    if (params.planning_frame.empty()) {
        return false;
    }

    // TODO: check for existence of planning joints in robot model

    if (params.cost_per_cell < 0) {
        return false;
    }

    return true;
}

static
bool ExtractJointStateGoal(
    RobotModel* model,
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    auto& goal_constraints = v_goal_constraints.front();

    ROS_INFO_NAMED(PI_LOGGER, "Set goal configuration");

    RobotState sbpl_angle_goal(model->jointVariableCount(), 0);
    RobotState sbpl_angle_tolerance(model->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < model->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "Insufficient joint constraints specified (%zu < %zu)!", goal_constraints.joint_constraints.size(), model->jointVariableCount());
//        return false;
    }
    if (goal_constraints.joint_constraints.size() > model->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "Excess joint constraints specified (%zu > %zu)!", goal_constraints.joint_constraints.size(), model->jointVariableCount());
//        return false;
    }

    for (size_t i = 0; i < model->jointVariableCount(); ++i) {
        auto& variable_name = model->getPlanningJoints()[i];

        auto jit = std::find_if(
                begin(goal_constraints.joint_constraints),
                end(goal_constraints.joint_constraints),
                [&](const moveit_msgs::JointConstraint& constraint) {
                    return constraint.joint_name == variable_name;
                });
        if (jit == end(goal_constraints.joint_constraints)) {
            ROS_WARN("Assume goal position 1 for joint '%s'", variable_name.c_str());
            sbpl_angle_goal[i] = 1.0;
            sbpl_angle_tolerance[i] = 0.1;
        } else {
            sbpl_angle_goal[i] = jit->position;
            sbpl_angle_tolerance[i] = std::min(
                    std::fabs(jit->tolerance_above), std::fabs(jit->tolerance_below));
            ROS_INFO_NAMED(PI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, variable_name.c_str(), sbpl_angle_goal[i], sbpl_angle_tolerance[i]);
        }
    }

    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;

    auto* fk_iface = model->getExtension<ForwardKinematicsInterface>();

    // TODO: really need to reevaluate the necessity of the planning link
    if (fk_iface) {
        goal.pose = fk_iface->computeFK(goal.angles);
    } else {
        goal.pose = Eigen::Affine3d::Identity();
    }

    return true;
}

static
bool ExtractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Affine3d& goal_pose)
{
    assert(!constraints.position_constraints.empty() &&
            constraints.orientation_constraints.empty());

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?

    auto& position_constraint = constraints.position_constraints.front();
    auto& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    auto& bounding_primitive = position_constraint.constraint_region.primitives.front();
    auto& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Affine3d T_planning_eef; // T_planning_off * T_off_eef;
    tf::poseMsgToEigen(primitive_pose, T_planning_eef);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;
    return true;
}

static
bool ExtractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double tol[6])
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front().constraint_region.primitives.empty())
    {
        auto& position_constraint = goal_constraints.position_constraints.front();
        auto& constraint_primitive = position_constraint.constraint_region.primitives.front();
        auto& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    } else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        auto& orientation_constraints = goal_constraints.orientation_constraints;
        auto& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    } else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

static
bool ExtractPositionGoal(
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    return false;
}

static
bool ExtractPoseGoal(
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    assert(!v_goal_constraints.empty());
    auto& goal_constraints = v_goal_constraints.front();

    ROS_INFO_NAMED(PI_LOGGER, "Setting goal position");

    Eigen::Affine3d goal_pose;
    if (!ExtractGoalPoseFromGoalConstraints(goal_constraints, goal_pose)) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    goal.type = GoalType::XYZ_RPY_GOAL;
    goal.pose = goal_pose;

    double sbpl_tolerance[6] = { 0.0 };
    if (!ExtractGoalToleranceFromGoalConstraints(goal_constraints, sbpl_tolerance)) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal.xyz_tolerance[0] = sbpl_tolerance[0];
    goal.xyz_tolerance[1] = sbpl_tolerance[1];
    goal.xyz_tolerance[2] = sbpl_tolerance[2];
    goal.rpy_tolerance[0] = sbpl_tolerance[3];
    goal.rpy_tolerance[1] = sbpl_tolerance[4];
    goal.rpy_tolerance[2] = sbpl_tolerance[5];
    return true;
}

static
bool ExtractPosesGoal(
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    goal.type = GoalType::MULTIPLE_POSE_GOAL;
    goal.poses.reserve(v_goal_constraints.size());
    for (size_t i = 0; i < v_goal_constraints.size(); ++i) {
        auto& constraints = v_goal_constraints[i];
        Eigen::Affine3d goal_pose;
        if (!ExtractGoalPoseFromGoalConstraints(constraints, goal_pose)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
            return false;
        }

        goal.poses.push_back(goal_pose);

        if (i == 0) { // only use tolerances from the first set of constraints
            double sbpl_tolerance[6] = { 0.0 };
            if (!ExtractGoalToleranceFromGoalConstraints(constraints, sbpl_tolerance)) {
                ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
                return false;
            }

            goal.xyz_tolerance[0] = sbpl_tolerance[0];
            goal.xyz_tolerance[1] = sbpl_tolerance[1];
            goal.xyz_tolerance[2] = sbpl_tolerance[2];
            goal.rpy_tolerance[0] = sbpl_tolerance[3];
            goal.rpy_tolerance[1] = sbpl_tolerance[4];
            goal.rpy_tolerance[2] = sbpl_tolerance[5];

            // TODO: see frustration in ExtractJointStateGoal
            goal.pose = goal.poses[0];
        }
    }

    return true;
}

// Convert the set of input goal constraints to an SMPL goal type and update
// the goal within the graph, the heuristic, and the search.
bool PlannerInterface::setGoal(const GoalConstraints& v_goal_constraints)
{
    GoalConstraint goal;

    if (IsPoseGoal(v_goal_constraints)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to pose!");
        if (!ExtractPoseGoal(v_goal_constraints, goal)) {
            ROS_ERROR("Failed to set goal position");
            return false;
        }

        ROS_INFO_NAMED(PI_LOGGER, "New Goal");
        ROS_INFO_NAMED(PI_LOGGER, "    frame: %s", m_params.planning_frame.c_str());
        double yaw, pitch, roll;
        angles::get_euler_zyx(goal.pose.rotation(), yaw, pitch, roll);
        ROS_INFO_NAMED(PI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, Y: %0.3f, P: %0.3f, R: %0.3f)", goal.pose.translation()[0], goal.pose.translation()[1], goal.pose.translation()[2], yaw, pitch, roll);
        ROS_INFO_NAMED(PI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2], goal.rpy_tolerance[0], goal.rpy_tolerance[1], goal.rpy_tolerance[2]);
    } else if (IsJointStateGoal(v_goal_constraints)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to joint configuration!");
        if (!ExtractJointStateGoal(m_robot, v_goal_constraints, goal)) {
            ROS_ERROR("Failed to set goal configuration");
            return false;
        }
    } else if (IsPositionGoal(v_goal_constraints)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to position!");
        if (!ExtractPositionGoal(v_goal_constraints, goal)) {
            return false;
        }
    } else if (IsMultiPoseGoal(v_goal_constraints)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to multiple goal poses!");
        if (!ExtractPosesGoal(v_goal_constraints, goal)) {
            return false;
        }
        ROS_INFO_NAMED(PI_LOGGER, "  frame: %s", m_params.planning_frame.c_str());
        for (auto& pose : goal.poses) {
            double yaw, pitch, roll;
            angles::get_euler_zyx(pose.rotation(), yaw, pitch, roll);
            ROS_INFO_NAMED(PI_LOGGER, "  pose: (x: %0.3f, y: %0.3f, z: %0.3f, Y: %0.3f, P: %0.3f, R: %0.3f)", pose.translation()[0], pose.translation()[1], pose.translation()[2], yaw, pitch, roll);
        }
        ROS_INFO_NAMED(PI_LOGGER, "  tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2], goal.rpy_tolerance[0], goal.rpy_tolerance[1], goal.rpy_tolerance[2]);
    } else {
        ROS_ERROR("Unknown goal type!");
        return false;
    }

    // set sbpl environment goal
    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    // set planner goal
    auto goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

// Convert the input robot state to an SMPL robot state and update the start
// state in the graph, heuristic, and search.
bool PlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    ROS_INFO_NAMED(PI_LOGGER, "set start configuration");

    // TODO: Ideally, the RobotModel should specify joints rather than variables
    if (!state.multi_dof_joint_state.joint_names.empty()) {
        auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (auto& joint_name : m_robot->getPlanningJoints()) {
            auto it = std::find(begin(mdof_joint_names), end(mdof_joint_names), joint_name);
            if (it != end(mdof_joint_names)) {
                ROS_WARN_NAMED(PI_LOGGER, "planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            m_robot->getPlanningJoints(),
            initial_positions,
            missing))
    {
        ROS_WARN_STREAM("start state is missing planning joints: " << missing);

        moveit_msgs::RobotState fixed_state = state;
        for (auto& variable : missing) {
            ROS_WARN("  Assume position 0.0 for joint variable '%s'", variable.c_str());
            fixed_state.joint_state.name.push_back(variable);
            fixed_state.joint_state.position.push_back(0.0);
        }

        if (!leatherman::getJointPositions(
                fixed_state.joint_state,
                fixed_state.multi_dof_joint_state,
                m_robot->getPlanningJoints(),
                initial_positions,
                missing))
        {
            return false;
        }

//        return false;
    }

    ROS_INFO_STREAM_NAMED(PI_LOGGER, "  joint variables: " << initial_positions);

    if (!m_pspace->setStart(initial_positions)) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    auto start_id = m_pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (m_planner->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    return true;
}

bool PlannerInterface::plan(double allowed_time, std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario
    SV_SHOW_DEBUG_NAMED("bfs_walls", getBfsWallsVisualization());
    SV_SHOW_DEBUG_NAMED("bfs_values", getBfsValuesVisualization());

    ROS_WARN_NAMED(PI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    b_ret = m_planner->replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    return b_ret;
}

/// Test if a particular set of goal constraints it supported.
///
/// This tests whether, in general, any planning algorithm supported by this
/// interface can support a particular set of constraints. Certain planning
/// algorithms may not be able to handle a given set of constraints. This
/// method also cannot check for validity of constraints against a particular
/// robot model at this step. In particular, it cannot assert that the a set
/// of joint constraints lists a constraint for each joint, which is currently
/// required.
bool PlannerInterface::SupportsGoalConstraints(
    const GoalConstraints& goal_constraints,
    std::string& why)
{
    if (goal_constraints.empty()) {
        return true;
    }

    if (!(IsMultiPoseGoal(goal_constraints) ||
            IsPoseGoal(goal_constraints) ||
            IsPositionGoal(goal_constraints) ||
            IsJointStateGoal(goal_constraints)))
    {
        why = "goal constraints are not supported";
        return false;
    }

    return true;
}

bool PlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        ROS_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    std::string why;
    if (!SupportsGoalConstraints(req.goal_constraints, why)) {
        ROS_ERROR("Goal constraints not supported (%s)", why.c_str());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    return true;
}

auto PlannerInterface::getPlannerStats() -> std::map<std::string, double>
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = m_planner->get_initial_eps_planning_time();
    stats["initial epsilon"] = m_planner->get_initial_eps();
    stats["initial solution expansions"] = m_planner->get_n_expands_init_solution();
    stats["final epsilon planning time"] = m_planner->get_final_eps_planning_time();
    stats["final epsilon"] = m_planner->get_final_epsilon();
    stats["solution epsilon"] = m_planner->get_solution_eps();
    stats["expansions"] = m_planner->get_n_expands();
    stats["solution cost"] = m_sol_cost;
    return stats;
}

auto PlannerInterface::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_checker->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            auto r = 0.1f;
            auto g = cinc * (float)(path.size() - (i + 1));
            auto b = cinc * (float)i;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory";
        marker.id = i;
    }

    return ma;
}

auto PlannerInterface::getBfsValuesVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getValuesVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getValuesVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getValuesVisualization();
    } else {
        return visual::Marker{ };
    }
}

auto PlannerInterface::getBfsWallsVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getWallsVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getWallsVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getWallsVisualization();
    } else {
        return visual::Marker{ };
    }
}

void PlannerInterface::clearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
//    res.trajectory_start.joint_state;
//    res.trajectory_start.multi_dof_joint_state;
//    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = req.group_name;
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

bool PlannerInterface::parsePlannerID(
    const std::string& planner_id,
    std::string& space_name,
    std::string& heuristic_name,
    std::string& search_name) const
{
    boost::regex alg_regex("(\\w+)(?:\\.(\\w+))?(?:\\.(\\w+))?");

    boost::smatch sm;

    ROS_INFO("Match planner id '%s' against regex '%s'", planner_id.c_str(), alg_regex.str().c_str());
    if (!boost::regex_match(planner_id, sm, alg_regex)) {
        return false;
    }

    auto default_search_name = "arastar";
    auto default_heuristic_name = "bfs";
    auto default_space_name = "manip";

    if (sm.size() < 2 || sm[1].str().empty()) {
        search_name = default_search_name;
    } else {
        search_name = sm[1];
    }

    if (sm.size() < 3 || sm[2].str().empty()) {
        heuristic_name = default_heuristic_name;
    } else {
        heuristic_name = sm[2];
    }

    if (sm.size() < 4 || sm[3].str().empty()) {
        space_name = default_space_name;
    } else {
        space_name = sm[3];
    }

    return true;
}

void PlannerInterface::clearGraphStateToPlannerStateMap()
{
    if (!m_pspace) {
        return;
    }

    std::vector<int*>& state_id_to_index = m_pspace->StateID2IndexMapping;
    for (int* mapping : state_id_to_index) {
        for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
            mapping[i] = -1;
        }
    }
}

bool PlannerInterface::reinitPlanner(const std::string& planner_id)
{
    if (planner_id == m_planner_id) {
        // TODO: check for specification of default planning components when
        // they may not have been previously specified
        return true;
    }

    ROS_INFO_NAMED(PI_LOGGER, "Initialize planner");

    std::string search_name;
    std::string heuristic_name;
    std::string space_name;
    if (!parsePlannerID(planner_id, space_name, heuristic_name, search_name)) {
        ROS_ERROR("Failed to parse planner setup");
        return false;
    }

    ROS_INFO_NAMED(PI_LOGGER, " -> Planning Space: %s", space_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Heuristic: %s", heuristic_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Search: %s", search_name.c_str());

    auto psait = m_space_factories.find(space_name);
    if (psait == end(m_space_factories)) {
        ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    m_pspace = psait->second(m_robot, m_checker, &m_params);
    if (!m_pspace) {
        ROS_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == end(m_heuristic_factories)) {
        ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace.get());
    if (!heuristic) {
        ROS_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics.clear();
    m_heuristics.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (auto& entry : m_heuristics) {
        m_pspace->insertHeuristic(entry.second.get());
    }

    auto pait = m_planner_factories.find(search_name);
    if (pait == end(m_planner_factories)) {
        ROS_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics);
    m_planner = pait->second(m_pspace.get(), first_heuristic->second.get());
    if (!m_planner) {
        ROS_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    m_planner_id = planner_id;
    return true;
}

void PlannerInterface::profilePath(trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    auto& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev_point = traj.points[i - 1];
        auto& curr_point = traj.points[i];

        // find the maximum time required for any joint to reach the next
        // waypoint
        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            auto from_pos = prev_point.positions[jidx];
            auto to_pos = curr_point.positions[jidx];
            auto vel = m_robot->velLimit(jidx);
            if (vel <= 0.0) {
                continue;
            }
            auto t = 0.0;
            if (m_robot->isContinuous(jidx)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }

            max_time = std::max(max_time, t);
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }
}

void PlannerInterface::removeZeroDurationSegments(
    trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    size_t end_idx = 1; // current end of the non-filtered range
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev = traj.points[end_idx - 1];
        auto& curr = traj.points[i];
        if (curr.time_from_start != prev.time_from_start) {
            ROS_INFO("Move index %zu into %zu", i, end_idx);
            if (end_idx != i) {
                traj.points[end_idx] = std::move(curr);
            }
            end_idx++;
        }
    }
    traj.points.resize(end_idx);
}

bool PlannerInterface::isPathValid(const std::vector<RobotState>& path) const
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (!m_checker->isStateToStateValid(path[i - 1], path[i])) {
            ROS_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

void PlannerInterface::postProcessPath(std::vector<RobotState>& path) const
{
    auto check_planned_path = true;
    if (check_planned_path && !isPathValid(path)) {
        ROS_ERROR("Planned path is invalid");
    }

    // shortcut path
    if (m_params.shortcut_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate planned path with %zu waypoints before shortcutting.", path.size());
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        } else {
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        }
    }

    // interpolate path
    if (m_params.interpolate_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate trajectory");
        }
    }
}

void PlannerInterface::convertJointVariablePathToJointTrajectory(
    const std::vector<RobotState>& path,
    moveit_msgs::RobotTrajectory& traj) const
{
    ROS_INFO("Convert Variable Path to Robot Trajectory");

    traj.joint_trajectory.header.frame_id = m_params.planning_frame;
    traj.multi_dof_joint_trajectory.header.frame_id = m_params.planning_frame;

    traj.joint_trajectory.joint_names.clear();
    traj.joint_trajectory.points.clear();
    traj.multi_dof_joint_trajectory.joint_names.clear();
    traj.multi_dof_joint_trajectory.points.clear();

    // fill joint names header for both single- and multi-dof joint trajectories
    auto& variable_names = m_robot->getPlanningJoints();
    for (auto& var_name : variable_names) {
        std::string joint_name;
        if (IsMultiDOFJointVariable(var_name, &joint_name)) {
            auto it = std::find(
                    begin(traj.multi_dof_joint_trajectory.joint_names),
                    end(traj.multi_dof_joint_trajectory.joint_names),
                    joint_name);
            if (it == end(traj.multi_dof_joint_trajectory.joint_names)) {
                // avoid duplicates
                traj.multi_dof_joint_trajectory.joint_names.push_back(joint_name);
            }
        } else {
            traj.joint_trajectory.joint_names.push_back(var_name);
        }
    }

    ROS_INFO("  Path includes %zu single-dof joints and %zu multi-dof joints",
            traj.joint_trajectory.joint_names.size(),
            traj.multi_dof_joint_trajectory.joint_names.size());

    // empty or number of points in the path
    if (!traj.joint_trajectory.joint_names.empty()) {
        traj.joint_trajectory.points.resize(path.size());
    }
    // empty or number of points in the path
    if (!traj.multi_dof_joint_trajectory.joint_names.empty()) {
        traj.multi_dof_joint_trajectory.points.resize(path.size());
    }

    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];

        for (size_t vidx = 0; vidx < point.size(); ++vidx) {
            auto& var_name = variable_names[vidx];

            std::string joint_name, local_name;
            if (IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto& p = traj.multi_dof_joint_trajectory.points[pidx];
                p.transforms.resize(traj.multi_dof_joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.multi_dof_joint_trajectory.joint_names),
                        end(traj.multi_dof_joint_trajectory.joint_names),
                        joint_name);
                if (it == end(traj.multi_dof_joint_trajectory.joint_names)) continue;

                auto tidx = std::distance(begin(traj.multi_dof_joint_trajectory.joint_names), it);

                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    p.transforms[tidx].translation.x = point[vidx];
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    p.transforms[tidx].translation.y = point[vidx];
                } else if (local_name == "trans_z") {
                    p.transforms[tidx].translation.z = point[vidx];
                } else if (local_name == "theta") {
                    Eigen::Quaterniond q(Eigen::AngleAxisd(point[vidx], Eigen::Vector3d::UnitZ()));
                    tf::quaternionEigenToMsg(q, p.transforms[tidx].rotation);
                } else if (local_name == "rot_w") {
                    p.transforms[tidx].rotation.w = point[vidx];
                } else if (local_name == "rot_x") {
                    p.transforms[tidx].rotation.x = point[vidx];
                } else if (local_name == "rot_y") {
                    p.transforms[tidx].rotation.y = point[vidx];
                } else if (local_name == "rot_z") {
                    p.transforms[tidx].rotation.z = point[vidx];
                } else {
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }
            } else {
                auto& p = traj.joint_trajectory.points[pidx];
                p.positions.resize(traj.joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.joint_trajectory.joint_names),
                        end(traj.joint_trajectory.joint_names),
                        var_name);
                if (it == end(traj.joint_trajectory.joint_names)) continue;

                auto posidx = std::distance(begin(traj.joint_trajectory.joint_names), it);

                p.positions[posidx] = point[vidx];
            }
        }
    }
}

bool PlannerInterface::writePath(
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj) const
{
    boost::filesystem::path p(m_params.plan_output_dir);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory", m_params.plan_output_dir.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
        auto& var_name = m_robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != m_robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    auto wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            auto& wp = traj.joint_trajectory.points[widx];
            auto joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.joint_trajectory.joint_names[jidx];
                auto vp = wp.positions[jidx];
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        joint_name);
                if (it != end(state.joint_state.name)) {
                    auto tvidx = std::distance(begin(state.joint_state.name), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            auto& wp = traj.multi_dof_joint_trajectory.points[widx];
            auto joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.multi_dof_joint_trajectory.joint_names[jidx];
                auto& t = wp.transforms[jidx];
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it != end(state.multi_dof_joint_state.joint_names)) {
                    size_t tvidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
            auto& var_name = m_robot->getPlanningJoints()[vidx];

            std::string joint_name, local_name;
            if (IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it == end(state.multi_dof_joint_state.joint_names)) continue;

                auto jidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                auto& transform = state.multi_dof_joint_state.transforms[jidx];
                double pos;
                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    pos = transform.translation.x;
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    pos = transform.translation.y;
                } else if (local_name == "trans_z") {
                    pos = transform.translation.z;
                } else if (local_name == "theta") {
                    // this list just gets larger:
                    // from sbpl_collision_checking, MoveIt, Bullet, leatherman
                    double s_squared = 1.0 - transform.rotation.w * transform.rotation.w;
                    if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
                        pos = 0.0;
                    } else {
                        double s = 1.0 / sqrt(s_squared);
                        pos = (acos(transform.rotation.w) * 2.0) * transform.rotation.x * s;
                    }
                } else if (local_name == "rot_w") {
                    pos = transform.rotation.w;
                } else if (local_name == "rot_x") {
                    pos = transform.rotation.x;
                } else if (local_name == "rot_y") {
                    pos = transform.rotation.y;
                } else if (local_name == "rot_z") {
                    pos = transform.rotation.z;
                } else {
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }

                ofs << pos;
            } else {
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        var_name);
                if (it == end(state.joint_state.name)) continue;

                auto tvidx = std::distance(begin(state.joint_state.name), it);
                auto vp = state.joint_state.position[tvidx];
                ofs << vp;
            }

            if (vidx != m_robot->jointVariableCount() - 1) {
                ofs << ',';
            }
        }
        ofs << '\n';
    }

    return true;
}

} // namespace motion
} // namespace sbpl
