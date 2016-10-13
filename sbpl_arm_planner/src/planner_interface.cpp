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

#include <sbpl_arm_planner/planner_interface.h>

// standard includes
#include <assert.h>

// system includes
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <sbpl_arm_planner/angles.h>
#include <sbpl_arm_planner/bfs_heuristic.h>
#include <sbpl_arm_planner/euclid_dist_heuristic.h>
#include <sbpl_arm_planner/joint_dist_heuristic.h>
#include <sbpl_arm_planner/manip_lattice.h>
#include <sbpl_arm_planner/manip_lattice_action_space.h>
#include <sbpl_arm_planner/multi_frame_bfs_heuristic.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/post_processing.h>
#include <sbpl_arm_planner/workspace_lattice.h>
#include <sbpl_arm_planner/visualize.h>

namespace sbpl {
namespace manip {

const char* PI_LOGGER = "simple";

PlannerInterface::PlannerInterface(
    RobotModel* robot,
    CollisionChecker* checker,
    OccupancyGrid* grid)
:
    m_robot(robot),
    m_checker(checker),
    m_grid(grid),
    m_params(),
    m_initialized(false),
    m_pspace(),
    m_aspace(),
    m_heuristics(),
    m_planner(),
    m_heur_vec(),
    m_sol_cost(INFINITECOST),
    m_planner_id(),
    m_req(),
    m_res()
{
}

PlannerInterface::~PlannerInterface()
{
}

bool PlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO_NAMED(PI_LOGGER, "initialize arm planner interface");

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Frame: %s", params.planning_frame.c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Coord Values: %s", to_string(params.coord_vals).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Coord Deltas: %s", to_string(params.coord_delta).c_str());

    ROS_INFO_NAMED(PI_LOGGER, "  Use Multiple IK Solutions: %s", params.use_multiple_ik_solutions ? "true" : "false");

    ROS_INFO_NAMED(PI_LOGGER, "  Use BFS Heuristic: %s", params.use_bfs_heuristic ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Planning Link Sphere Radius: %0.3f", params.planning_link_sphere_radius);

    ROS_INFO_NAMED(PI_LOGGER, "  Epsilon: %0.3f", params.epsilon);
    ROS_INFO_NAMED(PI_LOGGER, "  Allowed Time: %0.3f", params.allowed_time);
    ROS_INFO_NAMED(PI_LOGGER, "  Search Mode: %s", params.search_mode ? "true" : "false");

    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Waypoint Time: %0.3f", params.waypoint_time);

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

    m_req = req; // record the last attempted request

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

    // plan
    res.trajectory_start = planning_scene.robot_state;
    m_params.allowed_time = req.allowed_planning_time;
    ROS_INFO_NAMED(PI_LOGGER, "Allowed Time (s): %0.3f", m_params.allowed_time);

    auto then = std::chrono::high_resolution_clock::now();

    std::vector<RobotState> path;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to position!");
        if (!planToPose(req, path, res)) {
            auto now = std::chrono::high_resolution_clock::now();
            res.planning_time = std::chrono::duration<double>(now - then).count();
            return false;
        }
    } else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to joint configuration!");
        if (!planToConfiguration(req, path, res)) {
            auto now = std::chrono::high_resolution_clock::now();
            res.planning_time = std::chrono::duration<double>(now - then).count();
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
        auto now = std::chrono::high_resolution_clock::now();
        res.planning_time = std::chrono::duration<double>(now - then).count();
        return false;
    }

    postProcessPath(path, res.trajectory.joint_trajectory);
    visualizePath(res.trajectory_start, res.trajectory);

    auto now = std::chrono::high_resolution_clock::now();
    res.planning_time = std::chrono::duration<double>(now - then).count();
    m_res = res; // record the last result
    return true;
}

bool PlannerInterface::checkParams(
    const PlanningParams& params) const
{
    if (params.allowed_time < 0.0) {
        return false;
    }

    if (params.waypoint_time < 0.0) {
        return false;
    }

    // TODO: check for frame in robot model?
    if (params.planning_frame.empty()) {
        return false;
    }

    // TODO: check for existence of planning joints in robot model

    if (params.epsilon < 1.0) {
        return false;
    }

    if (m_robot->jointVariableCount() != (int)params.coord_vals.size()) {
        return false;
    }

    if (m_robot->jointVariableCount() != (int)params.coord_delta.size()) {
        return false;
    }

    if (params.cost_multiplier < 0) {
        return false;
    }

    if (params.cost_per_cell < 0) {
        return false;
    }

    if (params.cost_per_meter < 0) {
        return false;
    }

    if (params.cost_per_second < 0) {
        return false;
    }

    if (params.time_per_cell < 0.0) {
        return false;
    }

    return true;
}

bool PlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    ROS_INFO_NAMED(PI_LOGGER, "set start configuration");

    if (!state.multi_dof_joint_state.joint_names.empty()) {
        const auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (const std::string& joint_name : m_robot->getPlanningJoints()) {
            auto it = std::find(mdof_joint_names.begin(), mdof_joint_names.end(), joint_name);
            if (it != mdof_joint_names.end()) {
                ROS_WARN_NAMED(PI_LOGGER, "planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state, m_robot->getPlanningJoints(), initial_positions, missing))
    {
        ROS_ERROR("start state is missing planning joints: %s", to_string(missing).c_str());
        return false;
    }

    ROS_INFO_NAMED(PI_LOGGER, "  joint variables: %s", to_string(initial_positions).c_str());

    if (m_pspace->setStart(initial_positions) == 0) {
        ROS_ERROR("environment failed to set start state. not planning.");
        return false;
    }

    const int start_id = m_pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("no start state has been set");
        return false;
    }

    if (m_planner->set_start(start_id) == 0) {
        ROS_ERROR("failed to set start state. not planning.");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalConfiguration(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Set goal configuration");

    std::vector<double> sbpl_angle_goal(m_robot->jointVariableCount(), 0);
    std::vector<double> sbpl_angle_tolerance(m_robot->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "All %zu arm joint constraints must be specified for goal!", m_robot->jointVariableCount());
        return false;
    }
    if (goal_constraints.joint_constraints.size() > m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "%d joint constraints specified! Using the first %zu!", (int)goal_constraints.joint_constraints.size(), m_robot->jointVariableCount());
        return false;
    }

    const size_t num_angle_constraints = std::min(
            goal_constraints.joint_constraints.size(), sbpl_angle_goal.size());
    for (size_t i = 0; i < num_angle_constraints; i++) {
        const auto& joint_constraint = goal_constraints.joint_constraints[i];
        sbpl_angle_goal[i] = joint_constraint.position;
        sbpl_angle_tolerance[i] = std::min(
                fabs(joint_constraint.tolerance_above),
                fabs(joint_constraint.tolerance_below));
        ROS_INFO_NAMED(PI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, goal_constraints.joint_constraints[i].joint_name.c_str(), sbpl_angle_goal[i], sbpl_angle_tolerance[i]);
    }

    GoalConstraint goal;
    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;

    // TODO: really need to reevaluate the necessity of the planning link
    goal.pose.resize(6, 0.0);
    goal.tgt_off_pose.resize(6, 0.0);

    // set sbpl environment goal
    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    // set planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalPosition(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Setting goal position");

    Eigen::Affine3d goal_pose;
    Eigen::Vector3d offset;
    if (!extractGoalPoseFromGoalConstraints(
            goal_constraints, goal_pose, offset))
    {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    GoalConstraint goal;
    goal.type = GoalType::XYZ_RPY_GOAL;
    goal.pose.resize(6);
    goal.pose[0] = goal_pose.translation()[0];
    goal.pose[1] = goal_pose.translation()[1];
    goal.pose[2] = goal_pose.translation()[2];
    angles::get_euler_zyx(
            goal_pose.rotation(), goal.pose[5], goal.pose[4], goal.pose[3]);
    goal.xyz_offset[0] = offset.x();
    goal.xyz_offset[1] = offset.y();
    goal.xyz_offset[2] = offset.z();

    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal.xyz_tolerance[0] = sbpl_tolerance[0];
    goal.xyz_tolerance[1] = sbpl_tolerance[1];
    goal.xyz_tolerance[2] = sbpl_tolerance[2];
    goal.rpy_tolerance[0] = sbpl_tolerance[3];
    goal.rpy_tolerance[1] = sbpl_tolerance[4];
    goal.rpy_tolerance[2] = sbpl_tolerance[5];

    ROS_INFO_NAMED(PI_LOGGER, "New Goal");
    ROS_INFO_NAMED(PI_LOGGER, "    frame: %s", m_params.planning_frame.c_str());
    ROS_INFO_NAMED(PI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f)", goal.pose[0], goal.pose[1], goal.pose[2], goal.pose[3], goal.pose[4], goal.pose[5]);
    ROS_INFO_NAMED(PI_LOGGER, "    offset: (%0.3f, %0.3f, %0.3f)", goal.xyz_offset[0], goal.xyz_offset[1], goal.xyz_offset[2]);
    ROS_INFO_NAMED(PI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", sbpl_tolerance[0], sbpl_tolerance[1], sbpl_tolerance[2], sbpl_tolerance[3], sbpl_tolerance[4], sbpl_tolerance[5]);

    // ...a lot more relies on this than I had hoped
    Eigen::Affine3d target_pose = goal_pose * Eigen::Translation3d(offset);
    goal.tgt_off_pose =
    {
        target_pose.translation()[0],
        target_pose.translation()[1],
        target_pose.translation()[2],
        goal.pose[3],
        goal.pose[4],
        goal.pose[5]
    };

    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal state");
        return false;
    }

    // set sbpl planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set goal state. Exiting.");
        return false;
    }

    return true;
}

bool PlannerInterface::plan(std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario
    SV_SHOW_INFO(getBfsWallsVisualization());

    ROS_WARN_NAMED(PI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    ReplanParams replan_params(m_params.allowed_time);
    replan_params.initial_eps = m_params.epsilon;
    replan_params.final_eps = 1.0;
    replan_params.dec_eps = 0.2;
    replan_params.return_first_solution = false;
    // replan_params.max_time = m_params.allowed_time_;
    replan_params.repair_time = 1.0;
    b_ret = m_planner->replan(&solution_state_ids, replan_params, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions: %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Initial Epsilon: %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Final Epsilon: %0.3f", m_planner->get_final_epsilon());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    return b_ret;
}

bool PlannerInterface::planToPose(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // transform goal pose into reference_frame

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalPosition(goal_constraints)) {
        ROS_ERROR("Failed to set goal position.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions).", m_params.allowed_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool PlannerInterface::planToConfiguration(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    if (!setGoalConfiguration(goal_constraints)) {
        ROS_ERROR("Failed to set goal position.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions).", m_params.allowed_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
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
    const std::vector<moveit_msgs::Constraints>& constraints,
    std::string& why)
{
    if (constraints.empty()) {
        return true;
    }

    if (constraints.size() > 1) {
        why = "no planner currently supports more than one goal constraint";
        return false;
    }

    const moveit_msgs::Constraints& constraint = constraints.front();

    if (!constraint.visibility_constraints.empty()) {
        why = "no planner currently supports goal visibility constraints";
        return false;
    }

    // technically multiple goal position/orientation constraints can be
    // solved for if there is one position/orientation volume that entirely
    // bounds all other position/orientation volumes...ignoring for now

    if (constraint.position_constraints.size() > 1) {
        why = "no planner currently supports more than one position constraint";
        return false;
    }

    if (constraint.orientation_constraints.size() > 1) {
        why = "no planner currently supports more than one orientation constraint";
        return false;
    }

    const bool no_pose_constraint =
            constraint.position_constraints.empty() &&
            constraint.orientation_constraints.empty();
    const bool has_pose_constraint =
            constraint.position_constraints.size() == 1 &&
            constraint.orientation_constraints.size() == 1;
    const bool has_joint_constraints = !constraint.joint_constraints.empty();

    if (has_joint_constraints) {
        if (has_pose_constraint) {
            why = "no planner currently supports both pose and joint constraints";
            return false;
        }
    } else {
        if (no_pose_constraint) {
            // no constraints -> ok!
            return true;
        } else if (has_pose_constraint) {
            if (constraint.position_constraints.front().link_name !=
                constraint.orientation_constraints.front().link_name)
            {
                why = "pose constraint must be for a single link";
                return false;
            }
            return true;
        } else {
            // pose constraint is invalid
            why = "no planner supports only one position constraint or one orientation constraint";
            return false;
        }
    }

    // made it through the gauntlet
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

    // check if position & orientation constraints is empty
    const moveit_msgs::Constraints& goal_constraints =
            req.goal_constraints.front();

    if ((
            goal_constraints.position_constraints.empty() ||
            goal_constraints.orientation_constraints.empty()
        ) &&
        goal_constraints.joint_constraints.empty())
    {
        ROS_ERROR("Position or orientation constraint is empty.");
        ROS_ERROR("Joint constraint is empty.");
        ROS_ERROR("Expecting a 6D end effector pose constraint or 7D joint constraint. Exiting.");
        return false;
    }

    // check if there is more than one goal constraint
    if (goal_constraints.position_constraints.size() > 1 ||
        goal_constraints.orientation_constraints.size() > 1)
    {
        ROS_WARN_NAMED(PI_LOGGER, "The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
    }

    return true;
}

std::map<std::string, double> PlannerInterface::getPlannerStats()
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

visualization_msgs::MarkerArray
PlannerInterface::getCollisionModelTrajectoryVisualization(
    const moveit_msgs::RobotState& ref_state,
    const moveit_msgs::RobotTrajectory& res_traj) const
{
    visualization_msgs::MarkerArray ma, ma1;
    std::vector<RobotState> traj;

    if (res_traj.joint_trajectory.points.empty()) {
        ROS_ERROR("No trajectory found to visualize yet. Plan a path first.");
        return ma;
    }

    traj.resize(res_traj.joint_trajectory.points.size());
    double cinc = 1.0/double(res_traj.joint_trajectory.points.size());
    for (size_t i = 0; i < res_traj.joint_trajectory.points.size(); ++i) {
        traj[i].resize(res_traj.joint_trajectory.points[i].positions.size());
        for (size_t j = 0; j < res_traj.joint_trajectory.points[i].positions.size(); j++) {
            traj[i][j] = res_traj.joint_trajectory.points[i].positions[j];
        }

        ma1 = m_checker->getCollisionModelVisualization(traj[i]);

        for (size_t j = 0; j < ma1.markers.size(); ++j) {
            ma1.markers[j].color.r = 0.1;
            ma1.markers[j].color.g = cinc * double(res_traj.joint_trajectory.points.size() - (i + 1));
            ma1.markers[j].color.b = cinc * double(i);
        }
        ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
    }

    for (size_t i = 0; i < ma.markers.size(); ++i) {
        ma.markers[i].ns = "trajectory";
        ma.markers[i].id = i;
    }

    return ma;
}

visualization_msgs::MarkerArray
PlannerInterface::getGoalVisualization() const
{
    const moveit_msgs::Constraints& goal_constraints = m_req.goal_constraints.front();
    if (goal_constraints.position_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to get visualization marker for goals because no position constraints found.");
        return visualization_msgs::MarkerArray();
    }

    // compute space needed for goal poses
    int num_goal_pos_constraints = 0;
    for (int i = 0; i < goal_constraints.position_constraints.size(); ++i) {
        const moveit_msgs::PositionConstraint& pos_constraint =
                goal_constraints.position_constraints[i];
        num_goal_pos_constraints += pos_constraint.constraint_region.primitive_poses.size();
    }
    std::vector<std::vector<double>> poses(
            num_goal_pos_constraints, std::vector<double>(6, 0));

    for (size_t i = 0; i < goal_constraints.position_constraints.size(); ++i) {
        const moveit_msgs::PositionConstraint& pos_constraint =
                goal_constraints.position_constraints[i];
        const moveit_msgs::BoundingVolume constraint_region =
                pos_constraint.constraint_region;
        for (size_t j = 0; j < constraint_region.primitive_poses.size(); ++j) {
            const geometry_msgs::Pose& prim_pose =
                    constraint_region.primitive_poses[j];

            size_t idx = i * pos_constraint.constraint_region.primitive_poses.size() + j;

            poses[idx][0] = prim_pose.position.x;
            poses[idx][1] = prim_pose.position.y;
            poses[idx][2] = prim_pose.position.z;

            if (goal_constraints.orientation_constraints.size() > i) {
                leatherman::getRPY(
                        goal_constraints.orientation_constraints[i].orientation,
                        poses[idx][3], poses[idx][4], poses[idx][5]);
            }
            else
            {
                poses[idx][3] = 0;
                poses[idx][4] = 0;
                poses[idx][5] = 0;
            }
        }
    }
    return ::viz::getPosesMarkerArray(poses, goal_constraints.position_constraints[0].header.frame_id, "goals", 0);
}

visualization_msgs::MarkerArray
PlannerInterface::getBfsValuesVisualization() const
{
    if (m_heuristics.empty()) {
        return visualization_msgs::MarkerArray();
    }

    auto first = m_heuristics.begin();

    auto hbfs = std::dynamic_pointer_cast<BfsHeuristic>(first->second);
    if (hbfs) {
        return hbfs->getValuesVisualization();
    }

    auto hmfbfs = std::dynamic_pointer_cast<MultiFrameBfsHeuristic>(first->second);
    if (hmfbfs) {
        return hmfbfs->getValuesVisualization();
    }

    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray
PlannerInterface::getBfsWallsVisualization() const
{
    if (m_heuristics.empty()) {
        return visualization_msgs::MarkerArray();
    }

    auto first = m_heuristics.begin();

    auto hbfs = std::dynamic_pointer_cast<BfsHeuristic>(first->second);
    if (hbfs) {
        return hbfs->getWallsVisualization();
    }

    auto hmfbfs = std::dynamic_pointer_cast<MultiFrameBfsHeuristic>(first->second);
    if (hmfbfs) {
        return hmfbfs->getWallsVisualization();
    }

    return visualization_msgs::MarkerArray();
}

bool PlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Affine3d& goal_pose,
    Eigen::Vector3d& offset) const
{
    if (constraints.position_constraints.empty() ||
        constraints.orientation_constraints.empty())
    {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one position and one orientation constraint");
        return false;
    }

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?
    const moveit_msgs::PositionConstraint& position_constraint = constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Affine3d T_planning_eef = // T_planning_off * T_off_eef;
            Eigen::Translation3d(
                    primitive_pose.position.x,
                    primitive_pose.position.y,
                    primitive_pose.position.z) *
            Eigen::Quaterniond(
                    primitive_pose.orientation.w,
                    primitive_pose.orientation.x,
                    primitive_pose.orientation.y,
                    primitive_pose.orientation.z);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;

    tf::vectorMsgToEigen(position_constraint.target_point_offset, offset);
    return true;
}

bool PlannerInterface::extractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double* tol)
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front()
                .constraint_region.primitives.empty())
    {
        const moveit_msgs::PositionConstraint& position_constraint =
                goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive =
                position_constraint.constraint_region.primitives.front();
        const std::vector<double>& dims = constraint_primitive.dimensions;
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
    }
    else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const std::vector<moveit_msgs::OrientationConstraint>& orientation_constraints = goal_constraints.orientation_constraints;
        const moveit_msgs::OrientationConstraint& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

void PlannerInterface::clearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    res.trajectory_start.joint_state;
    res.trajectory_start.multi_dof_joint_state;
    res.trajectory_start.attached_collision_objects;
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

    const std::string default_search_name = "arastar";
    const std::string default_heuristic_name = "bfs";
    const std::string default_space_name = "manip";

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

    // initialize the planning space
    if (space_name == "manip") {
        ROS_INFO_NAMED(PI_LOGGER, "Initialize Manip Lattice");
        m_pspace = std::make_shared<ManipLattice>(
                m_robot, m_checker, &m_params, m_grid);

        // instantiate action space and load from file
        m_aspace = std::make_shared<ManipLatticeActionSpace>(m_pspace);
        ManipLatticeActionSpace* manip_actions =
                (ManipLatticeActionSpace*)m_aspace.get();

        if (!manip_actions->load(m_params.action_filename)) {
            ROS_ERROR("Failed to load actions from file '%s'", m_params.action_filename.c_str());
            return false;
        }
        manip_actions->useMultipleIkSolutions(m_params.use_multiple_ik_solutions);

        ROS_DEBUG_NAMED(PI_LOGGER, "Action Set:");
        for (auto ait = manip_actions->begin(); ait != manip_actions->end(); ++ait) {
            ROS_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_string(ait->type).c_str());
            if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_RPY) {
                ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", manip_actions->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
                ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", manip_actions->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_RPY));
            }
            else if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_XYZ) {
                ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", manip_actions->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
                ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", manip_actions->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ));
            }
            else if (ait->type == sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY) {
                ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", manip_actions->useAmp(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
                ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", manip_actions->ampThresh(sbpl::manip::MotionPrimitive::SNAP_TO_XYZ_RPY));
            }
            else if (ait->type == sbpl::manip::MotionPrimitive::LONG_DISTANCE ||
                ait->type == sbpl::manip::MotionPrimitive::SHORT_DISTANCE)
            {
                ROS_DEBUG_NAMED(PI_LOGGER, "    action: %s", to_string(ait->action).c_str());
            }
        }

        // associate action space with lattice
        if (!m_pspace->setActionSpace(m_aspace)) {
            ROS_ERROR("Failed to associate action space with planning space");
            return false;
        }
    } else if (space_name == "workspace") {
        ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
        m_pspace = std::make_shared<WorkspaceLattice>(
                m_robot, m_checker, &m_params, m_grid);
        WorkspaceLattice* workspace_lattice = (WorkspaceLattice*)m_pspace.get();
        WorkspaceLattice::Params wsp;
        wsp.R_count = 360;
        wsp.P_count = 180 + 1;
        wsp.Y_count = 360;

        RedundantManipulatorInterface* rmi =
                m_robot->getExtension<RedundantManipulatorInterface>();
        if (!rmi) {
            ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
            return false;
        }
        wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));
        if (!workspace_lattice->init(wsp)) {
            ROS_ERROR("Failed to initialize Workspace Lattice");
            return false;
        }
    } else {
        ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics.clear();
    if (heuristic_name == "mfbfs") {
        auto h = std::make_shared<MultiFrameBfsHeuristic>(m_pspace, m_grid);
        m_heuristics.insert(std::make_pair("MFBFS", h));
    } else if (heuristic_name == "bfs") {
        auto h = std::make_shared<BfsHeuristic>(m_pspace, m_grid);
        m_heuristics.insert(std::make_pair("BFS", h));
    } else if (heuristic_name == "euclid") {
        auto h = std::make_shared<EuclidDistHeuristic>(m_pspace, m_grid);
        m_heuristics.insert(std::make_pair("EUCLID", h));
    } else if (heuristic_name == "joint_distance") {
        auto h = std::make_shared<JointDistHeuristic>(m_pspace, m_grid);
        m_heuristics.insert(std::make_pair("JOINT_DIST", h));
    }else {
        ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    // add heuristics to planning space and gather contiguous vector (for use
    // with MHA*)
    m_heur_vec.clear();
    for (const auto& entry : m_heuristics) {
        RobotHeuristicPtr heuristic = entry.second;
        m_pspace->insertHeuristic(heuristic);
        m_heur_vec.push_back(heuristic.get());
    }

    // initialize the search algorithm
    if (search_name == "arastar") {
        m_planner.reset(new ARAPlanner(m_pspace.get(), true));
        m_planner->set_initialsolution_eps(m_params.epsilon);
        m_planner->set_search_mode(m_params.search_mode);
    } else if (search_name == "mhastar") {
        ROS_INFO_NAMED(PI_LOGGER, "Using %zu heuristics", m_heur_vec.size());

        MHAPlanner* mha = new MHAPlanner(
                m_pspace.get(),
                m_heur_vec[0],
                &m_heur_vec[0],
                m_heur_vec.size());

        // TODO: figure out a clean way to pass down planner-specific parameters
        // via solve or an auxiliary member function
        mha->set_initial_mha_eps(2.0);

        m_planner.reset(mha);
        m_planner->set_initialsolution_eps(m_params.epsilon);
        m_planner->set_search_mode(m_params.search_mode);
    } else if (search_name == "larastar") {
        m_planner.reset(new LazyARAPlanner(m_pspace.get(), true));
        m_planner->set_initialsolution_eps(m_params.epsilon);
        m_planner->set_search_mode(m_params.search_mode);
    } else if (search_name == "lmhastar") {
        ROS_ERROR("LMHA* unimplemented");
        return false;
    } else if (search_name == "adstar") {
        ROS_ERROR("AD* unimplemented");
        return false;
    } else if (search_name == "rstar") {
        ROS_ERROR("R* unimplemented");
        return false;
    } else {
        ROS_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    m_planner_id = planner_id;
    return true;
}

void PlannerInterface::profilePath(
    trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    const std::vector<std::string>& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint& prev_point = traj.points[i - 1];
        trajectory_msgs::JointTrajectoryPoint& curr_point = traj.points[i];

        // find the maximum distance traveled by any joint
        // find the time required for each joint to travel to the next waypoint
        // set the time from the start as the time to get to the previous point
        //     plus the time required for the slowest joint to reach the waypoint

        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            const double from_pos = prev_point.positions[jidx];
            const double to_pos = curr_point.positions[jidx];
            const double vel = m_robot->velLimit(jidx);
            double t = 0.0;
            if (m_robot->hasPosLimit(jidx)) {
                const double dist = fabs(to_pos - from_pos);
                t = dist / vel;
            }
            else {
                // use the shortest angular distance
                const double dist = fabs(angles::shortest_angle_diff(
                        from_pos, to_pos));
                t = dist / vel;
            }

            if (t > max_time) {
                max_time = t;
            }
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    trajectory_msgs::JointTrajectory itraj = traj;
    traj.points.clear();
    const trajectory_msgs::JointTrajectoryPoint* prev_point = &itraj.points.front();
    if (!itraj.points.empty()) {
        traj.points.push_back(*prev_point);
    }
    for (size_t i = 1; i < itraj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& curr_point = itraj.points[i];
        if (curr_point.time_from_start != prev_point->time_from_start) {
            traj.points.push_back(curr_point);
            prev_point = &curr_point;
        }
    }
}

bool PlannerInterface::isPathValid(
    const std::vector<RobotState>& path) const
{
    for (size_t i = 1; i < path.size(); ++i) {
        double dist;
        int plen, nchecks;
        if (!m_checker->isStateToStateValid(path[i - 1], path[i], plen, nchecks, dist)) {
            ROS_ERROR("path between %s and %s is invalid (%zu -> %zu)", to_string(path[i - 1]).c_str(), to_string(path[i]).c_str(), i - 1, i);
            return false;
        }
    }
    return true;
}

void PlannerInterface::postProcessPath(
    const std::vector<RobotState>& path,
    trajectory_msgs::JointTrajectory& traj) const
{
    const bool check_planned_path = true;
    if (check_planned_path && !isPathValid(path)) {
        ROS_ERROR("Planned path is invalid");
    }

    convertJointVariablePathToJointTrajectory(path, traj);

    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

    // shortcut path
    if (m_params.shortcut_path) {
        trajectory_msgs::JointTrajectory straj;
        if (!InterpolateTrajectory(m_checker, traj.points, straj.points)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate planned trajectory with %zu waypoints before shortcutting.", traj.points.size());
            trajectory_msgs::JointTrajectory otraj = traj;
            ShortcutTrajectory(
                    m_robot, m_checker, otraj.points, traj.points, m_params.shortcut_type);
        }
        else {
            ShortcutTrajectory(
                    m_robot, m_checker, straj.points, traj.points, m_params.shortcut_type);
        }
    }

    // interpolate path
    if (m_params.interpolate_path) {
        trajectory_msgs::JointTrajectory itraj = traj;
        if (!InterpolateTrajectory(m_checker, itraj.points, traj.points)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate trajectory");
        }
    }

    if (m_params.print_path) {
        leatherman::printJointTrajectory(traj, "path");
    }

    profilePath(traj);
}

void PlannerInterface::convertJointVariablePathToJointTrajectory(
    const std::vector<RobotState>& path,
    trajectory_msgs::JointTrajectory& traj) const
{
    traj.header.frame_id = m_params.planning_frame;
    traj.joint_names = m_robot->getPlanningJoints();
    traj.points.clear();
    traj.points.reserve(path.size());
    for (const auto& point : path) {
        trajectory_msgs::JointTrajectoryPoint traj_pt;
        traj_pt.positions = point;
        traj.points.push_back(std::move(traj_pt));
    }
}

void PlannerInterface::visualizePath(
    const moveit_msgs::RobotState& traj_start,
    const moveit_msgs::RobotTrajectory& traj) const
{
    SV_SHOW_INFO(getCollisionModelTrajectoryVisualization(traj_start, traj));
}

} // namespace manip
} // namespace sbpl
