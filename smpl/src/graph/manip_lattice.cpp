////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/graph/manip_lattice.h>

// standard includes
#include <iomanip>
#include <sstream>

// system includes
#include <sbpl/planners/planner.h>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/spatial.h>
#include "../profiling.h"

auto std::hash<smpl::ManipLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    auto seed = (size_t)0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace smpl {

static
bool IsValidStateID(const ManipLattice* lattice, int state_id)
{
    return (state_id >= 0) & (state_id < (int)lattice->m_states.size());
}

static
int ApplyTransition(ManipLattice* lattice, const Action& action)
{
    assert(!action.empty());
    auto& succ_real_coords = action.back();
    auto succ_coord = RobotCoord(lattice->robot()->jointVariableCount());
    lattice->stateToCoord(succ_real_coords, succ_coord);
    return lattice->getOrCreateState(succ_coord, succ_real_coords);
}

static
bool SetGoalPose(ManipLattice* lattice, const GoalConstraint& goal)
{
    auto* vis_name = "goal_pose";
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(goal.pose, lattice->m_viz_frame_id, vis_name));

    using namespace std::chrono;
    auto now = clock::now();
    auto now_s = duration_cast<duration<double>>(now.time_since_epoch());
    SMPL_DEBUG_NAMED(G_LOG, "time: %f", now_s.count());
    SMPL_DEBUG_NAMED(G_LOG, "A new goal has been set.");
    SMPL_DEBUG_NAMED(G_LOG, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", goal.pose.translation()[0], goal.pose.translation()[1], goal.pose.translation()[2]);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (meters): %0.3f", goal.xyz_tolerance[0]);
    double yaw, pitch, roll;
    get_euler_zyx(goal.pose.rotation(), yaw, pitch, roll);
    SMPL_DEBUG_NAMED(G_LOG, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", roll, pitch, yaw);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (radians): %0.3f", goal.rpy_tolerance[0]);

    // set the (modified) goal
    return lattice->RobotPlanningSpace::setGoal(goal);
}

static
bool SetGoalPoses(ManipLattice* lattice, const GoalConstraint& goal)
{
    // TODO: a visualization would be nice
    return lattice->RobotPlanningSpace::setGoal(goal);
}

static
bool SetGoalConfiguration(ManipLattice* lattice, const GoalConstraint& goal)
{
    if (goal.angles.size() != lattice->robot()->jointVariableCount() ||
        goal.angle_tolerances.size() != lattice->robot()->jointVariableCount())
    {
        return false;
    }

    auto vis_name = "target_config";
    SV_SHOW_INFO_NAMED(vis_name, lattice->getStateVisualization(goal.angles, vis_name));

    SMPL_INFO_NAMED(G_LOG, "A new goal has been set");
    SMPL_INFO_STREAM_NAMED(G_LOG, "  config: " << goal.angles);
    SMPL_INFO_STREAM_NAMED(G_LOG, "  tolerance: " << goal.angle_tolerances);

    // notify observers of updated goal
    return lattice->RobotPlanningSpace::setGoal(goal);
}

static
bool SetUserGoal(ManipLattice* lattice, const GoalConstraint& goal)
{
    return lattice->RobotPlanningSpace::setGoal(goal);
}

ManipLattice::~ManipLattice()
{
    for (auto i = 0; i < m_states.size(); ++i) {
        delete m_states[i];
        m_states[i] = NULL;
    }
}

bool ManipLattice::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const std::vector<double>& resolutions,
    ActionSpace* actions,
    CostFunction* cost_fun)
{
    SMPL_DEBUG_NAMED(G_LOG, "Initialize Manip Lattice");

    if (_robot == NULL) {
        SMPL_ERROR_NAMED(G_LOG, "Robot Model is null");
        return false;
    }

    if (checker == NULL) {
        SMPL_ERROR_NAMED(G_LOG, "Collision Checker is null");
        return false;
    }

    if (actions == NULL) {
        SMPL_ERROR_NAMED(G_LOG, "Action Space is null");
        return false;
    }

    if (cost_fun == NULL) {
        SMPL_ERROR_NAMED(G_LOG, "Cost Function is null");
        return false;
    }

    if (resolutions.size() != _robot->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "Insufficient variable resolutions for robot model");
        return false;
    }

    if (!RobotPlanningSpace::init(_robot, checker)) {
        SMPL_ERROR_NAMED(G_LOG, "Failed to initialize Robot Planning Space");
        return false;
    }

    m_fk_iface = _robot->getExtension<ForwardKinematicsInterface>();

    m_min_limits.resize(_robot->jointVariableCount());
    m_max_limits.resize(_robot->jointVariableCount());
    m_continuous.resize(_robot->jointVariableCount());
    m_bounded.resize(_robot->jointVariableCount());
    for (auto jidx = 0; jidx < _robot->jointVariableCount(); ++jidx) {
        m_min_limits[jidx] = _robot->minPosLimit(jidx);
        m_max_limits[jidx] = _robot->maxPosLimit(jidx);
        m_continuous[jidx] = _robot->isContinuous(jidx);
        m_bounded[jidx] = _robot->hasPosLimit(jidx);

        SMPL_DEBUG_NAMED(G_LOG, "variable %d: { min: %f, max: %f, continuous: %s, bounded: %s }",
                jidx,
                m_min_limits[jidx],
                m_max_limits[jidx],
                m_continuous[jidx] ? "true" : "false",
                m_bounded[jidx] ? "true" : "false");
    }

    m_goal_state_id = reserveHashEntry();
    SMPL_DEBUG_NAMED(G_LOG, "  goal state has state ID %d", m_goal_state_id);

    auto discretization = std::vector<int>(_robot->jointVariableCount());
    auto deltas = std::vector<double>(_robot->jointVariableCount());
    for (auto vidx = 0; vidx < _robot->jointVariableCount(); ++vidx) {
        if (m_continuous[vidx]) {
            discretization[vidx] = (int)std::round((2.0 * M_PI) / resolutions[vidx]);
            deltas[vidx] = (2.0 * M_PI) / (double)discretization[vidx];
        } else if (m_bounded[vidx]) {
            auto span = std::fabs(m_max_limits[vidx] - m_min_limits[vidx]);
            discretization[vidx] = std::max(1, (int)std::round(span / resolutions[vidx]));
            deltas[vidx] = span / (double)discretization[vidx];
        } else {
            discretization[vidx] = std::numeric_limits<int>::max();
            deltas[vidx] = resolutions[vidx];
        }
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord vals: " << discretization);
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord deltas: " << deltas);

    m_coord_vals = std::move(discretization);
    m_coord_deltas = std::move(deltas);

    m_action_space = actions;
    m_cost_fun = cost_fun;

    return true;
}

void ManipLattice::PrintState(int stateID, bool verbose, FILE* fout)
{
    assert(IsValidStateID(this, stateID));

    if (fout == NULL) {
        fout = stdout;
    }

    auto* entry = m_states[stateID];

    auto ss = std::stringstream();

    if (stateID == m_goal_state_id) {
        ss << "<goal state: { ";
        switch (goal().type) {
        case GoalType::XYZ_GOAL:
        case GoalType::XYZ_RPY_GOAL:
            double y, p, r;
            get_euler_zyx(goal().pose.rotation(), y, p, r);
            ss << "pose: { " <<
                    goal().pose.translation().x() << ", " <<
                    goal().pose.translation().y() << ", " <<
                    goal().pose.translation().z() << ", " <<
                    y << ", " << p << ", " << r << " }";
            break;
        case GoalType::JOINT_STATE_GOAL:
            ss << "state: " << goal().angles;
            break;
        default:
            assert(0);
            break;
        }
        ss << " }>";
    } else {
        ss << "{ ";
        for (size_t i = 0; i < entry->state.size(); ++i) {
            ss << std::setprecision(3) << entry->state[i];
            if (i != entry->state.size() - 1) {
                ss << ", ";
            }
        }
        ss << " }";
    }

    if (fout == stdout) {
        SMPL_DEBUG_NAMED(G_LOG, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        SMPL_WARN("%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

void ManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto before = succs->size();
    GetUniqueSuccs(state_id, succs, costs);
    auto after = succs->size();
    auto goal_succ_count = 0;
    for (auto i = before; i != after; ++i) {
        auto orig_id = (*succs)[i];
        if (isGoal(m_states[orig_id]->state)) {
            SMPL_INFO("Found goal %d", orig_id);
            (*succs)[i] = m_goal_state_id;
        }
    }
    SMPL_DEBUG_COND_NAMED(G_EXPANSIONS_LOG, goal_succ_count > 0, "Got %d goal successors!", goal_succ_count);
}

void ManipLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    auto before = succs->size();
    GetUniqueLazySuccs(state_id, succs, costs, true_costs);
    auto after = succs->size();
    auto goal_succ_count = 0;
    for (auto i = before; i != after; ++i) {
        if (isGoal(m_states[(*succs)[i]]->state)) {
            (*succs)[i] = m_goal_state_id;
        }
    }
    SMPL_DEBUG_COND_NAMED(G_EXPANSIONS_LOG, goal_succ_count > 0, "Got %d goal successors!", goal_succ_count);
}

int ManipLattice::GetTrueCost(int state_id, int succ_id)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "evaluating cost of transition %d -> %d", state_id, succ_id);

    assert(IsValidStateID(this, state_id));
    assert(IsValidStateID(this, succ_id));

    auto* state = getHashEntry(state_id);
    auto* succ_state = getHashEntry(succ_id);
    assert(state->coord.size() >= robot()->jointVariableCount());
    assert(succ_state->coord.size() >= robot()->jointVariableCount());

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id, state);

    auto goal_edge = (succ_id == m_goal_state_id);

    // Find the transition to the successor state with the lowest cost
    auto best_cost = std::numeric_limits<int>::max();
    for (auto i = 0; i < (int)actions.size(); ++i) {
        auto action_id = actions[i];
        auto action = m_action_space->GetActionPath(state_id, state, action_id);

        if (!isActionWithinBounds(state->state, action)) {
            continue;
        }

        // apply transition function
        auto this_succ_id = ApplyTransition(this, action);
        if (this_succ_id < 0) {
            continue;
        }
        auto* this_succ_state = m_states[succ_id];

        // if the successor is a goal state, skip actions which don't transition
        // to a goal state
        if (goal_edge && !isGoal(this_succ_state->state)) {
            continue;
        }

        // skip actions
        if (this_succ_id != succ_id) {
            continue;
        }

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %d:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints %zu:", action.size());

        auto cost = m_lazy_cost_fun->GetTrueActionCost(
                this,
                state_id, state,
                action_id, &action,
                this_succ_id, succ_state);

        if (cost < best_cost) {
            best_cost = cost;
        }
    }

    if (best_cost == std::numeric_limits<int>::max()) {
        return -1;
    }

    return best_cost;
}

auto ManipLattice::extractState(int state_id) -> const RobotState&
{
    return m_states[state_id]->state;
}

bool ManipLattice::projectToPose(int state_id, Affine3& pose)
{
    if (state_id == getGoalStateID()) {
        pose = goal().pose;
        return true;
    }

    pose = computePlanningFrameFK(m_states[state_id]->state);
    return true;
}

void ManipLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
    SMPL_WARN("GetPreds unimplemented");
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
void ManipLattice::coordToState(
    const RobotCoord& coord,
    RobotState& state) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (auto i = 0; i < (int)coord.size(); ++i) {
        if (m_continuous[i]) {
            state[i] = coord[i] * m_coord_deltas[i];
        } else if (!m_bounded[i]) {
            state[i] = (double)coord[i] * m_coord_deltas[i];
        } else {
            state[i] = m_min_limits[i] + coord[i] * m_coord_deltas[i];
        }
    }
}

void ManipLattice::stateToCoord(
    const RobotState& state,
    RobotCoord& coord) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (auto i = 0; i < (int)state.size(); ++i) {
        if (m_continuous[i]) {
            auto pos_angle = normalize_angle_positive(state[i]);

            coord[i] = (int)((pos_angle + m_coord_deltas[i] * 0.5) / m_coord_deltas[i]);

            if (coord[i] == m_coord_vals[i]) {
                coord[i] = 0;
            }
        } else if (!m_bounded[i]) {
            if (state[i] >= 0.0) {
                coord[i] = (int)(state[i] / m_coord_deltas[i] + 0.5);
            } else {
                coord[i] = (int)(state[i] / m_coord_deltas[i] - 0.5);
            }
        } else {
            coord[i] = (int)(((state[i] - m_min_limits[i]) / m_coord_deltas[i]) + 0.5);
        }
    }
}

auto ManipLattice::getHashEntry(int state_id) const -> ManipLatticeState*
{
    assert(IsValidStateID(this, state_id));
    assert(m_states[state_id] != NULL);
    return m_states[state_id];
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int ManipLattice::getHashEntry(const RobotCoord& coord)
{
    auto state = ManipLatticeState();
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int ManipLattice::createHashEntry(
    const RobotCoord& coord,
    const RobotState& state)
{
    auto state_id = reserveHashEntry();
    auto* entry = getHashEntry(state_id);

    entry->coord = coord;
    entry->state = state;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int ManipLattice::getOrCreateState(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0) {
        state_id = createHashEntry(coord, state);
    }
    return state_id;
}

int ManipLattice::reserveHashEntry()
{
    auto* entry = new ManipLatticeState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    // map planner state -> graph state
    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

/// NOTE: const although RobotModel::computeFK used underneath may
/// not be
auto ManipLattice::computePlanningFrameFK(const RobotState& state) const
    -> Affine3
{
    assert(state.size() == robot()->jointVariableCount());
    assert(m_fk_iface);

    return m_fk_iface->computeFK(state);
}

bool ManipLattice::isActionWithinBounds(
    const RobotState& state,
    const Action& action)
{
    for (auto& waypoint : action) {
        if (!robot()->checkJointLimits(waypoint)) {
            return false;
        }
    }

    return true;
}

static
bool WithinPositionTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    auto dx = std::fabs(A.translation()[0] - B.translation()[0]);
    auto dy = std::fabs(A.translation()[1] - B.translation()[1]);
    auto dz = std::fabs(A.translation()[2] - B.translation()[2]);
    return (dx <= tol[0]) & (dy <= tol[1]) & (dz <= tol[2]);
}

static
bool WithinOrientationTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    Quaternion qg(B.rotation());
    Quaternion q(A.rotation());
    if (q.dot(qg) < 0.0) {
        qg = Quaternion(-qg.w(), -qg.x(), -qg.y(), -qg.z());
    }

    auto theta = normalize_angle(2.0 * acos(q.dot(qg)));
    return theta < tol[0];
}

static
auto WithinTolerance(
    const Affine3& A,
    const Affine3& B,
    const double xyz_tolerance[3],
    const double rpy_tolerance[3])
    -> std::pair<bool, bool>
{
    if (WithinPositionTolerance(A, B, xyz_tolerance)) {
        if (WithinOrientationTolerance(A, B, rpy_tolerance)) {
            return std::make_pair(true, true);
        } else {
            return std::make_pair(true, false);
        }
    }
    return std::make_pair(false, false);
}

bool ManipLattice::isGoal(const RobotState& state)
{
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        for (int i = 0; i < goal().angles.size(); i++) {
            if (this->m_continuous[i]) {
                auto dist = shortest_angle_dist(goal().angles[i], state[i]);
                if (dist > goal().angle_tolerances[i]) return false;
            } else {
                auto dist = std::fabs(state[i] - goal().angles[i]);
                if (dist > goal().angle_tolerances[i]) return false;
            }
        }
        return true;
    }
    case GoalType::XYZ_RPY_GOAL:
    {
        // get pose of planning link
        auto pose = computePlanningFrameFK(state);

        auto near = WithinTolerance(
                pose,
                goal().pose,
                goal().xyz_tolerance,
                goal().rpy_tolerance);
        return near.first & near.second;
    }
    case GoalType::MULTIPLE_POSE_GOAL:
    {
        auto pose = computePlanningFrameFK(state);
        for (auto& goal_pose : goal().poses) {
            auto near = WithinTolerance(
                    pose, goal_pose,
                    goal().xyz_tolerance, goal().rpy_tolerance);
            if (near.first & near.second) {
                return true;
            }
        }
        return false;
    }
    case GoalType::XYZ_GOAL:
    {
        auto pose = computePlanningFrameFK(state);
        return WithinPositionTolerance(pose, goal().pose, goal().xyz_tolerance);
    }
    case GoalType::USER_GOAL_CONSTRAINT_FN:
    {
        return goal().check_goal(goal().check_goal_user, state);
    }
    default:
    {
        SMPL_ERROR_NAMED(G_LOG, "Unknown goal type.");
        return false;
    }
    }

    return false;
}

auto ManipLattice::getStateVisualization(
    const RobotState& state,
    const std::string& ns)
    -> std::vector<visual::Marker>
{
    auto markers = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

bool ManipLattice::setStart(const RobotState& state)
{
    SMPL_DEBUG_NAMED(G_LOG, "set the start state");

    if ((int)state.size() < robot()->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "start state does not contain enough joint positions");
        return false;
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state: " << state);

    // check joint limits of starting configuration
    if (!robot()->checkJointLimits(state, true)) {
        SMPL_WARN(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    if (!collisionChecker()->isStateValid(state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_WARN_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(state));
        SMPL_WARN(" -> in collision");
        return false;
    }

    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(state, vis_name));

    // get arm position in environment
    auto start_coord = RobotCoord(robot()->jointVariableCount());
    stateToCoord(state, start_coord);
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord: " << start_coord);

    m_start_state_id = getOrCreateState(start_coord, state);

    m_action_space->updateStart(state);

    // notify observers of updated start state
    return RobotPlanningSpace::setStart(state);
}

bool ManipLattice::setGoal(const GoalConstraint& goal)
{
    auto success = false;

    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
        success = SetGoalPose(this, goal);
        break;
    case GoalType::MULTIPLE_POSE_GOAL:
        success = SetGoalPoses(this, goal);
        break;
    case GoalType::JOINT_STATE_GOAL:
        success = SetGoalConfiguration(this, goal);
        break;
    case GoalType::USER_GOAL_CONSTRAINT_FN:
        success = SetUserGoal(this, goal);
        break;
    default:
        return false;
    }

    if (success) {
        m_action_space->updateGoal(goal);
    }

    return success;
}

void ManipLattice::setVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

auto ManipLattice::visualizationFrameId() const -> const std::string&
{
    return m_viz_frame_id;
}

auto ManipLattice::getDiscreteCenter(const RobotState& state) const -> RobotState
{
    auto coord = RobotCoord(robot()->jointVariableCount());
    auto center = RobotState(robot()->jointVariableCount());
    stateToCoord(state, coord);
    coordToState(coord, center);
    return center;
}

auto ManipLattice::getDiscreteState(const RobotState& state) const -> RobotCoord
{
    auto coord = RobotCoord(robot()->jointVariableCount());
    stateToCoord(state, coord);
    return coord;
}

void ManipLattice::clearStates()
{
    for (auto& state : m_states) {
        delete state;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_states.shrink_to_fit();

    m_goal_state_id = reserveHashEntry();
}

void ManipLattice::GetUniqueSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert((state_id >= 0) & (state_id < m_states.size()));
    assert((succs != NULL) & (costs != NULL));
    assert(m_action_space != NULL);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    auto* state = getHashEntry(state_id);

    assert(state->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id, state);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (auto i = 0; i < (int)actions.size(); ++i) {
        auto action_id = actions[i];
        auto action = m_action_space->GetActionPath(state_id, state, action_id);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        // check action for feasibility
        if (!isActionWithinBounds(state->state, action)) {
            continue;
        }

        // add the successor state to the graph
        auto succ_state_id = ApplyTransition(this, action);
        if (succ_state_id < 0) {
            continue;
        }

        auto* succ_state = getHashEntry(succ_state_id);

        // compute the cost of the action
        auto cost = m_cost_fun->GetActionCost(
                this,
                state_id, state,
                action_id, &action,
                succ_state_id, succ_state);

        if (cost < 1) continue;

        succs->push_back(succ_state_id);
        costs->push_back(cost);

        // log successor details
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "      succ: %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        cost: %5d", cost);
    }
}

void ManipLattice::GetUniqueLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    assert(IsValidStateID(this, state_id));

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expand state %d", state_id);

    auto* state = getHashEntry(state_id);

    assert(state->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id, state);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (auto i = 0; i < actions.size(); ++i) {
        auto& action_id = actions[i];
        auto action = m_action_space->GetActionPath(state_id, state, action_id);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        auto succ_state_id = ApplyTransition(this, action);
        if (succ_state_id < 0) continue;
        auto* succ_state = getHashEntry(succ_state_id);

        succs->push_back(succ_state_id);
        auto cost = m_lazy_cost_fun->GetLazyActionCost(
                this,
                state_id, state,
                action_id, &action,
                succ_state_id, succ_state);
        if (cost.first < 1) continue;

        costs->push_back(cost.first);
        true_costs->push_back(cost.second);

        // log successor details
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "      succ: %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        cost: %5d", cost);
    }
}

// Return the ID of the lowest-cost action that transitions from state_id to
// succ_id, or -1 if no such action exists.
auto ManipLattice::FindBestAction(int state_id, int succ_id) -> int
{
    assert(state_id != getGoalStateID());
    auto is_goal_transition = (succ_id == getGoalStateID());

    SMPL_DEBUG_COND_NAMED(G_LOG, is_goal_transition, "Search for transition to goal state");

    auto* state = m_states[state_id];

    auto actions = m_action_space->Apply(state_id, state);

    // find the goal state corresponding to the cheapest valid action
    auto best_cost = std::numeric_limits<int>::max();
    auto best_action = -1;
    for (auto action_id : actions) {
        auto action = m_action_space->GetActionPath(state_id, state, action_id);

        if (!isActionWithinBounds(state->state, action)) {
            continue;
        }

        auto succ_state_id = ApplyTransition(this, action);
        if (succ_state_id < 0) {
            continue;
        }
        auto* succ_state = m_states[succ_state_id];

        if (is_goal_transition) {
            // skip non-goal states
            if (!isGoal(succ_state->state)) continue;
        } else {
            if (succ_state_id != succ_id) continue;
        }

        auto cost = m_cost_fun->GetActionCost(
                this,
                state_id, state,
                action_id, &action,
                succ_state_id, succ_state);
        if (cost < 1) {
            continue;
        }

        if (cost < best_cost) {
            best_cost = cost;
            best_action = action_id;
        }
    }

    return best_action;
}

bool ManipLattice::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    if (idpath.empty()) {
        return true;
    }

    assert(idpath.size() > 1);
    assert(idpath[0] != getGoalStateID());

    auto opath = std::vector<RobotState>();

    {
        // grab the first point
        assert(IsValidStateID(this, idpath[0]));
        auto* first_state = m_states[idpath[0]];
        if (first_state == NULL) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(first_state->state);
    }

    // grab the rest of the points
    for (auto i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        assert(IsValidStateID(this, prev_id) & IsValidStateID(this, curr_id));

        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        assert(prev_id != getGoalStateID());

        auto* prev_state = m_states[prev_id];

        auto best_action = FindBestAction(prev_id, curr_id);
        if (best_action < 0) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to find action from state %d to state %d", prev_id, curr_id);
            return false;
        }

        auto best_action_path = m_action_space->GetActionPath(
                prev_id, prev_state, best_action);

        for (auto& wp : best_action_path) {
            opath.push_back(wp);
        }
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

auto ManipLattice::getExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }

    if (class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<PoseProjectionExtension>())
    {
        if (m_fk_iface) {
            return this;
        }
    }

    return nullptr;
}

/// Return the ID of the goal state or -1 if no goal has been set.
int ManipLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

/// Return the ID of the start state or -1 if no start has been set.
///
/// This returns the reserved id corresponding to all states which are goal
/// states and not the state id of any particular unique state.
int ManipLattice::getStartStateID() const
{
    return m_start_state_id;
}

/// Get the (heuristic) distance from the planning frame position to the start.
auto ManipLattice::getStartConfiguration() const -> RobotState
{
    if (m_start_state_id >= 0) {
        return getHashEntry(m_start_state_id)->state;
    } else {
        return RobotState();
    }
}

} // namespace smpl
