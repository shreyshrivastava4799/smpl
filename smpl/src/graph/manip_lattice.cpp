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
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

// system includes
#include <boost/functional/hash.hpp>

#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/action_space.h>
#include <smpl/graph/cost_function.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/spatial.h>

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
    auto succ_coord = RobotCoord(lattice->GetRobotModel()->jointVariableCount());
    lattice->StateToCoord(succ_real_coords, succ_coord);
    return lattice->GetOrCreateState(succ_coord, succ_real_coords);
}

ManipLattice::~ManipLattice()
{
    for (auto i = 0; i < m_states.size(); ++i) {
        delete m_states[i];
        m_states[i] = NULL;
    }
}

bool ManipLattice::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const std::vector<double>& resolutions,
    ActionSpace* actions,
    CostFunction* cost_fun)
{
    return Init(robot, checker, resolutions, actions, cost_fun, NULL);
}

bool ManipLattice::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const std::vector<double>& resolutions,
    ActionSpace* actions,
    CostFunction* cost_fun,
    LazyCostFunction* lazy_cost_fun)
{
    SMPL_DEBUG_NAMED(G_LOG, "Initialize Manip Lattice");

    if (robot == NULL) {
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

    if (resolutions.size() != robot->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "Insufficient variable resolutions for robot model");
        return false;
    }

    if (!DiscreteSpace::Init(robot, checker)) {
        SMPL_ERROR_NAMED(G_LOG, "Failed to initialize Robot Planning Space");
        return false;
    }

    m_action_space = actions;
    m_cost_fun = cost_fun;
    m_lazy_cost_fun = lazy_cost_fun;

    m_fk_iface = robot->GetExtension<IForwardKinematics>();

    m_min_limits.resize(robot->jointVariableCount());
    m_max_limits.resize(robot->jointVariableCount());
    m_continuous.resize(robot->jointVariableCount());
    m_bounded.resize(robot->jointVariableCount());
    for (auto jidx = 0; jidx < robot->jointVariableCount(); ++jidx) {
        m_min_limits[jidx] = robot->minPosLimit(jidx);
        m_max_limits[jidx] = robot->maxPosLimit(jidx);
        m_continuous[jidx] = robot->isContinuous(jidx);
        m_bounded[jidx] = robot->hasPosLimit(jidx);

        SMPL_DEBUG_NAMED(G_LOG, "variable %d: { min: %f, max: %f, continuous: %s, bounded: %s }",
                jidx,
                m_min_limits[jidx],
                m_max_limits[jidx],
                m_continuous[jidx] ? "true" : "false",
                m_bounded[jidx] ? "true" : "false");
    }

    auto discretization = std::vector<int>(robot->jointVariableCount());
    auto deltas = std::vector<double>(robot->jointVariableCount());
    for (auto vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
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

    return true;
}

auto ManipLattice::GetResolutions() const -> const std::vector<double>&
{
    return m_coord_deltas;
}

auto ManipLattice::GetActionSpace() -> ActionSpace*
{
    return m_action_space;
}

auto ManipLattice::GetActionSpace() const -> const ActionSpace*
{
    return m_action_space;
}

auto ManipLattice::GetCostFunction() -> CostFunction*
{
    return m_cost_fun;
}

auto ManipLattice::GetCostFunction() const -> const CostFunction*
{
    return m_cost_fun;
}

auto ManipLattice::GetVisualizationFrameId() const -> const std::string&
{
    return m_viz_frame_id;
}

void ManipLattice::SetVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

auto ManipLattice::GetDiscreteCenter(const RobotState& state) const -> RobotState
{
    auto coord = RobotCoord(GetRobotModel()->jointVariableCount());
    auto center = RobotState(GetRobotModel()->jointVariableCount());
    StateToCoord(state, coord);
    CoordToState(coord, center);
    return center;
}

auto ManipLattice::GetDiscreteState(const RobotState& state) const -> RobotCoord
{
    auto coord = RobotCoord(GetRobotModel()->jointVariableCount());
    StateToCoord(state, coord);
    return coord;
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
void ManipLattice::CoordToState(
    const RobotCoord& coord,
    RobotState& state) const
{
    assert((int)state.size() == GetRobotModel()->jointVariableCount() &&
            (int)coord.size() == GetRobotModel()->jointVariableCount());

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

void ManipLattice::StateToCoord(
    const RobotState& state,
    RobotCoord& coord) const
{
    assert((int)state.size() == GetRobotModel()->jointVariableCount() &&
            (int)coord.size() == GetRobotModel()->jointVariableCount());

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

int ManipLattice::ReserveHashEntry()
{
    auto* entry = new ManipLatticeState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

int ManipLattice::GetOrCreateState(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = GetHashEntry(coord);
    if (state_id < 0) {
        state_id = CreateHashEntry(coord, state);
    }
    return state_id;
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int ManipLattice::GetHashEntry(const RobotCoord& coord)
{
    auto state = ManipLatticeState();
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit == end(m_state_to_id)) {
        return -1;
    }
    return sit->second;
}

int ManipLattice::CreateHashEntry(
    const RobotCoord& coord,
    const RobotState& state)
{
    auto state_id = ReserveHashEntry();
    auto* entry = GetHashEntry(state_id);

    entry->coord = coord;
    entry->state = state;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

auto ManipLattice::GetHashEntry(int state_id) const -> ManipLatticeState*
{
    assert(IsValidStateID(this, state_id));
    assert(m_states[state_id] != NULL);
    return m_states[state_id];
}

void ManipLattice::ClearStates()
{
    for (auto& state : m_states) {
        delete state;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_states.shrink_to_fit();
}

bool ManipLattice::IsActionWithinBounds(
    const RobotState& state,
    const Action& action)
{
    for (auto& waypoint : action) {
        if (!GetRobotModel()->checkJointLimits(waypoint)) {
            return false;
        }
    }

    return true;
}

// Return the ID of the lowest-cost action that transitions from state_id to
// succ_id, or -1 if no such action exists.
auto ManipLattice::FindBestAction(int state_id, int succ_id) -> ManipLatticeAction
{
    auto* state = GetHashEntry(state_id);

    auto actions = m_action_space->Apply(state_id);

    // find the goal state corresponding to the cheapest valid action
    auto best_cost = std::numeric_limits<int>::max();
    auto best_action = ManipLatticeAction(); best_action.action_id = -1;
    for (auto& action : actions) {
        if (!IsActionWithinBounds(state->state, action.motion)) {
            continue;
        }

        auto succ_state_id = ApplyTransition(this, action.motion);
        if (succ_state_id < 0) {
            continue;
        }
        auto* succ_state = GetHashEntry(succ_state_id);

        if (succ_state_id != succ_id) continue;

        auto cost = m_cost_fun->GetActionCost(state_id, &action, succ_state_id);

        if (cost < 1) continue;

        if (cost < best_cost) {
            best_cost = cost;
            best_action = std::move(action);
        }
    }

    return best_action;
}

auto ManipLattice::GetStateVisualization(
    const RobotState& state,
    const std::string& ns)
    -> std::vector<visual::Marker>
{
    auto markers = GetCollisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

void ManipLattice::PrintState(int state_id, bool verbose, FILE* fout)
{
    assert(IsValidStateID(this, state_id));

    if (fout == NULL) {
        fout = stdout;
    }

    auto* entry = GetHashEntry(state_id);

    std::stringstream ss;

    ss << "{ ";
    for (size_t i = 0; i < entry->state.size(); ++i) {
        ss << std::setprecision(3) << entry->state[i];
        if (i != entry->state.size() - 1) {
            ss << ", ";
        }
    }
    ss << " }";

    if (fout == stdout) {
        SMPL_DEBUG_NAMED(G_LOG, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        SMPL_WARN("%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

auto ManipLattice::ExtractState(int state_id) -> const RobotState&
{
    return GetHashEntry(state_id)->state;
}

void ManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert((state_id >= 0) & (state_id < m_states.size()));
    assert((succs != NULL) & (costs != NULL));
    assert(m_action_space != NULL);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    auto* state = GetHashEntry(state_id);

    assert(state->coord.size() >= GetRobotModel()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (auto i = 0; i < (int)actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.motion.size());

        // check action for feasibility
        if (!IsActionWithinBounds(state->state, action.motion)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> out of bounds");
            continue;
        }

        // add the successor state to the graph
        auto succ_state_id = ApplyTransition(this, action.motion);
        if (succ_state_id < 0) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> failed to create successor state");
            continue;
        }

        auto* succ_state = GetHashEntry(succ_state_id);

        // compute the cost of the action
        auto cost = m_cost_fun->GetActionCost(state_id, &action, succ_state_id);

        if (cost < 1) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> infinite cost");
            continue;
        }

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

int ManipLattice::GetStateID(const RobotState& state)
{
    if ((int)state.size() < GetRobotModel()->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "state does not contain enough variables");
        return -1;
    }

    // check joint limits of starting configuration
    if (!GetRobotModel()->checkJointLimits(state, true)) {
        SMPL_WARN(" -> violates the joint limits");
        return -1;
    }

    auto state_coord = RobotCoord(GetRobotModel()->jointVariableCount());
    StateToCoord(state, state_coord);
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord: " << state_coord);

    return GetOrCreateState(state_coord, state);
}

bool ManipLattice::ExtractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    if (idpath.empty()) {
        return true;
    }

    assert(idpath.size() > 1);

    auto opath = std::vector<RobotState>();

    {
        // grab the first point
        assert(IsValidStateID(this, idpath[0]));
        auto* first_state = GetHashEntry(idpath[0]);
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

        auto* prev_state = GetHashEntry(prev_id);

        auto best_action = FindBestAction(prev_id, curr_id);
        if (best_action.action_id < 0) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to find action from state %d to state %d", prev_id, curr_id);
            return false;
        }

        for (auto& wp : best_action.motion) {
            opath.push_back(wp);
        }
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, GetStateVisualization(path.back(), vis_name));
    return true;
}

bool ManipLattice::UpdateHeuristics(Heuristic** heuristics, int count)
{
    return m_action_space->UpdateHeuristics(heuristics, count);
}

bool ManipLattice::UpdateStart(int state_id)
{
    SMPL_DEBUG_NAMED(G_LOG, "set the start state");

    assert(IsValidStateID(this, state_id));
    auto* state = GetHashEntry(state_id);

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state: " << state->state);

    // check if the start configuration is in collision
    if (!GetCollisionChecker()->isStateValid(state->state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_WARN_NAMED(vis_name, GetCollisionChecker()->getCollisionModelVisualization(state->state));
        SMPL_WARN(" -> in collision");
        return false;
    }

    if (!m_action_space->UpdateStart(state_id)) {
        return false;
    }

    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, GetStateVisualization(state->state, vis_name));

    return DiscreteSpace::UpdateStart(state_id);
}

bool ManipLattice::UpdateGoal(GoalConstraint* goal)
{
    if (!m_action_space->UpdateGoal(goal)) {
        return false;
    }

    return DiscreteSpace::UpdateGoal(goal);
}

auto ManipLattice::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<DiscreteSpace>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<IExtractRobotState>() ||
        class_code == GetClassCode<ISearchable>())
    {
        return this;
    }

    if (class_code == GetClassCode<ILazySearchable>()) {
        if (m_lazy_cost_fun != NULL) return this;
    }

    // Conditionally expose projections to 3D and 6D if forward kinematics
    // exists for the associated RobotModel.
    if (class_code == GetClassCode<IProjectToPoint>() ||
        class_code == GetClassCode<IProjectToPose>())
    {
        if (m_fk_iface != NULL) {
            return this;
        }
    }

    return NULL;
}

void ManipLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    assert(IsValidStateID(this, state_id));

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expand state %d", state_id);

    auto* state = GetHashEntry(state_id);

    assert(state->coord.size() >= GetRobotModel()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.motion.size());

        auto succ_state_id = ApplyTransition(this, action.motion);
        if (succ_state_id < 0) continue;
        auto* succ_state = GetHashEntry(succ_state_id);

        succs->push_back(succ_state_id);
        auto cost = m_lazy_cost_fun->GetLazyActionCost(state_id, &action, succ_state_id);
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

int ManipLattice::GetTrueCost(int state_id, int succ_id)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "evaluating cost of transition %d -> %d", state_id, succ_id);

    assert(IsValidStateID(this, state_id));
    assert(IsValidStateID(this, succ_id));

    auto* state = GetHashEntry(state_id);
    auto* succ_state = GetHashEntry(succ_id);
    assert(state->coord.size() >= GetRobotModel()->jointVariableCount());
    assert(succ_state->coord.size() >= GetRobotModel()->jointVariableCount());

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(state->state, vis_name));

    auto actions = m_action_space->Apply(state_id);

    // Find the transition to the successor state with the lowest cost
    auto best_cost = std::numeric_limits<int>::max();
    for (auto i = 0; i < (int)actions.size(); ++i) {
        auto& action = actions[i];

        if (!IsActionWithinBounds(state->state, action.motion)) {
            continue;
        }

        // apply transition function
        auto this_succ_id = ApplyTransition(this, action.motion);
        if (this_succ_id < 0) {
            continue;
        }
        auto* this_succ_state = GetHashEntry(succ_id);

        // skip actions
        if (this_succ_id != succ_id) {
            continue;
        }

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %d:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints %zu:", action.motion.size());

        auto cost = m_lazy_cost_fun->GetTrueActionCost(state_id, &action, this_succ_id);

        if (cost < best_cost) {
            best_cost = cost;
        }
    }

    if (best_cost == std::numeric_limits<int>::max()) {
        return -1;
    }

    return best_cost;
}

auto ManipLattice::ProjectToPose(int state_id) -> Affine3
{
    return m_fk_iface->computeFK(GetHashEntry(state_id)->state);
}

} // namespace smpl
