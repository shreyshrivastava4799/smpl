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

#include <smpl/graph/workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/planning_params.h>

auto std::hash<smpl::WorkspaceLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    auto seed = (size_t)0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace smpl {

auto operator<<(std::ostream& o, const WorkspaceLatticeState& s) -> std::ostream&
{
    o << "{ coord: " << s.coord << ", state: " << s.state << " }";
    return o;
}

template <
    class InputIt,
    class Equal = std::equal_to<typename std::iterator_traits<InputIt>::value_type>>
bool all_equal(InputIt first, InputIt last, typename std::iterator_traits<InputIt>::reference val)
{
    typedef typename std::iterator_traits<InputIt>::reference reference;
    return std::all_of(
            first, last,
            [&val](reference a) { return Equal()(a, val); });
}

WorkspaceLattice::~WorkspaceLattice()
{
    for (auto i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = NULL;
    }
    m_states.clear();
}

bool WorkspaceLattice::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const WorkspaceProjectionParams& params,
    WorkspaceLatticeActionSpace* actions)
{
    if (!InitWorkspaceProjection(&m_proj, robot, params)) {
        return false;
    }

    if (!DiscreteSpace::Init(robot, checker)) {
        return false;
    }

    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector
    SMPL_DEBUG_NAMED(G_LOG, "initialize environment");

    m_actions = actions;
    return true;
}

auto WorkspaceLattice::GetActionSpace() -> WorkspaceLatticeActionSpace*
{
    return m_actions;
}

auto WorkspaceLattice::GetActionSpace() const -> const WorkspaceLatticeActionSpace*
{
    return m_actions;
}

auto WorkspaceLattice::GetVisualizationFrameId() const -> const std::string&
{
    return m_viz_frame_id;
}

void WorkspaceLattice::SetVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

int WorkspaceLattice::ReserveHashEntry()
{
    auto* state = new WorkspaceLatticeState;
    int state_id = (int)this->m_states.size();
    m_states.push_back(state);
    return state_id;
}

/// Create a state entry for a given coordinate and return its id
///
/// If an entry already exists for the coordinate, the id corresponding to that
/// entry is returned; otherwise, a new entry is created and its id returned.
int WorkspaceLattice::GetOrCreateState(const WorkspaceCoord& coord)
{
    auto state = WorkspaceLatticeState();
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit != m_state_to_id.end()) {
        return sit->second;
    }

    int new_id = (int)m_states.size();

    // create a new entry
    auto* state_entry = new WorkspaceLatticeState(state);

    // map id <-> state
    m_states.push_back(state_entry);
    m_state_to_id[state_entry] = new_id;

    return new_id;
}

/// Retrieve a state by its id.
///
/// The id is not checked for validity and the state is assumed to have already
/// been created, either by GetSuccs during a search or by designating a new
/// start or goal state.
auto WorkspaceLattice::GetState(int state_id) const -> WorkspaceLatticeState*
{
    assert(state_id >= 0 && state_id < m_states.size());
    return m_states[state_id];
}

bool WorkspaceLattice::CheckAction(
    const RobotState& state,
    const WorkspaceAction& action,
    RobotState* final_robot_state)
{
    auto wptraj = std::vector<RobotState>();
    wptraj.reserve(action.size());

    // check waypoints for ik solutions and joint limits
    for (auto widx = 0; widx < action.size(); ++widx) {
        auto& waypoint = action[widx];

        SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "        " << widx << ": " << waypoint);

        auto seed = state;
        // copy over seed angles from the intermediate state
        for (auto i = 0; i < GetNumFreeAngles(&m_proj); ++i) {
            seed[m_proj.fa_indices[i]] = waypoint[6 + i];
        }

        auto irstate = RobotState();
        if (!StateWorkspaceToRobot(&m_proj, waypoint, seed, irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "         -> failed to find ik solution");
            return false;
        }

        if (!GetRobotModel()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> violates joint limits");
            return false;
        }

        wptraj.push_back(std::move(irstate));
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!GetCollisionChecker()->isStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path to first waypoint in collision");
        return false;
    }

    for (auto widx = 1; widx < wptraj.size(); ++widx) {
        auto& prev_wp = wptraj[widx - 1];
        auto& curr_wp = wptraj[widx];
        if (!GetCollisionChecker()->isStateToStateValid(prev_wp, curr_wp)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path between waypoints in collision");
            return false;
        }
    }

    if (final_robot_state != NULL) {
        *final_robot_state = wptraj.back();
    }

    return true;
}

int WorkspaceLattice::ComputeCost(
    const WorkspaceLatticeState& src,
    const WorkspaceLatticeState& dst)
{
    return 30;
}

bool WorkspaceLattice::CheckLazyAction(
    const RobotState& state,
    const WorkspaceAction& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    // check waypoints for ik solutions and joint limits
    for (auto widx = 0; widx < action.size(); ++widx) {
        auto& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        " << widx << ": " << istate);

        RobotState irstate;
        if (!StateWorkspaceToRobot(&m_proj, istate, state, irstate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "         -> failed to find ik solution");
            return false;
        }

        wptraj.push_back(irstate);

        if (!GetRobotModel()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> violates joint limits");
            return false;
        }
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

auto WorkspaceLattice::GetStateVisualization(
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

void WorkspaceLattice::PrintState(int state_id, bool verbose, FILE* fout)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    if (!fout) {
        fout = stdout;
    }

    auto* state = GetState(state_id);

    std::stringstream ss;
    ss << *state;

    if (fout == stdout) {
        SMPL_DEBUG_NAMED(G_LOG, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        SMPL_WARN_NAMED(G_LOG, "%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

auto WorkspaceLattice::ExtractState(int state_id) -> const RobotState&
{
    return this->m_states[state_id]->state;
}

auto WorkspaceLattice::ProjectToPose(int state_id) -> Affine3
{
    auto* state = GetState(state_id);

    double p[6];
    PoseCoordToWorkspace(&m_proj, &state->coord[0], &p[0]);

    return Translation3(p[FK_PX], p[FK_PY], p[FK_PZ]) *
            AngleAxis(p[FK_QZ], Vector3::UnitZ()) *
            AngleAxis(p[FK_QY], Vector3::UnitY()) *
            AngleAxis(p[FK_QX], Vector3::UnitX());
}

int WorkspaceLattice::GetStateID(const RobotState& real_coords)
{
    if (real_coords.size() < GetRobotModel()->jointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "state contains insufficient coordinate positions");
        return -1;
    }

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state: " << real_coords);

    if (!GetRobotModel()->checkJointLimits(real_coords)) {
        SMPL_ERROR_NAMED(G_LOG, "state violates joint limits");
        return -1;
    }

    auto disc_coords = WorkspaceCoord();
    StateRobotToCoord(&m_proj, real_coords, disc_coords);
    auto state_id = GetOrCreateState(disc_coords);
    auto* state = GetState(state_id);
    state->state = real_coords;
    return state_id;
}

bool WorkspaceLattice::ExtractPath(
    const std::vector<int>& state_ids,
    std::vector<RobotState>& path)
{
    for (auto state_id : state_ids) {
        auto* state = GetState(state_id);
        path.push_back(state->state);
    }

    return true;
}

void WorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    auto* parent_entry = GetState(state_id);

//    assert(parent_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(parent_entry->state, vis_name));

    auto actions = std::vector<WorkspaceAction>();
    m_actions->Apply(*parent_entry, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        auto final_rstate = RobotState();
        if (!CheckAction(parent_entry->state, action, &final_rstate)) {
            continue;
        }

        auto& final_state = action.back();
        auto succ_coord = WorkspaceCoord();
        StateWorkspaceToCoord(&m_proj, final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        auto succ_id = GetOrCreateState(succ_coord);
        auto* succ_state = GetState(succ_id);
        succ_state->state = final_rstate;

        // put successor on successor list with the proper cost
        succs->push_back(succ_id);

        auto edge_cost = ComputeCost(*parent_entry, *succ_state);
        costs->push_back(edge_cost);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        cost: %5d", edge_cost);
    }
}

void WorkspaceLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    auto* state_entry = GetState(state_id);

//    assert(state_entry->coord.size() == m_dof_count);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  state: " << state_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(state_entry->state, vis_name));

    auto actions = std::vector<WorkspaceAction>();
    m_actions->Apply(*state_entry, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        auto final_rstate = RobotState();
        if (!CheckLazyAction(state_entry->state, action, &final_rstate)) {
            continue;
        }

        auto& final_state = action.back();
        auto succ_coord = WorkspaceCoord();
        StateWorkspaceToCoord(&m_proj, final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = GetOrCreateState(succ_coord);
        auto* succ_state = GetState(succ_id);
        succ_state->state = final_rstate;

        // put successor on successor list with the proper cost
        succs->push_back(succ_id);

        auto edge_cost = ComputeCost(*state_entry, *succ_state);
        costs->push_back(edge_cost);

        true_costs->push_back(false);

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        cost: %5d", edge_cost);
    }
}

int WorkspaceLattice::GetTrueCost(int state_id, int succ_state_id)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Evaluate cost of transition %d -> %d", state_id, succ_state_id);

    auto* parent_entry = GetState(state_id);
    auto* child_entry = GetState(succ_state_id);
//    assert(parent_entry->coord.size() == m_dof_count);
//    assert(child_entry->coord.size() == m_dof_count);

    auto actions = std::vector<WorkspaceAction>();
    m_actions->Apply(*parent_entry, actions);

    auto succ_coord = WorkspaceCoord();
    auto best_cost = std::numeric_limits<int>::max();
    for (auto aidx = 0; aidx < actions.size(); ++aidx) {
        auto& action = actions[aidx];

        auto final_rstate = RobotState();
        if (!CheckAction(parent_entry->state, action, &final_rstate)) {
            continue;
        }

        StateWorkspaceToCoord(&m_proj, action.back(), succ_coord);

        if (succ_coord != child_entry->coord) {
            continue;
        }

        auto edge_cost = /* TODO: compute cost */ 30;
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost == std::numeric_limits<int>::max()) {
        return -1;
    }

    return best_cost;
}


bool WorkspaceLattice::UpdateStart(int state_id)
{
    SMPL_DEBUG_NAMED(G_LOG, "set the start state");

    auto* state = GetState(state_id);

    if (!GetCollisionChecker()->isStateValid(state->state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_WARN_NAMED(vis_name, GetCollisionChecker()->getCollisionModelVisualization(state->state));
        SMPL_WARN("start state is in collision");
        return false;
    }

    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, GetStateVisualization(state->state, vis_name));

    SMPL_DEBUG_STREAM_NAMED(G_LOG, "  coord: " << state->coord);

    return DiscreteSpace::UpdateStart(state_id);
}

bool WorkspaceLattice::UpdateGoal(GoalConstraint* goal)
{
    return DiscreteSpace::UpdateGoal(goal);
}

auto WorkspaceLattice::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<DiscreteSpace>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<IProjectToPose>() ||
        class_code == GetClassCode<IProjectToPoint>() ||
        class_code == GetClassCode<IExtractRobotState>())
    {
        return this;
    }
    return NULL;
}

} // namespace smpl
