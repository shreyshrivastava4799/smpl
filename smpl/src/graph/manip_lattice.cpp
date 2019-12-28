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
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace smpl {

ManipLattice::~ManipLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();
    m_state_to_id.clear();
}

bool ManipLattice::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const std::vector<double>& resolutions,
    ActionSpace* actions)
{    

    SMPL_DEBUG_NAMED(G_LOG, "Initialize Manip Lattice");

    if (!actions) {
        SMPL_ERROR_NAMED(G_LOG, "Action Space is null");
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
    for (int jidx = 0; jidx < _robot->jointVariableCount(); ++jidx) {
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

    std::vector<int> discretization(_robot->jointVariableCount());
    std::vector<double> deltas(_robot->jointVariableCount());
    for (size_t vidx = 0; vidx < _robot->jointVariableCount(); ++vidx) {
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

    m_actions = actions;

    // change 
    initIslands();



    return true;
}

void ManipLattice::PrintState(int stateID, bool verbose, FILE* fout)
{
    assert(stateID >= 0 && stateID < (int)m_states.size());

    if (!fout) {
        fout = stdout;
    }

    ManipLatticeState* entry = m_states[stateID];

    std::stringstream ss;

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
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!m_actions->apply(parent_entry->state, actions)) {
        SMPL_WARN("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        auto is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));

        // log successor details
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Got %d goal successors!", goal_succ_count);
    }
}


void ManipLattice::GetIslandSuccs(
    int state_id,
    std::vector<int>* island_succs,
    std::vector<int>* island_costs)
{
    // printf("Inside GetIslandSuccs\n");
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));


    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (int i = 0; i < islands.size(); ++i)
    {
 
        if( insideActivationRegion(parent_entry->state, islands[i]) )
        {
            // compute destination coords
            stateToCoord(islands[i], succ_coord);
            // check if hash entry already exists, if not then create one
            int succ_state_id = getOrCreateState(succ_coord, islands[i]);
            ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
            // add the islands to succesor list of the parent state
            island_succs->push_back(succ_state_id);
            island_costs->push_back(cost(parent_entry, succ_entry, false));
        }
    }

}

Stopwatch GetLazySuccsStopwatch("GetLazySuccs", 10);

void ManipLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    GetLazySuccsStopwatch.start();
    PROFAUTOSTOP(GetLazySuccsStopwatch);

    assert(state_id >= 0 && state_id < m_states.size());

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* state_entry = m_states[state_id];

    assert(state_entry);
    assert(state_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << state_entry->state);

    auto& source_angles = state_entry->state;
    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(source_angles, vis_name));

    std::vector<Action> actions;
    if (!m_actions->apply(source_angles, actions)) {
        SMPL_WARN("Failed to get successors");
        return;
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    int goal_succ_count = 0;
    RobotCoord succ_coord(robot()->jointVariableCount());
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        stateToCoord(action.back(), succ_coord);

        auto succ_is_goal_state = isGoal(action.back());
        if (succ_is_goal_state) {
            ++goal_succ_count;
        }

        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        if (succ_is_goal_state) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(state_entry, succ_entry, succ_is_goal_state));
        true_costs->push_back(false);

        // log successor details
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", cost(state_entry, succ_entry, succ_is_goal_state));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Got %d goal successors!", goal_succ_count);
    }
}

Stopwatch GetTrueCostStopwatch("GetTrueCost", 10);

int ManipLattice::GetTrueCost(int parentID, int childID)
{
    GetTrueCostStopwatch.start();
    PROFAUTOSTOP(GetTrueCostStopwatch);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "evaluating cost of transition %d -> %d", parentID, childID);

    assert(parentID >= 0 && parentID < (int)m_states.size());
    assert(childID >= 0 && childID < (int)m_states.size());

    ManipLatticeState* parent_entry = m_states[parentID];
    ManipLatticeState* child_entry = m_states[childID];
    assert(parent_entry && parent_entry->coord.size() >= robot()->jointVariableCount());
    assert(child_entry && child_entry->coord.size() >= robot()->jointVariableCount());

    auto& parent_angles = parent_entry->state;
    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_angles, vis_name));

    std::vector<Action> actions;
    if (!m_actions->apply(parent_angles, actions)) {
        SMPL_WARN("Failed to get actions");
        return -1;
    }

    auto goal_edge = (childID == m_goal_state_id);

    size_t num_actions = 0;

    // check actions for validity and find the valid action with the least cost
    RobotCoord succ_coord(robot()->jointVariableCount());
    int best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        auto& action = actions[aidx];

        stateToCoord(action.back(), succ_coord);

        // check whether this action leads to the child state
        if (goal_edge) {
            // skip actions which don't end up at a goal state
            if (!isGoal(action.back())) {
                continue;
            }
        } else {
            // skip actions which don't end up at the child state
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", num_actions++);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints %zu:", action.size());

        if (!checkAction(parent_angles, action)) {
            continue;
        }

        // get the unique state
        int succ_state_id = goal_edge ? getHashEntry(succ_coord) : childID;
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        assert(succ_entry);

        auto edge_cost = cost(parent_entry, succ_entry, goal_edge);
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost != std::numeric_limits<int>::max()) {
        return best_cost;
    } else {
        return -1;
    }
}

const RobotState& ManipLattice::extractState(int state_id)
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

    for (size_t i = 0; i < coord.size(); ++i) {
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

    for (size_t i = 0; i < state.size(); ++i) {
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

ManipLatticeState* ManipLattice::getHashEntry(int state_id) const
{
    if (state_id < 0 || state_id >= (int)m_states.size()) {
        return nullptr;
    }

    return m_states[state_id];
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int ManipLattice::getHashEntry(const RobotCoord& coord)
{
    ManipLatticeState state;
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
    int state_id = reserveHashEntry();
    ManipLatticeState* entry = getHashEntry(state_id);

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
    ManipLatticeState* entry = new ManipLatticeState;
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

// change
// changing to have different cost for different distance
int ManipLattice::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    bool bState2IsGoal) const
{
    auto DefaultCostMultiplier = 1000;

    double distance = 0;
    for (int i = 0; i < HashEntry1->state.size(); ++i)
        distance += std::pow(HashEntry1->state[i]-HashEntry2->state[i], 2);

    return int(std::sqrt(distance)*DefaultCostMultiplier);


}

bool ManipLattice::checkAction(const RobotState& state, const Action& action)
{
    std::uint32_t violation_mask = 0x00000000;

    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        " << iidx << ": " << istate);

        // check joint limits
        if (!robot()->checkJointLimits(istate)) {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!collisionChecker()->isStateValid(istate))
//        {
//            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> in collision);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
    if (!collisionChecker()->isStateToStateValid(state, action[0])) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> path to first waypoint in collision");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        auto& prev_istate = action[j - 1];
        auto& curr_istate = action[j];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate))
        {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        -> path between waypoints %zu and %zu in collision", j - 1, j);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
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
    return dx <= tol[0] && dy <= tol[1] && dz <= tol[2];
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
            if (fabs(state[i] - goal().angles[i]) > goal().angle_tolerances[i]) {
                return false;
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

    m_actions->updateStart(state);

    // notify observers of updated start state
    return RobotPlanningSpace::setStart(state);
}

bool ManipLattice::setGoal(const GoalConstraint& goal)
{
    auto success = false;

    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
        success = setGoalPose(goal);
        break;
    case GoalType::MULTIPLE_POSE_GOAL:
        success = setGoalPoses(goal);
        break;
    case GoalType::JOINT_STATE_GOAL:
        success = setGoalConfiguration(goal);
        break;
    case GoalType::USER_GOAL_CONSTRAINT_FN:
        success = setUserGoal(goal);
        break;
    default:
        return false;
    }

    if (success) {
        m_actions->updateGoal(goal);
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
    RobotCoord coord(robot()->jointVariableCount());
    RobotState center(robot()->jointVariableCount());
    stateToCoord(state, coord);
    coordToState(coord, center);
    return center;
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

bool ManipLattice::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    if (idpath.empty()) {
        return true;
    }

    std::vector<RobotState> opath;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        auto state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            auto* entry = getHashEntry(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            opath.push_back(entry->state);
        } else {
            auto* entry = getHashEntry(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            opath.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(opath.back(), vis_name));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    // grab the first point
    {
        auto* entry = getHashEntry(idpath[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            SMPL_DEBUG_NAMED(G_LOG, "Search for transition to goal state");

            ManipLatticeState* prev_entry = m_states[prev_id];
            auto& prev_state = prev_entry->state;

            std::vector<Action> actions;
            if (!m_actions->apply(prev_state, actions)) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get actions while extracting the path");
                return false;
            }

            // find the goal state corresponding to the cheapest valid action
            ManipLatticeState* best_goal_state = nullptr;
            RobotCoord succ_coord(robot()->jointVariableCount());
            int best_cost = std::numeric_limits<int>::max();
            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                auto& action = actions[aidx];

                // skip non-goal states
                if (!isGoal(action.back())) {
                    continue;
                }

                // check the validity of this transition
                if (!checkAction(prev_state, action)) {
                    continue;
                }

                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);

                auto edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_goal_state = succ_entry;
                }
            }

            if (!best_goal_state) {
                SMPL_ERROR_STREAM_NAMED(G_LOG, "Failed to find valid goal successor from state " << prev_entry->state << " during path extraction");
                return false;
            }

            opath.push_back(best_goal_state->state);
        } else {
            auto* entry = getHashEntry(curr_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry state %d", curr_id);
                return false;
            }

            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Extract successor state " << entry->state);
            opath.push_back(entry->state);
        }
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

Extension* ManipLattice::getExtension(size_t class_code)
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

/// \brief Return the ID of the goal state or -1 if no goal has been set.
int ManipLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

/// \brief Return the ID of the start state or -1 if no start has been set.
///
/// This returns the reserved id corresponding to all states which are goal
/// states and not the state id of any particular unique state.
int ManipLattice::getStartStateID() const
{
    return m_start_state_id;
}

/// \brief Get the (heuristic) distance from the planning frame position to the
///     start
RobotState ManipLattice::getStartConfiguration() const
{
    if (m_start_state_id >= 0) {
        return getHashEntry(m_start_state_id)->state;
    } else {
        return RobotState();
    }
}

/// Set a 6-dof goal pose for the planning link
bool ManipLattice::setGoalPose(const GoalConstraint& gc)
{
    auto* vis_name = "goal_pose";
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(gc.pose, m_viz_frame_id, vis_name));

    using namespace std::chrono;
    auto now = clock::now();
    auto now_s = duration_cast<duration<double>>(now.time_since_epoch());
    SMPL_DEBUG_NAMED(G_LOG, "time: %f", now_s.count());
    SMPL_DEBUG_NAMED(G_LOG, "A new goal has been set.");
    SMPL_DEBUG_NAMED(G_LOG, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", gc.pose.translation()[0], gc.pose.translation()[1], gc.pose.translation()[2]);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (meters): %0.3f", gc.xyz_tolerance[0]);
    double yaw, pitch, roll;
    get_euler_zyx(gc.pose.rotation(), yaw, pitch, roll);
    SMPL_DEBUG_NAMED(G_LOG, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", roll, pitch, yaw);
    SMPL_DEBUG_NAMED(G_LOG, "    tol (radians): %0.3f", gc.rpy_tolerance[0]);


    // set the (modified) goal
    return RobotPlanningSpace::setGoal(gc);
}

bool ManipLattice::setGoalPoses(const GoalConstraint& gc)
{
    // TODO: a visualization would be nice
    return RobotPlanningSpace::setGoal(gc);
}

/// \brief Set a full joint configuration goal.
bool ManipLattice::setGoalConfiguration(const GoalConstraint& goal)
{
    if (goal.angles.size() != robot()->jointVariableCount() ||
        goal.angle_tolerances.size() != robot()->jointVariableCount())
    {
        return false;
    }

    auto vis_name = "target_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(goal.angles, vis_name));

    SMPL_INFO_NAMED(G_LOG, "A new goal has been set");
    SMPL_INFO_STREAM_NAMED(G_LOG, "  config: " << goal.angles);
    SMPL_INFO_STREAM_NAMED(G_LOG, "  tolerance: " << goal.angle_tolerances);

    // notify observers of updated goal
    return RobotPlanningSpace::setGoal(goal);
}

bool ManipLattice::setUserGoal(const GoalConstraint& goal)
{
    return RobotPlanningSpace::setGoal(goal);
}

bool ManipLattice::insideActivationRegion(const RobotState& state, const RobotState& island)
{
    double distance = 0;
    for (int i = 0; i < state.size(); ++i)
        distance += std::pow(state[i]-island[i], 2);

    // printf("%lf %lf\n",std::sqrt(distance), activationRadius);
    return (std::sqrt(distance) <= activationRadius);
}
//change
void ManipLattice::initIslands()
{
    // entering random states that will be passed to main file for checking
    RobotState temp;
    temp.push_back(-0.522581);
    temp.push_back(1.111512);
    temp.push_back(-1.588889);
    temp.push_back(-0.682027);
    temp.push_back(0.209440);
    temp.push_back(-1.956422);
    temp.push_back(-0.139626);
    islands.push_back(temp);
    temp.clear();

    temp.push_back(-1.149677);
    temp.push_back(1.180981);
    temp.push_back(-0.750794);
    temp.push_back(-1.589173);
    temp.push_back(0.558505);
    temp.push_back(-0.631651);
    temp.push_back(-1.60570);
    islands.push_back(temp);
    temp.clear();

    temp.push_back(-0.452903);
    temp.push_back(1.111512);
    temp.push_back(-1.449206);
    temp.push_back(-0.682027);
    temp.push_back(-0.279253);
    temp.push_back(-1.956422);
    temp.push_back(-0.139626);
    islands.push_back(temp);
    temp.clear();

    temp.push_back(-0.452903);
    temp.push_back(1.111512);
    temp.push_back(-1.519048);
    temp.push_back(-0.612246);
    temp.push_back(-0.279253);
    temp.push_back(-1.956422);
    temp.push_back(-0.139626);
    islands.push_back(temp);
    temp.clear();

}

} // namespace smpl















