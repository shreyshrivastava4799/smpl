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

#include <smpl/graph/manip_lattice_action_space.h>

// standard includes
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>

namespace smpl {

static
auto ApplyMotionPrimitive(const RobotState& state, const MotionPrimitive& mp)
    -> Action
{
    auto action = mp.action;
    for (auto& waypoint : action) {
        for (auto i = 0; i < waypoint.size(); ++i) {
            waypoint[i] = state[i] + waypoint[i];
        }
    }
    return action;
}

static
void ComputeIKAction(
    ManipulationActionSpace* actions,
    const RobotState& state,
    const Affine3& goal,
    ik_option::IkOption option,
    std::vector<Action>& o_actions)
{
    assert(actions->m_ik_iface != NULL);

    if (actions->m_use_multiple_ik_solutions) {
        auto solutions = std::vector<RobotState>();
        if (!actions->m_ik_iface->computeIK(goal, state, solutions, option)) {
            return;
        }
        for (auto& solution : solutions) {
            auto action = Action{ std::move(solution) };
            o_actions.push_back(std::move(action));
        }
    } else {
        auto ik_sol = RobotState();
        if (!actions->m_ik_iface->computeIK(goal, state, ik_sol)) {
            return;
        }

        auto action = Action{ std::move(ik_sol) };
        o_actions.push_back(std::move(action));
    }
}

// If we don't have forward kinematics or we don't have distance
// information, we want the adaptive motion primitives to always be active
// and to always use short-distance motion primitives. To do this, we return
// distances of 0.0 for both the start and the goal.
static
auto GetStartAndGoalDistances(
    ManipulationActionSpace* actions,
    const RobotState& state)
    -> std::pair<double, double>
{
    if (actions->m_fk_iface == NULL) {
        return std::make_pair(0.0, 0.0);
    }

    auto pose = actions->m_fk_iface->computeFK(state);

    auto start_dist = 0.0;
    auto goal_dist = 0.0;

    if (actions->m_start_heuristic != NULL) {
        start_dist = actions->m_start_heuristic->GetMetricStartDistance(pose.translation()[0], pose.translation()[1], pose.translation()[2]);
    }
    if (actions->m_goal_heuristic != NULL) {
        goal_dist = actions->m_goal_heuristic->GetMetricGoalDistance(pose.translation()[0], pose.translation()[1], pose.translation()[2]);
    }

    return std::make_pair(start_dist, goal_dist);
}

static
auto ComputeAllActionMotions(
    ManipulationActionSpace* actions,
    const ManipLatticeState* state)
    -> std::vector<Action>
{
    auto dists = GetStartAndGoalDistances(actions, state->state);
    auto start_dist = dists.first;
    auto goal_dist = dists.second;

    auto motions = std::vector<Action>();

    auto short_dist_active =
            !actions->m_mprim_enabled[MotionPrimitive::LONG_DISTANCE] ||
            (
                actions->m_mprim_enabled[MotionPrimitive::LONG_DISTANCE] &&
                (
                    start_dist <= actions->m_mprim_thresh[MotionPrimitive::LONG_DISTANCE] ||
                    goal_dist <= actions->m_mprim_thresh[MotionPrimitive::LONG_DISTANCE]
                )
            );
    if (short_dist_active) {
        for (auto& mprim : actions->m_short_dist_mprims) {
            auto action = ApplyMotionPrimitive(state->state, mprim);
            motions.push_back(std::move(action));
        }
    }

    auto long_dist_active =
            actions->m_mprim_enabled[MotionPrimitive::LONG_DISTANCE] &&
            start_dist > actions->m_mprim_thresh[MotionPrimitive::LONG_DISTANCE] &&
            goal_dist > actions->m_mprim_thresh[MotionPrimitive::LONG_DISTANCE];

    if (long_dist_active) {
        for (auto& mprim : actions->m_long_dist_mprims) {
            auto action = ApplyMotionPrimitive(state->state, mprim);
            motions.push_back(std::move(action));
        }
    }

    auto snap_to_goal_active =
            actions->m_get_goal_state != NULL &&
            actions->m_mprim_enabled[MotionPrimitive::SNAP_TO_GOAL_CONFIG] &&
            goal_dist <= actions->m_mprim_thresh[MotionPrimitive::SNAP_TO_GOAL_CONFIG];
    if (snap_to_goal_active) {
        auto goal_state = actions->m_get_goal_state->GetState();
        auto action = Action{ std::move(goal_state) };
        motions.push_back(std::move(action));
    }

    if ((actions->m_get_goal_pose != NULL) & (actions->m_ik_iface != NULL)) {
        auto goal_pose = actions->m_get_goal_pose->GetPose();

        if (actions->m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ_RPY] &&
            goal_dist <= actions->m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ_RPY])
        {
            ComputeIKAction(actions, state->state, goal_pose, ik_option::UNRESTRICTED, motions);
        }

        if (actions->m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ] &&
            goal_dist <= actions->m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ])
        {
            ComputeIKAction(actions, state->state, goal_pose, ik_option::RESTRICT_RPY, motions);
        }

        if (actions->m_mprim_enabled[MotionPrimitive::SNAP_TO_RPY] &&
            goal_dist <= actions->m_mprim_thresh[MotionPrimitive::SNAP_TO_RPY])
        {
            ComputeIKAction(actions, state->state, goal_pose, ik_option::RESTRICT_XYZ, motions);
        }
    }

    return motions;
}

bool ManipulationActionSpace::Init(ManipLattice* space, Heuristic* heuristic)
{
    if (!ActionSpace::Init(space)) {
        return false;
    }

    Clear();

    auto* robot = space->GetRobotModel();

    m_fk_iface = robot->GetExtension<IForwardKinematics>();
    m_ik_iface = robot->GetExtension<IInverseKinematics>();

    if (m_fk_iface == NULL) {
        SMPL_WARN("Manip Lattice Action Set recommends IForwardKinematics");
    }

    if (m_ik_iface == NULL) {
        SMPL_WARN("Manip Lattice Action Set recommends IInverseKinematics");
    }

    m_start_heuristic = heuristic->GetExtension<IMetricStartHeuristic>();
    m_goal_heuristic = heuristic->GetExtension<IMetricGoalHeuristic>();

    return true;
}

/// Load motion primitives from file, removing any pre-existing actions.
///
/// The motion primitive file describes each motion primitive as a discrete
/// offset from the source state. Motion primitives are grouped by long or short
/// distance. If the file contains any long distance motion primitives, they are
/// automatically enabled.
///
/// Action Set File Format
///
/// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
/// dv11         dv12        ... dv1m
/// ...
/// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
/// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
/// ...
/// dvi1         dvi2        ... dvim
bool ManipulationActionSpace::Load(const std::string& action_filename)
{
    Clear();

    auto* f = fopen(action_filename.c_str(), "r");
    if (!f) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    auto num_motions = 0;
    auto num_dims = 0;
    auto num_short_motions = 0;

    // read in header information
    if (fscanf(f, "Motion_Primitives(degrees): %d %d %d", &num_motions, &num_dims, &num_short_motions) < 3) {
        SMPL_ERROR("Failed to parse motion primitives file header. Expected format: 'Motion_Primitives(degrees): <num motions> <num dims> <num short motions>'");
        return false;
    }

    auto num_long_motions = num_motions - num_short_motions;

    EnableLongMotions(num_long_motions > 0);

    auto read_discrete_motion = [](
        FILE* f,
        const ManipLattice* graph,
        int num_dims,
        std::vector<double>* mprim)
        -> bool
    {
        assert(mprim->size() == num_dims);
        for (auto i = 0; i < num_dims; ++i) {
            int d;
            if (fscanf(f, "%d", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1.");
                return false;
            }
            if (feof(f)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
            (*mprim)[i] = (double)d * graph->GetResolutions()[i];
        }
        SMPL_DEBUG_STREAM("Got motion primitive " << (*mprim));
        return true;
    };

    auto* lattice = GetPlanningSpace();

    for (auto i = 0; i < num_long_motions; ++i) {
        auto mprim = std::vector<double>(num_dims);
        if (!read_discrete_motion(f, lattice, num_dims, &mprim)) {
            return false;
        }
        AddMotionPrimitive(mprim, false);
    }
    for (auto i = 0; i < num_short_motions; ++i) {
        auto mprim = std::vector<double>(num_dims);
        if (!read_discrete_motion(f, lattice, num_dims, &mprim)) {
            return false;
        }
        AddMotionPrimitive(mprim, true);
    }

    fclose(f);
    return true;
}

/// \brief Add a long or short distance motion primitive to the action set
/// \param mprim The angle delta for each joint, in radians
/// \param short_dist true = short distance; false = long distance
/// \param add_converse Whether to add the negative of this motion primitive
///     to the action set
void ManipulationActionSpace::AddMotionPrimitive(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    auto m = MotionPrimitive();

    std::vector<MotionPrimitive>* mprims;
    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
        mprims = &m_short_dist_mprims;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
        mprims = &m_long_dist_mprims;
    }

    m.action.push_back(mprim);

    mprims->push_back(m);

    if (add_converse) {
        for (auto& waypoint : m.action) {
            for (auto& delta : waypoint) {
                delta *= -1.0;
            }
        }
        mprims->push_back(m);
    }
}

int ManipulationActionSpace::GetNumLongMotions() const
{
    return (int)m_long_dist_mprims.size();
}

int ManipulationActionSpace::GetNumShortMotions() const
{
    return (int)m_short_dist_mprims.size();
}

bool ManipulationActionSpace::AreLongMotionsEnabled() const
{
    return m_mprim_enabled[MotionPrimitive::LONG_DISTANCE];
}

bool ManipulationActionSpace::IsInterpMotionEnabled() const
{
    return m_mprim_enabled[MotionPrimitive::SNAP_TO_GOAL_CONFIG];
}

bool ManipulationActionSpace::IsIKMotionXYZRPYEnabled() const
{
    return m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ_RPY];
}

bool ManipulationActionSpace::IsIKMotionXYZEnabled() const
{
    return m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ];
}

bool ManipulationActionSpace::IsIKMotionRPYEnabled() const
{
    return m_mprim_enabled[MotionPrimitive::SNAP_TO_RPY];
}

bool ManipulationActionSpace::IsMultipleIKSolutionsEnabled() const
{
    return m_use_multiple_ik_solutions;
}

auto ManipulationActionSpace::GetLongMotionThreshold() const -> double
{
    return m_mprim_thresh[MotionPrimitive::LONG_DISTANCE];
}

auto ManipulationActionSpace::GetInterpMotionThreshold() const -> double
{
    return m_mprim_thresh[MotionPrimitive::SNAP_TO_GOAL_CONFIG];
}

auto ManipulationActionSpace::GetIKMotionXYZRPYThreshold() const -> double
{
    return m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ_RPY];
}

auto ManipulationActionSpace::GetIKMotionXYZThreshold() const -> double
{
    return m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ];
}

auto ManipulationActionSpace::GetIKMotionRPYThreshold() const -> double
{
    return m_mprim_thresh[MotionPrimitive::SNAP_TO_RPY];
}

void ManipulationActionSpace::EnableLongMotions(bool enable)
{
    m_mprim_enabled[MotionPrimitive::LONG_DISTANCE] = enable;
}

void ManipulationActionSpace::EnableInterpMotion(bool enable)
{
    m_mprim_enabled[MotionPrimitive::SNAP_TO_GOAL_CONFIG] = enable;
}

void ManipulationActionSpace::EnableIKMotionXYZRPY(bool enable)
{
    m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ_RPY] = enable;
}

void ManipulationActionSpace::EnableIKMotionXYZ(bool enable)
{
    m_mprim_enabled[MotionPrimitive::SNAP_TO_XYZ] = enable;
}

void ManipulationActionSpace::EnableIKMotionRPY(bool enable)
{
    m_mprim_enabled[MotionPrimitive::SNAP_TO_RPY] = enable;
}

void ManipulationActionSpace::EnableMultipleIKSolutions(bool enable)
{
    m_use_multiple_ik_solutions = enable;
}

void ManipulationActionSpace::SetLongMotionThreshold(double thresh)
{
    m_mprim_thresh[MotionPrimitive::LONG_DISTANCE] = thresh;
}

void ManipulationActionSpace::SetInterpMotionThreshold(double thresh)
{
    m_mprim_thresh[MotionPrimitive::SNAP_TO_GOAL_CONFIG] = thresh;
}

void ManipulationActionSpace::SetIKMotionXYZRPYThreshold(double thresh)
{
    m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ_RPY] = thresh;
}

void ManipulationActionSpace::SetIKMotionXYZThreshold(double thresh)
{
    m_mprim_thresh[MotionPrimitive::SNAP_TO_XYZ] = thresh;
}

void ManipulationActionSpace::SetIKMotionRPYThreshold(double thresh)
{
    m_mprim_thresh[MotionPrimitive::SNAP_TO_RPY] = thresh;
}

/// Remove long and short motion primitives and disable adaptive motions.
///
/// Thresholds for short distance and adaptive motions are retained
void ManipulationActionSpace::Clear()
{
    m_short_dist_mprims.clear();
    m_long_dist_mprims.clear();

    std::fill(m_mprim_enabled, m_mprim_enabled + sizeof(m_mprim_enabled), false);
    m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] = true;
}

bool ManipulationActionSpace::UpdateStart(int state_id)
{
    return true;
}

bool ManipulationActionSpace::UpdateGoal(GoalConstraint* goal)
{
    m_get_goal_pose = goal->GetExtension<IGetPose>();
    m_get_goal_state = goal->GetExtension<IGetRobotState>();
    return true;
}

auto ManipulationActionSpace::Apply(int state_id, ActionArray store) -> ActionArray
{
    auto* state = GetPlanningSpace()->GetHashEntry(state_id);
    auto motions = ComputeAllActionMotions(this, state);

    store.resize(motions.size());
    for (auto i = 0; i < motions.size(); ++i) {
        store[i].action_id = i;
        store[i].motion = std::move(motions[i]);
    }

    return store;
}

} // namespace smpl
