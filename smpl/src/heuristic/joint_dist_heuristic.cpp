////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

#include <smpl/heuristic/joint_dist_heuristic.h>

// standard includes
#include <assert.h>
#include <cmath>

// project includes
#include <smpl/planning_params.h>
#include <smpl/types.h>
#include <smpl/console/console.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/discrete_space.h>

namespace smpl {

bool JointDistHeuristic::Init(DiscreteSpace* space)
{
    auto* extract_state_iface = space->GetExtension<IExtractRobotState>();
    if (extract_state_iface == NULL) {
        return false;
    }

    if (!Heuristic::Init(space)) {
        return false;
    }

    m_ers = extract_state_iface;
    return true;
}

bool JointDistHeuristic::UpdateStart(int state_id)
{
    m_start_state = m_ers->ExtractState(state_id);
    return true;
}

bool JointDistHeuristic::UpdateGoal(GoalConstraint* goal)
{
    auto* get_state = goal->GetExtension<IGetRobotState>();
    if (get_state == NULL) {
        return false;
    }
    m_get_goal_state = get_state;
    return true;
}

auto JointDistHeuristic::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>() ||
        class_code == GetClassCode<IStartHeuristic>() ||
        class_code == GetClassCode<IPairwiseHeuristic>())
    {
        return this;
    }

    return NULL;
}

static
double ComputeJointDistance(const RobotState& s, const RobotState& t)
{
    auto dsum = 0.0;
    for (auto i = 0; i < s.size(); ++i) {
        auto dj = s[i] - t[i];
        dsum += dj * dj;
    }
    return std::sqrt(dsum);
}

int JointDistHeuristic::GetGoalHeuristic(int state_id)
{
    auto goal_state = m_get_goal_state->GetState();
    auto& state = m_ers->ExtractState(state_id);
    assert(goal_state.size() == state.size());

    auto dist = ComputeJointDistance(state, goal_state);
    auto h = ToFixedPoint(dist);
    SMPL_DEBUG_NAMED(H_LOG, "h(%d) = %d", state_id, h);
    return h;
}

int JointDistHeuristic::GetStartHeuristic(int state_id)
{
    auto& s = m_start_state;
    auto& t = m_ers->ExtractState(state_id);
    auto dist = ComputeJointDistance(s, t);
    return ToFixedPoint(dist);
}

int JointDistHeuristic::GetPairwiseHeuristic(int from_id, int to_id)
{
    auto& s = m_ers->ExtractState(from_id);
    auto& t = m_ers->ExtractState(to_id);
    auto dist = ComputeJointDistance(s, t);
    return ToFixedPoint(dist);
}

} // namespace smpl
