////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Andrew Dornbush
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

#include <smpl/graph/cost_function.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl {

CostFunction::~CostFunction()
{
}

bool CostFunction::Init(ManipLattice* space)
{
    if (space == NULL) return false;
    m_space = space;
    return true;
}

auto CostFunction::GetPlanningSpace() -> ManipLattice*
{
    return m_space;
}

auto CostFunction::GetPlanningSpace() const -> const ManipLattice*
{
    return m_space;
}

bool CostFunction::UpdateStart(int state_id)
{
    return true;
}

bool CostFunction::UpdateGoal(GoalConstraint* goal)
{
    return true;
}

// Check whether an action is valid. First checks the path segment between the
// source state and the first waypoint on the action, and then checks each path
// segment defined by the action.
static
bool IsActionValid(
    CollisionChecker* checker,
    const RobotState& src_state,
    const ManipLatticeAction* action)
{
    // collision check the trajectory between the source state and the first waypoint
    if (!checker->IsStateToStateValid(src_state, action->motion[0])) {
        return false;
    }

    for (auto i = 1; i < action->motion.size(); ++i) {
        auto& prev_state = action->motion[i - 1];
        auto& curr_state = action->motion[i];
        if (!checker->IsStateToStateValid(prev_state, curr_state)) {
            return false;
        }
    }

    return true;
}

int UniformCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    assert(!action->motion.empty());

    auto* lattice = GetPlanningSpace();

    auto* checker = lattice->GetCollisionChecker();
    assert(checker != NULL);

    auto* src_state = lattice->GetHashEntry(state_id);

    if (!IsActionValid(checker, src_state->state, action)) {
        return 0;
    }

    return this->cost_per_action;
}

auto GetL1Norm(const RobotState& dst, const RobotState& src) -> double
{
    auto d = 0.0;
    for (auto i = 0; i < (int)src.size(); ++i) {
        auto d1 = src[i];
        auto d2 = dst[i];
        auto dd = d2 - d1; // TODO: different variable types
        d += std::fabs(dd);
    }
    return d;
}

int L1NormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    assert(!action->motion.empty());
    auto* lattice = GetPlanningSpace();
    auto* checker = lattice->GetCollisionChecker();
    assert(checker != NULL);

    auto* src_state = lattice->GetHashEntry(state_id);

    if (!IsActionValid(checker, src_state->state, action)) {
        return 0;
    }

    auto total = GetL1Norm(src_state->state, action->motion[0]);
    for (auto i = 1; i < action->motion.size(); ++i) {
        auto& prev = action->motion[i - 1];
        auto& curr = action->motion[i];
        total += GetL1Norm(curr, prev);
    }

    return (int)(FIXED_POINT_RATIO * total);
}

// TODO: some copypasta with jointspacedistheuristic
auto GetL2Norm(const RobotState& dst, const RobotState& src) -> double
{
    auto d = 0.0;
    for (auto i = 0; i < (int)src.size(); ++i) {
        auto d1 = src[i];
        auto d2 = dst[i];
        auto dd = d2 - d1; // TODO: different variable types
        d += dd * dd;
    }
    return std::sqrt(d);
}

int L2NormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    assert(!action->motion.empty());
    auto* lattice = GetPlanningSpace();
    auto* checker = lattice->GetCollisionChecker();
    assert(checker != NULL);

    auto* src_state = lattice->GetHashEntry(state_id);

    if (!IsActionValid(checker, src_state->state, action)) {
        return 0;
    }

    auto total = GetL2Norm(src_state->state, action->motion[0]);
    for (auto i = 1; i < action->motion.size(); ++i) {
        auto& prev = action->motion[i - 1];
        auto& curr = action->motion[i];
        total += GetL2Norm(curr, prev);
    }

    return (int)(FIXED_POINT_RATIO * total);
}

auto GetLInfNorm(const RobotState& dst, const RobotState& src) -> double
{
    auto d = 0.0;
    for (auto i = 0; i < (int)src.size(); ++i) {
        auto d1 = src[i];
        auto d2 = dst[i];
        auto dd = d2 - d1; // TODO: different variable types
        d = std::max(d, std::fabs(dd));
    }
    return d;
}

int LInfNormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    assert(!action->motion.empty());
    auto* lattice = GetPlanningSpace();
    auto* checker = lattice->GetCollisionChecker();
    assert(checker != NULL);

    auto* src_state = lattice->GetHashEntry(state_id);

    if (!IsActionValid(checker, src_state->state, action)) {
        return 0;
    }

    auto total = GetLInfNorm(src_state->state, action->motion[0]);
    for (auto i = 1; i < action->motion.size(); ++i) {
        auto& prev = action->motion[i - 1];
        auto& curr = action->motion[i];
        total += GetLInfNorm(curr, prev);
    }

    return (int)(FIXED_POINT_RATIO * total);
}

LazyCostFunction::~LazyCostFunction()
{
}

auto DefaultLazyCostFunction::GetLazyActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
    -> std::pair<int, bool>
{
    return std::make_pair(1000, false);
}

auto DefaultLazyCostFunction::GetTrueActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
    -> int
{
    return this->cost_fun->GetActionCost(state_id, action, succ_id);
}

} // namespace smpl
