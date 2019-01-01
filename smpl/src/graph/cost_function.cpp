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

int UniformCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    assert(!action->motion.empty());

    auto* lattice = GetPlanningSpace();

    auto* checker = lattice->GetCollisionChecker();
    assert(checker != NULL);

    // collision check the trajectory between the source state and the first waypoint
    auto* state = lattice->GetHashEntry(state_id);
    auto& src_state = state->state;
    if (!checker->isStateToStateValid(src_state, action->motion[0])) {
        return 0;
    }

    for (auto i = 1; i < action->motion.size(); ++i) {
        auto& prev_state = action->motion[i - 1];
        auto& curr_state = action->motion[i];
        if (!checker->isStateToStateValid(prev_state, curr_state)) {
            return 0;
        }
    }

    return this->cost_per_action;
}

int L1NormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    return 0;
}

int L2NormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    return 0;
}

int LInfNormCostFunction::GetActionCost(
    int state_id,
    const ManipLatticeAction* action,
    int succ_id)
{
    return 0;
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
