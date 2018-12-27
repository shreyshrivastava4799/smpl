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
#include <smpl/graph/manip_lattice.h>

namespace smpl {

int UniformCostFunction::GetActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
{
    assert(!action->empty());

    auto* checker = lattice->m_checker;
    assert(checker != NULL);

    // collision check the trajectory between the source state and the first waypoint
    auto& src_state = lattice->m_states[state_id]->state;
    if (!checker->isStateToStateValid(src_state, (*action)[0])) {
        return 0;
    }

    for (auto i = 1; i < action->size(); ++i) {
        auto& prev_state = (*action)[i - 1];
        auto& curr_state = (*action)[i];
        if (!checker->isStateToStateValid(prev_state, curr_state)) {
            return 0;
        }
    }

    return this->cost_per_action;
}

int L1NormCostFunction::GetActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
{
    return 0;
}

int L2NormCostFunction::GetActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
{
    return 0;
}

int LInfNormCostFunction::GetActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
{
    return 0;
}

auto DefaultLazyCostFunction::GetLazyActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
    -> std::pair<int, bool>
{
    return std::make_pair(1000, false);
}

auto DefaultLazyCostFunction::GetTrueActionCost(
    const ManipLattice* lattice,
    int state_id, const ManipLatticeState* state,
    int action_id, const Action* action,
    int succ_id, const ManipLatticeState* succ)
    -> int
{
    return this->cost_fun->GetActionCost(
            lattice, state_id, state, action_id, action, succ_id, succ);
}

} // namespace smpl
