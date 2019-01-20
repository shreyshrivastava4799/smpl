////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#include <smpl/search/mhastarpp.h>

// standard includes
#include <algorithm>

// module includes
#include "mhastar_base_impl.h"

namespace smpl {

static
void onSearchReinitialized(MHAStarPP* search)
{
    search->max_fval_closed_anc = search->base.start_state->od[0].f;
}

static
void onClosedAnchor(MHAStarPP* search, MHASearchState* s)
{
    if (s->od[0].f > search->max_fval_closed_anc) {
        search->max_fval_closed_anc = s->od[0].f;
    }
}

static
int priority(const MHAStarPP* search, MHASearchState* state)
{
    return state->g + (int)(search->base.w_heur * (double)state->od[0].h);
}

static
bool terminated(const MHAStarPP* search)
{
    return search->base.best_goal.g <= search->max_fval_closed_anc;
}

static
bool satisfies_p_criterion(const MHAStarPP* search, MHASearchState* state)
{
    auto f_min = search->base.open[0].min()->f;
    return state->g + state->od[0].h <= std::max(search->max_fval_closed_anc, f_min);
}

bool Init(
    MHAStarPP* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int num_heurs)
{
    return Init(&search->base, space, anchor, heurs, num_heurs);
}

auto GetInitialEps(const MHAStarPP* search) -> double
{
    return GetInitialEps(&search->base);
}

void SetInitialEps(MHAStarPP* search, double eps)
{
    return SetInitialEps(&search->base, eps);
}

auto GetTargetEps(const MHAStarPP* search) -> double
{
    return GetTargetEps(&search->base);
}

void SetTargetEps(MHAStarPP* search, double eps)
{
    return SetTargetEps(&search->base, eps);
}

auto GetDeltaEps(const MHAStarPP* search) -> double
{
    return GetDeltaEps(&search->base);
}

void SetDeltaEps(MHAStarPP* search, double eps)
{
    return SetDeltaEps(&search->base, eps);
}

int GetAnchorExpansionFreq(const MHAStarPP* search)
{
    return GetAnchorExpansionFreq(&search->base);
}

void SetAnchorExpansionFreq(MHAStarPP* search, int freq)
{
    return SetAnchorExpansionFreq(&search->base, freq);
}

bool UpdateStart(MHAStarPP* search, int state_id)
{
    return UpdateStart(&search->base, state_id);
}

bool UpdateGoal(MHAStarPP* search, GoalConstraint* goal)
{
    return UpdateGoal(&search->base, goal);
}

void ForcePlanningFromScratch(MHAStarPP* search)
{
    return ForcePlanningFromScratch(&search->base);
}

void ForcePlanningFromScratchAndFreeMemory(MHAStarPP* search)
{
    return ForcePlanningFromScratchAndFreeMemory(&search->base);
}

int Replan(
    MHAStarPP* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* solcost)
{
    return Replan(&search->base, search, timeout, solution, solcost);
}

auto GetSolutionEps(const MHAStarPP* search) -> double
{
    return GetSolutionEps(&search->base);
}

int GetNumExpansions(const MHAStarPP* search)
{
    return GetNumExpansions(&search->base);
}

int GetNumExpansionsInitialEps(const MHAStarPP* search)
{
    return GetNumExpansionsInitialEps(&search->base);
}

auto GetElapsedTime(const MHAStarPP* search) -> double
{
    return GetElapsedTime(&search->base);
}

auto GetElapsedTimeInitialEps(const MHAStarPP* search) -> double
{
    return GetElapsedTimeInitialEps(&search->base);
}

bool MHAStarPP::UpdateStart(int state_id)
{
    return ::smpl::UpdateStart(this, state_id);
}

bool MHAStarPP::UpdateGoal(GoalConstraint* goal)
{
    return ::smpl::UpdateGoal(this, goal);
}

void MHAStarPP::ForcePlanningFromScratch()
{
    return ::smpl::ForcePlanningFromScratch(this);
}

void MHAStarPP::ForcePlanningFromScratchAndFreeMemory()
{
    return ::smpl::ForcePlanningFromScratchAndFreeMemory(this);
}

int MHAStarPP::Replan(
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* solcost)
{
    return ::smpl::Replan(this, timeout, solution, solcost);
}

int MHAStarPP::GetNumExpansions()
{
    return ::smpl::GetNumExpansions(this);
}

auto MHAStarPP::GetElapsedTime() -> double
{
    return ::smpl::GetElapsedTime(this);
}


} // namespace smpl
