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

#ifndef SMPL_FMHASTAR_SEARCH_H
#define SMPL_FMHASTAR_SEARCH_H

// standard includes
#include <vector>

// project includes
#include <smpl/search/mhastar_base.h>
#include <smpl/search/search.h>

namespace smpl {

class DiscreteSpace;
class Heuristic;
class GoalConstraint;

class FMHAStar;

bool Init(
    FMHAStar* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int num_heurs);

auto GetInitialEps(const FMHAStar* search) -> double;
void SetInitialEps(FMHAStar* search, double eps);

auto GetTargetEps(const FMHAStar* search) -> double;
void SetTargetEps(FMHAStar* search, double eps);

auto GetDeltaEps(const FMHAStar* search) -> double;
void SetDeltaEps(FMHAStar* search, double eps);

int GetAnchorExpansionFreq(const FMHAStar* search);
void SetAnchorExpansionFreq(FMHAStar* search, int freq);

bool UpdateStart(FMHAStar* search, int state_id);
bool UpdateGoal(FMHAStar* search, GoalConstraint* goal);

void ForcePlanningFromScratch(FMHAStar* search);
void ForcePlanningFromScratchAndFreeMemory(FMHAStar* search);

int Replan(
    FMHAStar* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* solcost);

auto GetSolutionEps(const FMHAStar* search) -> double;

int GetNumExpansions(const FMHAStar* search);
int GetNumExpansionsInitialEps(const FMHAStar* search);

auto GetElapsedTime(const FMHAStar* search) -> double;
auto GetElapsedTimeInitialEps(const FMHAStar* search) -> double;

class FMHAStar : public Search
{
public:

    MHAStar base;

    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;

    void ForcePlanningFromScratch() final;
    void ForcePlanningFromScratchAndFreeMemory() final;

    int Replan(
        const TimeoutCondition& timeout,
        std::vector<int>* solution,
        int* solcost) final;

    int GetNumExpansions() final;
    auto GetElapsedTime() -> double final;
};

} // namespace smpl

#endif
