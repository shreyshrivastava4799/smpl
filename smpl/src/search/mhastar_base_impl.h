////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Andrew Dornbush
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
#ifndef SMPL_MHASTAR_BASE_IMPL_H
#define SMPL_MHASTAR_BASE_IMPL_H

// standard includes
#include <vector>

// project includes
#include <smpl/search/mhastar_base.h>

namespace smpl {

class DiscreteSpace;
struct MHAStar;
class Heuristic;
struct TimeoutCondition;

auto operator<<(std::ostream& o, const MHASearchState& s) -> std::ostream&;

bool Init(
    MHAStar* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int num_heurs);

auto GetInitialEps(const MHAStar* search) -> double;
void SetInitialEps(MHAStar* search, double eps);

auto GetTargetEps(const MHAStar* search) -> double;
void SetTargetEps(MHAStar* search, double eps);

auto GetDeltaEps(const MHAStar* search) -> double;
void SetDeltaEps(MHAStar* search, double eps);

int GetAnchorExpansionFreq(const MHAStar* search);
void SetAnchorExpansionFreq(MHAStar* search, int freq);

bool UpdateStart(MHAStar* search, int state_id);
bool UpdateGoal(MHAStar* search, GoalConstraint* goal);

void ForcePlanningFromScratch(MHAStar* search);
void ForcePlanningFromScratchAndFreeMemory(MHAStar* search);

template <class Derived>
int Replan(
    MHAStar* search,
    Derived* derived,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* solcost);

auto GetSolutionEps(const MHAStar* search) -> double;

int GetNumExpansions(const MHAStar* search);
int GetNumExpansionsInitialEps(const MHAStar* search);

auto GetElapsedTime(const MHAStar* search) -> double;
auto GetElapsedTimeInitialEps(const MHAStar* search) -> double;

void Clear(MHAStar* search);

} // namespace smpl

#include "mhastar_base_impl.hpp"

#endif
