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

#ifndef SMPL_MHASTAR_BASE_H
#define SMPL_MHASTAR_BASE_H

// standard includes
#include <ostream>
#include <iomanip>
#include <vector>

// project includes
#include <smpl/heap/intrusive_heap.h>

namespace smpl {

class DiscreteSpace;
class Heuristic;
class GoalConstraint;
class ISearchable;
class IGoalHeuristic;

struct MHASearchState
{
    struct HeapData : public heap_element
    {
        MHASearchState* me;
        int h;
        int f;
    };

    MHASearchState* bp;
    int call_number;
    int state_id;
    int g;
    bool closed_in_anc;
    bool closed_in_add;
    HeapData od[1];
};

// This struct is meant to act as a base implementation for the improved MHA*
// variants. The interface to this struct is available to the implementations of
// the improved MHA* implementations.
struct MHAStar
{
    struct HeapCompare
    {
        bool operator()(
            const MHASearchState::HeapData& s,
            const MHASearchState::HeapData& t) const
        {
            return s.f < t.f;
        }
    };

    using rank_pq = intrusive_heap<MHASearchState::HeapData, HeapCompare>;

    ISearchable* space = NULL;

    // Related objects
    std::vector<IGoalHeuristic*> heurs;
    int num_heurs = 0;           // number of additional heuristics used

    double w_heur_init = 1.0;
    double w_heur_target = 1.0;
    double w_heur_delta = 1.0;

    double w_heur = 1.0;           // current w_1

    /// suboptimality bound satisfied by the last search
    double w_heur_found = 0.0;

    int num_expansions = 0;   // current number of expansion
    double elapsed = 0.0;       // current amount of seconds

    int call_number = 0;

    MHASearchState* start_state = NULL;
    MHASearchState best_goal;
    GoalConstraint* goal = NULL;

    std::vector<MHASearchState*> search_states;

    // open[0] contain the actual OPEN list sorted by g(s) + h(s)
    // open[i], i > 0, maintains a copy of the PSET for each additional
    // heuristic, sorted by rank(s, i). The PSET maintains, at all times, those
    // states which are in the OPEN list, have not been closed inadmissably,
    // and satisfy the P-CRITERION
    std::vector<rank_pq> open;

    std::vector<int> succs;
    std::vector<int> costs;

    // how many passes to make through additional search queues before expanding
    // from the anchor search
    int anchor_expansion_freq = 1;
    int pass_count = 0;

    ~MHAStar();
};

} // namespace smpl

#endif
