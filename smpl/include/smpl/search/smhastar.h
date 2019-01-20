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

#ifndef SMPL_SMHASTAR_H
#define SMPL_SMHASTAR_H

// standard includes
#include <vector>

// project includes
#include <smpl/heap/intrusive_heap.h>
#include <smpl/search/search.h>

namespace smpl {

class DiscreteSpace;
class GoalConstraint;
class Heuristic;

///////////////
// Interface //
///////////////

class SMHAStar;

// Initialize the search.
bool Init(
    SMHAStar* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int heur_count);

// Get the weight applied to each additional heuristic on the first search
// iteration.
auto GetInitialEps(const SMHAStar* search) -> double;

// Set the weight applied to each additional heuristic on the first search
// iteration.
void SetInitialEps(SMHAStar* search, double eps);

// Get the weight used to control expansions from additional search queues
// with respect to the anchor heuristic.
auto GetInitialMHAEps(const SMHAStar* search) -> double;

// Set the weight used to control expansions from additional search queues
// with respect to the anchor heuristic.
void SetInitialMHAEps(SMHAStar* search, double eps_mha);

// Get the heuristic weight that will terminate the search.
auto GetTargetEpsilon(const SMHAStar* search) -> double;

// Set the heuristic weight that will terminate the search.
void SetTargetEpsilon(SMHAStar* search, double eps);

// Get the amount by which the heuristic weight is decreased between search
// iterations.
auto GetDeltaEpsilon(const SMHAStar* search) -> double;

// Set the amount by which the heuristic weight is decreased between search
// iterations.
void SetDeltaEpsilon(SMHAStar* search, double eps);

// Update the search with a new start state.
bool UpdateStart(SMHAStar* search, int state_id);

// Update the goal condition with a new goal condition.
bool UpdateGoal(SMHAStar* search, GoalConstraint* goal);

// Force the search to plan from scratch.
void ForcePlanningFromScratch(SMHAStar* search);

// Force the search to plan from scratch and free all memory from previous
// search iterations.
void ForcePlanningFromScratchAndFreeMemory(SMHAStar* search);

// Find a path from the current start state to a state satisfying the goal
// condition. The search proceeds until a path is found, the timeout is reached
// or no solution exists.
int Replan(
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost);

// Return the weight applied to each additional heuristic on the last
// successful iteration. A value of 0 means no solution has been found.
auto GetSolutionEps(const SMHAStar* search) -> double;

/// Get the number of expansions performed so far.
int GetNumExpansions(SMHAStar* search);

// Get the number of expansions performed so far on the first search iteration.
int GetNumExpansionsInitialEps(const SMHAStar* search);

// Return the time elapsed so far over all search iterations.
auto GetElapsedTime(SMHAStar* search) -> double;

// Return the time elapsed so far during the first search iteration.
auto GetElapsedTimeInitialEps(const SMHAStar* search) -> double;

////////////////////
// Implementation //
////////////////////

class IGoalHeuristic;
class ISearchable;

struct SMHAState
{
    int call_number;
    int state_id;
    int g;
    SMHAState* bp;

    bool closed_in_anc;
    bool closed_in_add;

    struct HeapData : public heap_element
    {
        // TODO: rather than map back to the state, the heap could know its
        // index into od for updates, though that might make it hard to
        // overallocate an array for the heap index, h, and f

        // TODO: in any case, this offset can be much smaller
        SMHAState* me;
        int h;
        int f;
    };

    HeapData od[1]; // overallocated for additional n heuristics
};

class SMHAStar : public Search
{
public:

    struct HeapCompare
    {
        bool operator()(
            const SMHAState::HeapData& s, const SMHAState::HeapData& t) const;
    };

    ISearchable* space = NULL;

    std::vector<IGoalHeuristic*> heurs;

    /// number of additional heuristics used
    int heur_count = 0;

    double w_heur_init = 1.0;
    double w_heur_final = 1.0;
    double w_heur_delta = 0.2;

    double w_anchor_init = 1.0;

    /// current w_1
    double w_heur = 1.0;

    /// current w_2
    double w_anchor = 1.0;

    /// suboptimality bound satisfied by the last search
    double w_heur_satisfied = 0.0;

    /// current number of expansion
    int num_expansions = 0;

    /// current amount of seconds
    double elapsed = 0.0;

    int call_number = 0;

    SMHAState* start_state = NULL;

    GoalConstraint* goal = NULL;
    SMHAState best_goal;

    std::vector<SMHAState*> search_states;

    /// sequence of (heur_count + 1) open lists
    using OpenList = intrusive_heap<SMHAState::HeapData, HeapCompare>;
    std::vector<OpenList> open;

    ~SMHAStar();

    int GetNumExpansions() final;
    auto GetElapsedTime() -> double final;
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;
    void ForcePlanningFromScratch() final;
    void ForcePlanningFromScratchAndFreeMemory() final;

    int Replan(
        const TimeoutCondition& timeout,
        std::vector<int>* solution,
        int* cost) final;
};

} // namespace smpl

#endif
