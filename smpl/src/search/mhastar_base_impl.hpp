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

#ifndef SMPL_MHASTAR_BASE_IMPL_HPP
#define SMPL_MHASTAR_BASE_IMPL_HPP

#include "mhastar_base_impl.h"

// standard includes
#include <algorithm>

// project includes
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/search/search.h>
#include <smpl/time.h>

namespace smpl {

static const char* LOG = "search.improved_mha";
static const char* V_LOG = "search.verbose.improved_mha";
static const char* E_LOG = "search.expansions.improved_mha";
static const char* S_LOG = "search.successors.improved_mha";

////////////////////
// Implementation //
////////////////////

static constexpr auto INFINITECOST = 1000000000;

inline
auto operator<<(std::ostream &o, const MHASearchState &s) -> std::ostream&
{
    o << "{ call_number: " << s.call_number << ", " <<
            "state_id: " << s.state_id << ", " <<
            "g: " << s.g << ", " <<
            "bp: " << s.bp << ", " <<
            "closed_in_anc: " << std::boolalpha << s.closed_in_anc << ", " <<
            "closed_in_add: " << std::boolalpha << s.closed_in_add << " }";
    return o;
}

static
int GetNumHeuristics(MHAStar* search)
{
    return search->num_heurs + 1;
}

static
int ComputeHeuristic(MHAStar* search, int state_id, int hidx)
{
    return search->heurs[hidx]->GetGoalHeuristic(state_id);
}

static
void InitState(
    MHAStar* search,
    MHASearchState* state,
    int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    for (auto i = 0; i < GetNumHeuristics(search); ++i) {
        state->od[i].me = state;
    }
}

static
auto GetState(MHAStar* search, int state_id) -> MHASearchState*
{
    if (search->search_states.size() < state_id + 1) {
        search->search_states.resize(state_id + 1, NULL);
    }

    if (search->search_states[state_id] == NULL) {
        // overallocate search state for appropriate heuristic information
        auto state_size =
                sizeof(MHASearchState) +
                sizeof(MHASearchState::HeapData) * (search->num_heurs);
        auto* s = (MHASearchState*)malloc(state_size);

        // force construction to correctly initialize heap position to null
        new (s) MHASearchState;
        for (auto i = 1; i < GetNumHeuristics(search); ++i) {
            new (&s->od[i]) MHASearchState::HeapData;
        }

        InitState(search, s, state_id);

        search->search_states[state_id] = s;
        return s;
    }

    return search->search_states[state_id];
}

// Reinitialize the state for a new search problem. Maintains the state id.
// Resets the cost-to-go to infinity. Removes the state from both closed lists.
// Recomputes all heuristics for the state. Does NOT remove from the OPEN or
// PSET lists.
static
void ReinitState(MHAStar* search, MHASearchState* state, bool goal = false)
{
    if (state->call_number != search->call_number) {
        state->call_number = search->call_number;
        state->g = INFINITECOST;
        state->bp = NULL;

        state->closed_in_anc = false;
        state->closed_in_add = false;

        if (goal) {
            state->od[0].h = 0;
            state->od[0].f = INFINITECOST;
        } else {
            for (auto i = 0; i < GetNumHeuristics(search); ++i) {
                state->od[i].h = ComputeHeuristic(search, state->state_id, i);
                state->od[i].f = INFINITECOST;
            }
            SMPL_DEBUG_STREAM_NAMED(V_LOG, "Reinitialized state: " << *state);
            for (auto i = 0; i < GetNumHeuristics(search); ++i) {
                SMPL_DEBUG_NAMED(V_LOG, "  h[%d]: %d", i, state->od[i].h);
                SMPL_DEBUG_NAMED(V_LOG, "  f[%d]: %d", i, state->od[i].f);
            }
        }
    }
}

static
void ClearOpenLists(MHAStar* search)
{
    for (auto i = 0; i < GetNumHeuristics(search); ++i) {
        search->open[i].clear();
    }
}

static
bool ClosedInAnySearch(const MHASearchState* state)
{
    return state->closed_in_anc || state->closed_in_add;
}

static
auto StateFromOpenState(
    MHAStar* search,
    MHASearchState::HeapData* open_state)
    -> MHASearchState*
{
    return open_state->me;
}

static
void InsertOrUpdate(MHAStar* search, MHASearchState* state, int hidx)
{
    if (search->open[hidx].contains(&state->od[hidx])) {
        search->open[hidx].update(&state->od[hidx]);
    } else {
        search->open[hidx].push(&state->od[hidx]);
    }
}

inline int ComputeRank(MHAStar* search, MHASearchState* state, int hidx)
{
    // TODO: calibrated or not?
    return state->g + (int)(search->w_heur * (double)state->od[hidx].h);
//    return state->od[hidx].h;
}

template <class Derived>
auto SelectState(MHAStar* search, Derived* derived, int hidx) -> MHASearchState*
{
    auto* state = StateFromOpenState(search, search->open[hidx].min());
    auto* min_open = search->open[0].min();
    if (satisfies_p_criterion(derived, state)) {
        return state;
    }

    for (auto it = std::next(search->open[hidx].begin()); it != search->open[hidx].end(); ++it) {
        state = StateFromOpenState(search, *it);
        if (satisfies_p_criterion(derived, state)) {
            return state;
        }
    }

    return NULL;
}

template <class Derived>
void Expand(MHAStar* search, Derived* derived, MHASearchState* state, int hidx)
{
    SMPL_DEBUG_NAMED(E_LOG, "Expanding state %d in search %d", state->state_id, hidx);

    ++search->num_expansions;

    // remove s from OPEN and all P-SETs
    for (auto hidx = 0; hidx < GetNumHeuristics(search); ++hidx) {
        if (search->open[hidx].contains(&state->od[hidx])) {
            search->open[hidx].erase(&state->od[hidx]);
        }
    }

    search->succs.clear();
    search->costs.clear();
    search->space->GetSuccs(state->state_id, &search->succs, &search->costs);
    assert(search->succs.size() == search->costs.size());

    SMPL_DEBUG_NAMED(E_LOG, "  %zu successors", search->succs.size());

    for (auto sidx = 0; sidx < search->succs.size(); ++sidx)  {
        auto cost = search->costs[sidx];
        auto* succ_state = GetState(search, search->succs[sidx]);
        ReinitState(search, succ_state);

        auto new_g = state->g + search->costs[sidx];
        SMPL_DEBUG_NAMED(S_LOG, "Compare new cost %d vs old cost %d", new_g, succ_state->g);
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;

            if (search->goal->IsGoal(search->succs[sidx])) {
                search->best_goal = *succ_state;
            }

            if (succ_state->closed_in_anc) continue;

            succ_state->od[0].f = priority(derived, succ_state);
            SMPL_DEBUG_NAMED(S_LOG, "Update state %d in ANCHOR with priority %d", succ_state->state_id, succ_state->od[0].f);
            InsertOrUpdate(search, succ_state, 0);

            // unless it's been closed in an inadmissible search...
            if (succ_state->closed_in_add) continue;

            // insert into the P-SET for each heuristic
            for (auto hidx = 1; hidx < GetNumHeuristics(search); ++hidx) {
                succ_state->od[hidx].f = ComputeRank(search, succ_state, hidx);
                SMPL_DEBUG_NAMED(S_LOG, "Update state %d in OPEN_%d with rank %d", succ_state->state_id, hidx, succ_state->od[hidx].f);
                InsertOrUpdate(search, succ_state, hidx);
            }
        }
    }
}

static
void ExtractPath(
    MHAStar* search,
    std::vector<int>* solution_path,
    int* solcost)
{
    SMPL_DEBUG_NAMED(LOG, "Extracting path");
    solution_path->clear();
    auto cost = 0;
    for (auto* state = &search->best_goal; state != NULL; state = state->bp) {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            cost += (state->g - state->bp->g);
        }
    }

    *solcost = 0;
    std::reverse(begin(*solution_path), end(*solution_path));
}

static
bool TimeLimitReached(const MHAStar* search, const TimeoutCondition& timeout)
{
    if (!timeout.bounded) {
        return false;
    }
    switch (timeout.type) {
    case TimeoutCondition::EXPANSIONS:
        return search->num_expansions >= timeout.max_expansions_init;
    case TimeoutCondition::TIME:
        return search->elapsed >= to_seconds(timeout.max_allowed_time_init);
    case TimeoutCondition::USER:
        return timeout.timed_out_fun();
    }
    return false;
}

///////////////
// Interface //
///////////////

inline bool Init(
    MHAStar* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int num_heurs)
{
    if (space == NULL || anchor == NULL || heurs == NULL || num_heurs < 0) {
        return false;
    }

    auto* searchable = space->GetExtension<ISearchable>();
    if (searchable == NULL) {
        return false;
    }

    auto goal_heuristics = std::vector<IGoalHeuristic*>();
    goal_heuristics.reserve(num_heurs + 1);

    auto* h_anchor = anchor->GetExtension<IGoalHeuristic>();
    if (h_anchor == NULL) {
        return false;
    }
    goal_heuristics.push_back(h_anchor);

    for (auto i = 0; i < num_heurs; ++i) {
        auto* h = heurs[i]->GetExtension<IGoalHeuristic>();
        if (h == NULL) {
            return false;
        }
        goal_heuristics.push_back(h);
    }

    search->space = searchable;
    search->heurs = std::move(goal_heuristics);
    search->num_heurs = num_heurs;
    search->open.resize(num_heurs + 1);

    // to ensure reinitialization is triggered
    search->best_goal.call_number = 0;
    return true;
}

inline auto GetInitialEps(const MHAStar* search) -> double
{
    return search->w_heur_init;
}

inline void SetInitialEps(MHAStar* search, double eps)
{
    search->w_heur_init = eps;
}

inline auto GetTargetEps(const MHAStar* search) -> double
{
    return search->w_heur_target;
}

inline void SetTargetEps(MHAStar* search, double eps)
{
    search->w_heur_target = eps;
}

inline auto GetDeltaEps(const MHAStar* search) -> double
{
    return search->w_heur_delta;
}

inline void SetDeltaEps(MHAStar* search, double eps)
{
    search->w_heur_delta = eps;
}

inline bool UpdateStart(MHAStar* search, int start_state_id)
{
    SMPL_DEBUG_NAMED(LOG, "Update start to %d", start_state_id);
    search->start_state = GetState(search, start_state_id);
    if (search->start_state == NULL) {
        return false;
    }
    return true;
}

inline bool UpdateGoal(MHAStar* search, GoalConstraint* goal)
{
    search->goal = goal;
    return true;
}

inline void ForcePlanningFromScratch(MHAStar* search)
{
}

inline void ForcePlanningFromScratchAndFreeMemory(MHAStar* search)
{
}

enum TermReason {
    ANCHOR_EXHAUSTED,
    TIME_LIMIT_REACHED,
    PATH_FOUND
};

template <class Derived>
auto DoSearch(
    MHAStar* search,
    Derived* derived,
    smpl::clock::time_point start_time,
    const TimeoutCondition& timeout)
    -> TermReason
{
    for (;;) {
        if (search->open[0].empty()) {
            return ANCHOR_EXHAUSTED;
        }

        if (TimeLimitReached(search, timeout)) {
            return TIME_LIMIT_REACHED;
        }

        for (auto hidx = 1; hidx < GetNumHeuristics(search); ++hidx) {
            if (search->open[0].empty()) {
                SMPL_WARN("Open list empty during inadmissible expansions?");
                return ANCHOR_EXHAUSTED;
            }

            if (terminated(derived)) {
                return PATH_FOUND;
            }

            if (!search->open[hidx].empty()) {
                auto* s = SelectState(search, derived, hidx);
                assert(s != NULL);
                s->closed_in_add = true;
                Expand(search, derived, s, hidx);
            } else {
                SMPL_WARN("PSET empty during inadmissible expansions?");
            }
        }

        search->pass_count++;
        if (search->pass_count == search->anchor_expansion_freq) {
            if (!search->open[0].empty()) {
                if (terminated(derived)) {
                    return PATH_FOUND;
                }

                auto* s = StateFromOpenState(search, search->open[0].min());
                s->closed_in_anc = true;
                Expand(search, derived, s, 0);

                onClosedAnchor(derived, s);
            }
            search->pass_count = 0;
        }

        search->elapsed = to_seconds(smpl::clock::now() - start_time);
    }
}

template <class Derived>
int Replan(
    MHAStar* search,
    Derived* derived,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* solcost)
{
    SMPL_DEBUG_NAMED(LOG, "Call replan");

    // Reset the search. TODO: pick up from where last search left off and
    // lazily reinitialize when the start changes
    ClearOpenLists(search);

    search->w_heur = search->w_heur_init;
    search->w_heur_found = 0.0;

    // reset time limits
    search->num_expansions = 0;
    search->elapsed = 0.0;
    search->pass_count = 0;

    auto start_time = smpl::clock::now();

    ++search->call_number;

    ReinitState(search, &search->best_goal, true);
    ReinitState(search, search->start_state);
    search->start_state->g = 0;

    SMPL_DEBUG_NAMED(LOG, "Insert start state into OPEN and PSET");

    // insert start state into OPEN with g(s) + h(s) as the priority
    // insert start state into PSET and place in all RANK lists
    search->start_state->od[0].f = priority(derived, search->start_state);
    search->open[0].push(&search->start_state->od[0]);
    SMPL_DEBUG_NAMED(LOG, "Insert start state %d into OPEN list with priority %d", search->start_state->state_id, search->start_state->od[0].f);
    for (auto hidx = 1; hidx < GetNumHeuristics(search); ++hidx) {
        search->start_state->od[hidx].f =
                ComputeRank(search, search->start_state, hidx);
        search->open[hidx].push(&search->start_state->od[hidx]);
        SMPL_DEBUG_NAMED(LOG, "Inserted start state %d into search %d with rank = %d", search->start_state->state_id, hidx, search->start_state->od[hidx].f);
    }

    onSearchReinitialized(derived);

    auto end_time = smpl::clock::now();
    search->elapsed = to_seconds(end_time - start_time);

    auto term_reason = DoSearch(search, derived, start_time, timeout);

    switch (term_reason) {
    case ANCHOR_EXHAUSTED:
        SMPL_DEBUG_NAMED(LOG, "Anchor search exhausted");
        return 0;
    case TIME_LIMIT_REACHED:
        SMPL_DEBUG_NAMED(LOG, "Time limit reached");
        return 0;
    case PATH_FOUND:
        search->w_heur_found = search->w_heur;
        ExtractPath(search, solution, solcost);
        return 1;
    default:
        return 0;
    }
}

inline auto GetSolutionEps(const MHAStar* search) -> double
{
    return search->w_heur_found;
}

inline int GetNumExpansions(const MHAStar* search)
{
    return search->num_expansions;
}

inline int GetNumExpansionsInitialEps(const MHAStar* search)
{
    return search->num_expansions;
}

inline auto GetElapsedTime(const MHAStar* search) -> double
{
    return search->elapsed;
}

inline auto GetElapsedTimeInitialEps(const MHAStar* search) -> double
{
    return search->elapsed;
}

inline int GetAnchorExpansionFreq(const MHAStar* search)
{
    return search->anchor_expansion_freq;
}

inline void SetAnchorExpansionFreq(MHAStar* search, int freq)
{
    search->anchor_expansion_freq = freq;
}

inline void Clear(MHAStar* search)
{
    ClearOpenLists(search);

    // free states
    for (auto* state : search->search_states) {
        free(state);
    }

    // empty state table
    search->search_states.clear();

    search->start_state = NULL;
    search->goal = NULL;
}

} // namespace smpl

#endif
