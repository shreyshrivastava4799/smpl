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

#include <smpl/search/smhastar.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <algorithm>
#include <utility>

// project includes
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/time.h>

namespace smpl {

////////////////////
// Implementation //
////////////////////

static const char* LOG = "search.smhastar";
static const char* ELOG = "search.smhastar.expansions";
static const char* SLOG = "search.smhastar.successors";

static constexpr auto INFINITECOST = 1000000000;

static
auto GetTime() -> double
{
    return to_seconds(clock::now().time_since_epoch());
}

static
int NumHeuristics(const SMHAStar* search)
{
    return search->heur_count;
}

static
int ComputeHeuristic(SMHAStar* search, int state_id, int hidx)
{
    return search->heurs[hidx]->GetGoalHeuristic(state_id);
}

static
void InitState(
    SMHAStar* search,
    SMHAState* state,
    size_t mha_state_idx,
    int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    state->closed_in_anc = false;
    state->closed_in_add = false;
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        state->od[i].me = state;
        state->od[i].h = ComputeHeuristic(search, state->state_id, i);
    }
}

static
auto GetState(SMHAStar* search, int state_id) -> SMHAState*
{
    if (search->search_states.size() <= state_id) {
        search->search_states.resize(state_id + 1, NULL);
    }

    auto& state = search->search_states[state_id];
    if (state == NULL) {
        // overallocate search state for appropriate heuristic information
        auto state_size =
                sizeof(SMHAState) +
                sizeof(SMHAState::HeapData) * (search->heur_count);
        auto* s = (SMHAState*)malloc(state_size);

        new (s) SMHAState;
        for (auto i = 0; i < search->heur_count; ++i) {
            new (&s->od[1 + i]) SMHAState::HeapData;
        }

        auto mha_state_idx = (int)search->search_states.size();
        InitState(search, s, mha_state_idx, state_id);

        // map graph state to search state
        state = s;
    }

    return state;
}

static
void ReinitState(SMHAStar* search, SMHAState* state, bool goal = false)
{
    if (state->call_number != search->call_number) {
        state->call_number = search->call_number;
        state->g = INFINITECOST;
        state->bp = NULL;

        state->closed_in_anc = false;
        state->closed_in_add = false;

        for (auto i = 0; i < NumHeuristics(search); ++i) {
            if (goal) {
                state->od[i].h = 0;
            } else {
                state->od[i].h = ComputeHeuristic(search, state->state_id, i);
            }
            state->od[i].f = INFINITECOST;
        }
    }
}

static
auto StateFromOpenState(SMHAState::HeapData* open_state) -> SMHAState*
{
    return open_state->me;
}

static
bool ClosedInAncSearch(const SMHAState* state)
{
    return state->closed_in_anc;
}

static
bool ClosedInAddSearch(const SMHAState* state)
{
    return state->closed_in_add;
}

static
bool ClosedInAnySearch(const SMHAState* state)
{
    return state->closed_in_anc || state->closed_in_add;
}

static
bool TimeLimitReached(const SMHAStar* search, const TimeoutCondition& timeout)
{
    if (!timeout.bounded) {
        return false;
    }
    switch (timeout.type) {
    case TimeoutCondition::EXPANSIONS:
        return search->num_expansions < timeout.max_expansions;
    case TimeoutCondition::TIME:
        return search->elapsed < to_seconds(timeout.max_allowed_time);
    case TimeoutCondition::USER:
        return timeout.timed_out_fun();
    }
    return false;
}

static
int ComputeKey(const SMHAStar* search, SMHAState* state, int hidx)
{
    return (int)((double)state->g + search->w_heur * (double)state->od[hidx].h);
}

static
int GetMinF(const SMHAStar::OpenList& pq)
{
    return pq.min()->f;
}

static
void InsertOrUpdate(SMHAStar* search, SMHAState* state, int hidx)
{
    if (search->open[hidx].contains(&state->od[hidx])) {
        search->open[hidx].update(&state->od[hidx]);
    } else {
        search->open[hidx].push(&state->od[hidx]);
    }
}

static
void Expand(SMHAStar* search, SMHAState* state, int hidx)
{
    SMPL_DEBUG_NAMED(LOG, "Expanding state %d in search %d", state->state_id, hidx);

    assert(!ClosedInAddSearch(state) || !ClosedInAncSearch(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    } else {
        state->closed_in_add = true;
    }
    ++search->num_expansions;

    // remove s from all open lists
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        if (search->open[i].contains(&state->od[i])) {
            search->open[i].erase(&state->od[i]);
        }
    }

    auto succ_ids = std::vector<int>();
    auto costs = std::vector<int>();
    search->space->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (auto sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        auto cost = costs[sidx];
        auto* succ_state = GetState(search, succ_ids[sidx]);
        ReinitState(search, succ_state);

        SMPL_DEBUG_NAMED(LOG, " Successor %d", succ_state->state_id);

        auto new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!ClosedInAncSearch(succ_state)) {
                auto fanchor = ComputeKey(search, succ_state, 0);
                succ_state->od[0].f = fanchor;
                InsertOrUpdate(search, succ_state, 0);
                SMPL_DEBUG_NAMED(LOG, "  Update in search %d with f = %d", 0, fanchor);

                if (!ClosedInAddSearch(succ_state)) {
                    for (auto i = 1; i < NumHeuristics(search); ++i) {
                        auto fn = ComputeKey(search, succ_state, i);
                        if (fn <= search->w_anchor * fanchor) {
                            succ_state->od[i].f = fn;
                            InsertOrUpdate(search, succ_state, i);
                            SMPL_DEBUG_NAMED(LOG, "  Update in search %d with f = %d", i, fn);
                        } else {
                            SMPL_DEBUG_NAMED(LOG, "  Skipping update of in search %d (%0.3f > %0.3f)", i, (double)fn, search->w_anchor * fanchor);
                        }
                    }
                }
            }
            if (search->goal->IsGoal(succ_ids[sidx])) {
                // NOTE: This assignment will not assign the N additional
                // heuristic values. It is fine to ignore those here, since
                // they are not queried anywhere within the search.
                search->best_goal = *succ_state;
            }
        }
    }

    assert(ClosedInAnySearch(state));
}

static
void ClearOpenLists(SMHAStar* search)
{
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        search->open[i].clear();
    }
}

static
void ReinitSearch(SMHAStar* search)
{
    ClearOpenLists(search);
}

static
void Clear(SMHAStar* search)
{
    ClearOpenLists(search);

    // free states
    for (auto& state : search->search_states) {
        if (state != NULL) {
            free(state);
            state = NULL;
        }
    }

    // empty state table
    search->search_states.clear();

    search->start_state = NULL;
    search->goal = NULL;
}

static
void ExtractPath(SMHAStar* search, std::vector<int>* path, int* cost)
{
    SMPL_DEBUG_NAMED(LOG, "Extracting path");
    path->clear();
    *cost = 0;
    for (auto* state = &search->best_goal; state != NULL; state = state->bp) {
        path->push_back(state->state_id);
        if (state->bp) {
            *cost += (state->g - state->bp->g);
        }
    }

    std::reverse(begin(*path), end(*path));
}

///////////////
// Interface //
///////////////

bool Init(
    SMHAStar* search,
    DiscreteSpace* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int heur_count)
{
    if (space == NULL || anchor == NULL || heurs == NULL || heur_count < 0) {
        return false;
    }

    auto* searchable = space->GetExtension<ISearchable>();
    if (searchable == NULL) {
        return false;
    }

    auto goal_heuristics = std::vector<IGoalHeuristic*>();
    goal_heuristics.reserve(heur_count + 1);

    auto* anchor_goal_heuristic = anchor->GetExtension<IGoalHeuristic>();
    if (anchor_goal_heuristic == NULL) {
        return false;
    }

    goal_heuristics.push_back(anchor_goal_heuristic);

    for (auto i = 0; i < heur_count; ++i) {
        auto* goal_heuristic = heurs[i]->GetExtension<IGoalHeuristic>();
        if (goal_heuristic == NULL) {
            return false;
        }
        goal_heuristics.push_back(goal_heuristic);
    }

    search->space = searchable;
    search->heurs = std::move(goal_heuristics);
    search->heur_count = heur_count;
    search->open.resize(search->heur_count + 1);
    return true;
}

auto GetTargetEpsilon(const SMHAStar* search) -> double
{
    return search->w_heur_final;
}

void SetTargetEpsilon(SMHAStar* search, double eps)
{
    search->w_heur_final = eps;
}

auto GetDeltaEpsilon(const SMHAStar* search) -> double
{
    return search->w_heur_delta;
}

void SetDeltaEpsilon(SMHAStar* search, double eps)
{
    search->w_heur_delta = eps;
}

auto GetInitialEps(const SMHAStar* search) -> double
{
    return search->w_heur_init;
}

void SetInitialEps(SMHAStar* search, double eps)
{
    search->w_heur_init = eps;
}

auto GetInitialMHAEps(const SMHAStar* search) -> double
{
    return search->w_anchor_init;
}

void SetInitialMHAEps(SMHAStar* search, double eps)
{
    search->w_anchor_init = eps;
}

int GetMaxExpansions(const SMHAStar* search)
{
    return search->max_expansions;
}

void SetMaxExpansions(SMHAStar* search, int expansion_count)
{
    search->max_expansions = expansion_count;
}

auto GetSolutionEps(const SMHAStar* search) -> double
{
    return search->w_heur_satisfied;
}

int GetNumExpansions(SMHAStar* search)
{
    return search->num_expansions;
}

int GetNumExpansionsInitialEps(const SMHAStar* search)
{
    return search->num_expansions;
}

auto GetElapsedTime(SMHAStar* search) -> double
{
    return search->elapsed;
}

auto GetElapsedTimeInitialEps(const SMHAStar* search) -> double
{
    return search->elapsed;
}

bool UpdateStart(SMHAStar* search, int start_state)
{
    search->start_state = GetState(search, start_state);
    if (search->start_state == NULL) {
        return false;
    }
    return true;
}

bool UpdateGoal(SMHAStar* search, GoalConstraint* goal)
{
    search->goal = goal;
    return true;
}

void ForcePlanningFromScratch(SMHAStar* search)
{
}

void ForcePlanningFromScratchAndFreeMemory(SMHAStar* search)
{
}

int Replan(
    SMHAStar* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_INFO_NAMED(LOG, "Generic Search parameters:");
    SMPL_INFO_NAMED(LOG, "  Initial Epsilon: %0.3f", search->w_heur_init);
    SMPL_INFO_NAMED(LOG, "  Final Epsilon: %0.3f", search->w_heur_final);
    SMPL_INFO_NAMED(LOG, "  Delta Epsilon: %0.3f", search->w_heur_delta);
    SMPL_INFO_NAMED(LOG, "MHA Search parameters:");
    SMPL_INFO_NAMED(LOG, "  MHA Epsilon: %0.3f", search->w_anchor_init);
    SMPL_INFO_NAMED(LOG, "  Max Expansions: %d", search->max_expansions);

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    ReinitSearch(search);

    search->w_heur = search->w_heur_init;
    search->w_anchor = search->w_anchor_init;
    search->w_heur_satisfied = 0.0;

    // reset time limits
    search->num_expansions = 0;
    search->elapsed = 0.0;

    auto start_time = GetTime();

    ++search->call_number;
    ReinitState(search, &search->best_goal, true);
    ReinitState(search, search->start_state);
    search->start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        auto key = ComputeKey(search, search->start_state, i);
        search->start_state->od[i].f = key;
        search->open[i].push(&search->start_state->od[i]);
        SMPL_DEBUG_NAMED(LOG, "Inserted start state %d into search %d with f = %d", search->start_state->state_id, i, key);
    }

    auto end_time = GetTime();
    search->elapsed += (end_time - start_time);

    while (!search->open[0].empty() && !TimeLimitReached(search, timeout)) {
        start_time = GetTime();

        // special case for mha* without additional heuristics
        if (NumHeuristics(search) == 1) {
            SMPL_WARN_ONCE("Running SMHA* with only the anchor search?");
            if (search->best_goal.g <= GetMinF(search->open[0])) {
                search->w_heur_satisfied = search->w_heur * search->w_anchor;
                ExtractPath(search, solution, cost);
                return 1;
            } else {
                auto* s = StateFromOpenState(search->open[0].min());
                Expand(search, s, 0);
            }
        }

        for (auto hidx = 1; hidx < NumHeuristics(search); ++hidx) {
            if (search->open[0].empty()) {
                break;
            }

            if (!search->open[hidx].empty()) {
                SMPL_DEBUG_NAMED(ELOG, "Compare search %d f_min %d against anchor f_min %d", hidx, GetMinF(search->open[hidx]), GetMinF(search->open[0]));
            }

            if (!search->open[hidx].empty() && GetMinF(search->open[hidx]) <=
                search->w_anchor * GetMinF(search->open[0]))
            {
                if (search->best_goal.g <= GetMinF(search->open[hidx])) {
                    search->w_heur_satisfied = search->w_heur * search->w_anchor;
                    ExtractPath(search, solution, cost);
                    return 1;
                } else {
                    auto* s = StateFromOpenState(search->open[hidx].min());
                    Expand(search, s, hidx);
                }
            } else {
                if (search->best_goal.g <= GetMinF(search->open[0])) {
                    search->w_heur_satisfied = search->w_heur * search->w_anchor;
                    ExtractPath(search, solution, cost);
                    return 1;
                } else {
                    auto* s = StateFromOpenState(search->open[0].min());
                    Expand(search, s, 0);
                }
            }
        }
        end_time = GetTime();
        search->elapsed += (end_time - start_time);
    }

    if (search->open[0].empty()) {
        SMPL_DEBUG_NAMED(LOG, "Anchor search exhausted");
    }
    if (TimeLimitReached(search, timeout)) {
        SMPL_DEBUG_NAMED(LOG, "Time limit reached");
    }

    return 0;
}

bool SMHAStar::HeapCompare::operator()(
    const SMHAState::HeapData& s, const SMHAState::HeapData& t) const
{
    return s.f < t.f;
}

SMHAStar::~SMHAStar()
{
    Clear(this);
}

int SMHAStar::GetNumExpansions()
{
    return ::smpl::GetNumExpansions(this);
}

auto SMHAStar::GetElapsedTime() -> double
{
    return ::smpl::GetElapsedTime(this);
}

bool SMHAStar::UpdateStart(int state_id)
{
    return ::smpl::UpdateStart(this, state_id);
}

bool SMHAStar::UpdateGoal(GoalConstraint* goal)
{
    return ::smpl::UpdateGoal(this, goal);
}

void SMHAStar::ForcePlanningFromScratch()
{
    return ::smpl::ForcePlanningFromScratch(this);
}

void SMHAStar::ForcePlanningFromScratchAndFreeMemory()
{
    return ::smpl::ForcePlanningFromScratchAndFreeMemory(this);
}

int SMHAStar::Replan(
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    return ::smpl::Replan(this, timeout, solution, cost);
}

} // namespace smpl
