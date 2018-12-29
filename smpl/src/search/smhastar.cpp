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

constexpr auto INFINITECOST = 1000000000;

static const char* LOG = "search.smhastar";
static const char* ELOG = "search.smhastar.expansions";
static const char* SLOG = "search.smhastar.successors";

static
auto GetTime() -> double
{
    return to_seconds(clock::now().time_since_epoch());
}

static
int NumHeuristics(const SMHAStar* search)
{
    return search->m_heur_count;
}

static
int ComputeHeuristic(SMHAStar* search, int state_id, int hidx)
{
    if (hidx == 0) {
        return search->m_anchor->GetGoalHeuristic(state_id);
    } else {
        return search->m_heurs[hidx - 1]->GetGoalHeuristic(state_id);
    }
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
    if (search->m_search_states.size() <= state_id) {
        search->m_search_states.resize(state_id + 1, NULL);
    }

    auto& state = search->m_search_states[state_id];
    if (state == NULL) {
        // overallocate search state for appropriate heuristic information
        auto state_size =
                sizeof(SMHAState) +
                sizeof(SMHAState::HeapData) * (search->m_heur_count);
        auto* s = (SMHAState*)malloc(state_size);

        new (s) SMHAState;
        for (auto i = 0; i < search->m_heur_count; ++i) {
            new (&s->od[1 + i]) SMHAState::HeapData;
        }

        auto mha_state_idx = (int)search->m_search_states.size();
        InitState(search, s, mha_state_idx, state_id);

        // map graph state to search state
        state = s;
    }

    return state;
}

static
void ReinitState(SMHAStar* search, SMHAState* state, bool goal = false)
{
    if (state->call_number != search->m_call_number) {
        state->call_number = search->m_call_number;
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
bool TimeLimitReached(const SMHAStar* search)
{
    if (search->m_params.return_first_solution) {
        return false;
    } else if (search->m_params.max_time > 0.0 && search->m_elapsed >= search->m_params.max_time) {
        return true;
    } else if (search->m_max_expansions > 0 && search->m_num_expansions >= search->m_max_expansions) {
        return true;
    } else {
        return false;
    }
}

static
int ComputeKey(const SMHAStar* search, SMHAState* state, int hidx)
{
    return (int)((double)state->g + search->m_eps * (double)state->od[hidx].h);
}

static
int GetMinF(const SMHAStar::OpenList& pq)
{
    return pq.min()->f;
}

static
void InsertOrUpdate(SMHAStar* search, SMHAState* state, int hidx)
{
    if (search->m_open[hidx].contains(&state->od[hidx])) {
        search->m_open[hidx].update(&state->od[hidx]);
    } else {
        search->m_open[hidx].push(&state->od[hidx]);
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
    ++search->m_num_expansions;

    // remove s from all open lists
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        if (search->m_open[i].contains(&state->od[i])) {
            search->m_open[i].erase(&state->od[i]);
        }
    }

    auto succ_ids = std::vector<int>();
    auto costs = std::vector<int>();
    search->m_space->GetSuccs(state->state_id, &succ_ids, &costs);
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
                        if (fn <= search->m_eps_mha * fanchor) {
                            succ_state->od[i].f = fn;
                            InsertOrUpdate(search, succ_state, i);
                            SMPL_DEBUG_NAMED(LOG, "  Update in search %d with f = %d", i, fn);
                        } else {
                            SMPL_DEBUG_NAMED(LOG, "  Skipping update of in search %d (%0.3f > %0.3f)", i, (double)fn, search->m_eps_mha * fanchor);
                        }
                    }
                }
            }
            if (search->m_goal->IsGoal(succ_ids[sidx])) {
                // NOTE: This assignment will not assign the N additional
                // heuristic values. It is fine to ignore those here, since
                // they are not queried anywhere within the search.
                search->m_best_goal = *succ_state;
            }
        }
    }

    assert(ClosedInAnySearch(state));
}

static
bool CheckParams(const SMHAStar* search, const ReplanParams& params)
{
    if (params.initial_eps < 1.0) {
        SMPL_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.final_eps > params.initial_eps) {
        SMPL_ERROR("Final Epsilon must be less than or equal to initial epsilon");
        return false;
    }

    if (params.dec_eps <= 0.0) {
        SMPL_ERROR("Delta epsilon must be strictly positive");
        return false;
    }

    if (search->m_initial_eps_mha < 1.0) {
        SMPL_ERROR("MHA Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.return_first_solution &&
        params.max_time <= 0.0 &&
        search->m_max_expansions <= 0)
    {
        SMPL_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;
}

static
void ClearOpenLists(SMHAStar* search)
{
    for (auto i = 0; i < NumHeuristics(search); ++i) {
        search->m_open[i].clear();
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
    for (auto& state : search->m_search_states) {
        if (state != NULL) {
            free(state);
            state = NULL;
        }
    }

    // empty state table
    search->m_search_states.clear();

    search->m_start_state = NULL;
    search->m_goal = NULL;
}

static
void ExtractPath(SMHAStar* search, std::vector<int>* path, int* cost)
{
    SMPL_DEBUG_NAMED(LOG, "Extracting path");
    path->clear();
    *cost = 0;
    for (auto* state = &search->m_best_goal; state != NULL; state = state->bp) {
        path->push_back(state->state_id);
        if (state->bp) {
            *cost += (state->g - state->bp->g);
        }
    }

    std::reverse(begin(*path), end(*path));
}

SMHAStar::SMHAStar()
{
    // Overwrite default members for ReplanParams to represent a single optimal
    // search
    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.2; // NOTE: same initial epsilon delta as ARA*
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    m_eps_satisfied = (double)INFINITECOST;

    /// Four Modes:
    ///     Search Until Solution Bounded
    ///     Search Until Solution Unbounded
    ///     Improve Solution Bounded
    ///     Improve Solution Unbounded
}

SMHAStar::~SMHAStar()
{
    Clear(this);
    if (m_open != NULL) {
        delete[] m_open;
    }
}

bool SMHAStar::Init(
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

    auto* anchor_goal_heuristic = anchor->GetExtension<IGoalHeuristic>();
    if (anchor_goal_heuristic == NULL) {
        return false;
    }

    auto goal_heuristics = std::vector<IGoalHeuristic*>();
    for (auto i = 0; i < heur_count; ++i) {
        auto* goal_heuristic = heurs[i]->GetExtension<IGoalHeuristic>();
        if (goal_heuristic == NULL) {
            return false;
        }
        goal_heuristics.push_back(goal_heuristic);
    }

    m_space = searchable;
    m_anchor = anchor_goal_heuristic;
    m_heurs = std::move(goal_heuristics);
    m_heur_count = heur_count;
    m_open = new OpenList[heur_count + 1];
    return true;
}

void SMHAStar::SetTargetEpsilon(double eps)
{
    m_params.final_eps = eps;
}

auto SMHAStar::GetTargetEpsilon() const -> double
{
    return m_params.final_eps;
}

void SMHAStar::SetDeltaEpsilon(double eps)
{
    m_params.dec_eps = eps;
}

auto SMHAStar::GetDeltaEpsilon() const -> double
{
    return m_params.dec_eps;
}

void SMHAStar::SetInitialEps(double eps)
{
    m_params.initial_eps = eps;
}

auto SMHAStar::GetInitialEps() const -> double
{
    return m_params.initial_eps;
}

int SMHAStar::SetSearchMode(bool bSearchUntilFirstSolution)
{
    return m_params.return_first_solution = bSearchUntilFirstSolution;
}

void SMHAStar::SetInitialMHAEps(double eps)
{
    m_initial_eps_mha = eps;
}

auto SMHAStar::GetInitialMHAEps() const -> double
{
    return m_initial_eps_mha;
}

void SMHAStar::SetMaxExpansions(int expansion_count)
{
    m_max_expansions = expansion_count;
}

int SMHAStar::GetMaxExpansions() const
{
    return m_max_expansions;
}

void SMHAStar::SetMaxTime(double max_time)
{
    m_params.max_time = max_time;
}

auto SMHAStar::GetMaxTime() const -> double
{
    return m_params.max_time;
}

auto SMHAStar::GetSolutionEps() const -> double
{
    return m_eps_satisfied;
}

int SMHAStar::GetNumExpansions() const
{
    return m_num_expansions;
}

int SMHAStar::GetNumExpansionsInitialEps() const
{
    return m_num_expansions;
}

auto SMHAStar::GetElapsedTime() const -> double
{
    return m_elapsed;
}

auto SMHAStar::GetElapsedTimeInitialEps() const -> double
{
    return m_elapsed;
}

bool SMHAStar::UpdateStart(int start_state)
{
    m_start_state = GetState(this, start_state);
    if (m_start_state == NULL) {
        return false;
    }
    return true;
}

bool SMHAStar::UpdateGoal(GoalConstraint* goal)
{
    m_goal = goal;
    return true;
}

void SMHAStar::UpdateCosts(const StateChangeQuery& changes)
{
}

void SMHAStar::UpdateCosts()
{
}

void SMHAStar::ForcePlanningFromScratch()
{
}

void SMHAStar::ForcePlanningFromScratchAndFreeMemory()
{
}

int SMHAStar::Replan(double allowed_time, std::vector<int>* solution)
{
    int cost;
    return Replan(allowed_time, solution, &cost);
}

int SMHAStar::Replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    ReplanParams params = m_params;
    params.max_time = allowed_time;
    return Replan(params, solution, cost);
}

int SMHAStar::Replan(ReplanParams params, std::vector<int>* solution)
{
    int cost;
    return Replan(params, solution, &cost);
}

int SMHAStar::Replan(
    ReplanParams params,
    std::vector<int>* solution,
    int* cost)
{
    if (!CheckParams(this, params)) { // errors printed within
        return 0;
    }

    m_params = params;

    SMPL_INFO_NAMED(LOG, "Generic Search parameters:");
    SMPL_INFO_NAMED(LOG, "  Initial Epsilon: %0.3f", m_params.initial_eps);
    SMPL_INFO_NAMED(LOG, "  Final Epsilon: %0.3f", m_params.final_eps);
    SMPL_INFO_NAMED(LOG, "  Delta Epsilon: %0.3f", m_params.dec_eps);
    SMPL_INFO_NAMED(LOG, "  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SMPL_INFO_NAMED(LOG, "  Max Time: %0.3f", m_params.max_time);
    SMPL_INFO_NAMED(LOG, "  Repair Time: %0.3f", m_params.repair_time);
    SMPL_INFO_NAMED(LOG, "MHA Search parameters:");
    SMPL_INFO_NAMED(LOG, "  MHA Epsilon: %0.3f", m_initial_eps_mha);
    SMPL_INFO_NAMED(LOG, "  Max Expansions: %d", m_max_expansions);

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    ReinitSearch(this);

    m_eps = m_params.initial_eps;
    m_eps_mha = m_initial_eps_mha;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = 0.0;

    auto start_time = GetTime();

    ++m_call_number;
    ReinitState(this, &m_best_goal, true);
    ReinitState(this, m_start_state);
    m_start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (auto i = 0; i < NumHeuristics(this); ++i) {
        auto key = ComputeKey(this, m_start_state, i);
        m_start_state->od[i].f = key;
        m_open[i].push(&m_start_state->od[i]);
        SMPL_DEBUG_NAMED(LOG, "Inserted start state %d into search %d with f = %d", m_start_state->state_id, i, key);
    }

    auto end_time = GetTime();
    m_elapsed += (end_time - start_time);

    while (!m_open[0].empty() && !TimeLimitReached(this)) {
        start_time = GetTime();

        // special case for mha* without additional heuristics
        if (NumHeuristics(this) == 1) {
            SMPL_WARN_ONCE("Running SMHA* with only the anchor search?");
            if (m_best_goal.g <= GetMinF(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                ExtractPath(this, solution, cost);
                return 1;
            } else {
                auto* s = StateFromOpenState(m_open[0].min());
                Expand(this, s, 0);
            }
        }

        for (auto hidx = 1; hidx < NumHeuristics(this); ++hidx) {
            if (m_open[0].empty()) {
                break;
            }

            if (!m_open[hidx].empty()) {
                SMPL_DEBUG_NAMED(ELOG, "Compare search %d f_min %d against anchor f_min %d", hidx, GetMinF(m_open[hidx]), GetMinF(m_open[0]));
            }

            if (!m_open[hidx].empty() && GetMinF(m_open[hidx]) <=
                m_eps_mha * GetMinF(m_open[0]))
            {
                if (m_best_goal.g <= GetMinF(m_open[hidx])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    ExtractPath(this, solution, cost);
                    return 1;
                } else {
                    auto* s = StateFromOpenState(m_open[hidx].min());
                    Expand(this, s, hidx);
                }
            } else {
                if (m_best_goal.g <= GetMinF(m_open[0])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    ExtractPath(this, solution, cost);
                    return 1;
                } else {
                    auto* s = StateFromOpenState(m_open[0].min());
                    Expand(this, s, 0);
                }
            }
        }
        end_time = GetTime();
        m_elapsed += (end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_DEBUG_NAMED(LOG, "Anchor search exhausted");
    }
    if (TimeLimitReached(this)) {
        SMPL_DEBUG_NAMED(LOG, "Time limit reached");
    }

    return 0;
}

} // namespace smpl
