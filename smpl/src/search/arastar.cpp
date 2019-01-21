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

#include <smpl/search/arastar.h>

// standard includes
#include <algorithm>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/heuristic.h>

namespace smpl {

constexpr auto INFINITECOST = 1000000000;

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

////////////////////
// Implementation //
////////////////////

// Recompute heuristics for all states.
static
void RecomputeHeuristics(ARAStar* search)
{
    for (auto* s : search->states) {
        if (s != NULL) {
            s->h = search->heur->GetGoalHeuristic(s->state_id);
        }
    }
}

// Test whether the search has run out of time.
static
bool TimedOut(
    const ARAStar* search,
    int elapsed_expansions,
    const clock::duration& elapsed_time)
{
    if (!search->time_params.bounded) {
        return false;
    }

    switch (search->time_params.type) {
    case TimeoutCondition::EXPANSIONS:
        if (search->satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= search->time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= search->time_params.max_expansions;
        }
    case TimeoutCondition::TIME:
        if (search->satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= search->time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= search->time_params.max_allowed_time;
        }
    case TimeoutCondition::USER:
        return search->time_params.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

// Create a new search state for a graph state.
static
auto CreateState(int state_id) -> ARASearchState*
{
    auto* ss = new ARASearchState;
    ss->state_id = state_id;
    ss->call_number = 0;
    return ss;
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
static
auto GetSearchState(ARAStar* search, int state_id) -> ARASearchState*
{
    if (search->states.size() <= state_id) {
        search->states.resize(state_id + 1, nullptr);
    }

    auto& state = search->states[state_id];
    if (state == NULL) {
        state = CreateState(state_id);
    }

    return state;
}

// Lazily (re)initialize a search state.
static
void ReinitSearchState(ARAStar* search, ARASearchState* state, bool goal = false)
{
    if (state->call_number != search->call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        if (goal) {
            state->h = 0;
        } else {
            state->h = search->heur->GetGoalHeuristic(state->state_id);
        }
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = search->call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

static
int ComputeKey(const ARAStar* search, ARASearchState* s)
{
    return s->g + (int)(search->curr_eps * (double)s->h);
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
static
void Expand(ARAStar* search, ARASearchState* s)
{
    search->succs.clear();
    search->costs.clear();
    search->space->GetSuccs(s->state_id, &search->succs, &search->costs);

    SMPL_DEBUG_NAMED(SELOG, "  %zu successors", search->succs.size());

    for (auto sidx = 0; sidx < search->succs.size(); ++sidx) {
        auto succ_state_id = search->succs[sidx];
        auto cost = search->costs[sidx];

        auto* succ_state = GetSearchState(search, succ_state_id);
        ReinitSearchState(search, succ_state);

        auto new_cost = s->eg + cost;
        SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != search->iteration) {
                succ_state->f = ComputeKey(search, succ_state);
                if (search->open.contains(succ_state)) {
                    SMPL_DEBUG_NAMED(SELOG, "  Update with new priority %ld", succ_state->f);
                    search->open.decrease(succ_state);
                } else {
                    SMPL_DEBUG_NAMED(SELOG, "  Insert into OPEN with priority %ld", succ_state->f);
                    search->open.push(succ_state);
                }
            } else if (!succ_state->incons) {
                SMPL_DEBUG_NAMED(SELOG, "  Insert into INCONS");
                search->incons.push_back(succ_state);
            }
            if (search->goal->IsGoal(succ_state->state_id)) {
                search->best_goal = *succ_state;
            }
        }
    }
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
static
int ImprovePath(
    ARAStar* search,
    const clock::time_point& start_time,
    ARASearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    while (!search->open.empty()) {
        auto* min_state = search->open.min();

        auto now = clock::now();
        elapsed_time = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || search->goal->IsGoal(min_state->state_id)/* TODO: IsGoal necessary? */) {
            SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
            return SUCCESS;
        }

        if (TimedOut(search, elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }

        SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        search->open.pop();

        assert(min_state->iteration_closed != search->iteration);
        assert(min_state->g != INFINITECOST);

        min_state->iteration_closed = search->iteration;
        min_state->eg = min_state->g;

        Expand(search, min_state);

        ++elapsed_expansions;
    }

    return EXHAUSTED_OPEN_LIST;
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
static
void ReorderOpen(ARAStar* search)
{
    for (auto it = search->open.begin(); it != search->open.end(); ++it) {
        (*it)->f = ComputeKey(search, *it);
    }
    search->open.make();
}

// Extract the path from the start state up to a new state.
static
void ExtractPath(
    const ARAStar* search,
    ARASearchState* to_state,
    std::vector<int>& solution,
    int& cost)
{
    for (auto* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

///////////////
// Interface //
///////////////

bool Init(ARAStar* search, DiscreteSpace* space, Heuristic* heuristic)
{
    if (space == NULL || heuristic == NULL) return false;

    auto* searchable = space->GetExtension<ISearchable>();
    if (searchable == NULL) {
        SMPL_ERROR("ARA* requires an ISearchable extension to DiscreteSpace");
        return false;
    }

    auto* goal_heuristic = heuristic->GetExtension<IGoalHeuristic>();
    if (goal_heuristic == NULL) {
        SMPL_ERROR("ARA* requires an IGoalHeursitic extension to Heuristic");
        return false;
    }

    search->space = searchable;
    search->heur = goal_heuristic;
    return true;
}

/// Return the initial suboptimality bound
auto GetInitialEps(const ARAStar* search) -> double
{
    return search->initial_eps;
}

/// Set the desired suboptimality bound for the initial solution.
void SetInitialEps(ARAStar* search, double eps)
{
    search->initial_eps = eps;
}

auto GetTargetEps(const ARAStar* search) -> double
{
    return search->final_eps;
}

void SetTargetEps(ARAStar* search, double target_eps)
{
    search->final_eps = std::max(target_eps, 1.0);
}

auto GetDeltaEps(const ARAStar* search) -> double
{
    return search->delta_eps;
}

void SetDeltaEps(ARAStar* search, double delta_eps)
{
    assert(delta_eps > 0.0);
    search->delta_eps = delta_eps;
}

bool AllowPartialSolutions(const ARAStar* search)
{
    return search->allow_partial_solutions;
}

void SetAllowPartialSolutions(ARAStar* search, bool enabled)
{
    search->allow_partial_solutions = enabled;
}

/// Set the start state.
bool UpdateStart(ARAStar* search, int start_state_id)
{
    search->start_state_id = start_state_id;
    return true;
}

/// Set the goal state.
bool UpdateGoal(ARAStar* search, GoalConstraint* goal)
{
    search->goal = goal;
    search->new_goal = true;
    return true;
}

/// Force the search to forget previous search efforts and start from scratch.
void ForcePlanningFromScratch(ARAStar* search)
{
    search->last_start_state_id = -1;
    search->new_goal = true;
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
void ForcePlanningFromScratchAndFreeMemory(ARAStar* search)
{
    ForcePlanningFromScratch(search);
    search->open.clear();
    for (auto* s : search->states) {
        if (s != NULL) {
            delete s;
        }
    }
    search->states.clear();
    search->states.shrink_to_fit();
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int Replan(
    ARAStar* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (search->start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (search->goal == NULL) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    search->time_params = timeout;

    auto* start_state = GetSearchState(search, search->start_state_id);
    auto* goal_state = &search->best_goal;

    if (search->start_state_id != search->last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        search->open.clear();
        search->incons.clear();
        ++search->call_number; // trigger state reinitializations

        ReinitSearchState(search, start_state);
        ReinitSearchState(search, goal_state, true);

        start_state->g = 0;
        start_state->f = ComputeKey(search, start_state);
        search->open.push(start_state);

        search->iteration = 1; // 0 reserved for "not closed on any iteration"

        search->expand_count_init = 0;
        search->search_time_init = clock::duration::zero();

        search->expand_count = 0;
        search->search_time = clock::duration::zero();

        search->curr_eps = search->initial_eps;

        search->satisfied_eps = std::numeric_limits<double>::infinity();

        search->last_start_state_id = search->start_state_id;
    }

    if (search->new_goal) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        RecomputeHeuristics(search);
        ReorderOpen(search);

        search->new_goal = false;
    }

    auto start_time = clock::now();
    auto num_expansions = 0;
    auto elapsed_time = clock::duration::zero();

    int err;
    while (search->satisfied_eps > search->final_eps) {
        if (search->curr_eps == search->satisfied_eps) {
            if (!timeout.improve) {
                break;
            }
            // begin a new search iteration
            ++search->iteration;
            search->curr_eps -= search->delta_eps;
            search->curr_eps = std::max(search->curr_eps, search->final_eps);
            for (auto* s : search->incons) {
                s->incons = false;
                search->open.push(s);
            }
            ReorderOpen(search);
            search->incons.clear();
            SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", search->iteration, search->curr_eps);
        }
        err = ImprovePath(search, start_time, goal_state, num_expansions, elapsed_time);
        if (search->curr_eps == search->initial_eps) {
            search->expand_count_init += num_expansions;
            search->search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        SMPL_DEBUG_NAMED(SLOG, "Improved solution");
        search->satisfied_eps = search->curr_eps;
    }

    search->search_time += elapsed_time;
    search->expand_count += num_expansions;

    if (search->satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (search->allow_partial_solutions && !search->open.empty()) {
            auto* next_state = search->open.min();
            ExtractPath(search, next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }

    ExtractPath(search, goal_state, *solution, *cost);
    return !SUCCESS;
}

/// Return the suboptimality bound of the current solution for the current search.
auto GetSolutionEps(const ARAStar* search) -> double
{
    return search->satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int GetNumExpansions(const ARAStar* search)
{
    return search->expand_count;
}

/// Return the number of expansions made in progress to the initial solution.
int GetNumExpansionsInitialEps(const ARAStar* search)
{
    return search->expand_count_init;
}

/// Return the time consumed by the search in progress to the final solution.
auto GetElapsedTime(const ARAStar* search) -> double
{
    return to_seconds(search->search_time);
}

/// Return the time consumed by the search in progress to the initial solution.
auto GetElapsedTimeInitialEps(const ARAStar* search) -> double
{
    return to_seconds(search->search_time_init);
}

ARAStar::ARAStar() :
    search_time_init(clock::duration::zero()),
    search_time(clock::duration::zero()),
    satisfied_eps(std::numeric_limits<double>::infinity())
{
    // to ensure reinitialization is triggered
    this->best_goal.call_number = 0;
}

ARAStar::~ARAStar()
{
    for (auto* s : states) {
        if (s != NULL) {
            delete s;
        }
    }
}

bool ARAStar::UpdateStart(int state_id)
{
    return ::smpl::UpdateStart(this, state_id);
}

bool ARAStar::UpdateGoal(GoalConstraint* goal)
{
    return ::smpl::UpdateGoal(this, goal);
}

void ARAStar::ForcePlanningFromScratch()
{
    return ::smpl::ForcePlanningFromScratch(this);
}

void ARAStar::ForcePlanningFromScratchAndFreeMemory()
{
    return ::smpl::ForcePlanningFromScratchAndFreeMemory(this);
}

int ARAStar::Replan(
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    return ::smpl::Replan(this, timeout, solution, cost);
}

int ARAStar::GetNumExpansions()
{
    return ::smpl::GetNumExpansions(this);
}

auto ARAStar::GetElapsedTime() -> double
{
    return ::smpl::GetElapsedTime(this);
}


} // namespace smpl
