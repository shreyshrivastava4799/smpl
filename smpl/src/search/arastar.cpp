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

ARAStar::ARAStar() :
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity())
{
    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeoutCondition::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
    m_best_goal.call_number = 0;
}

bool ARAStar::Init(DiscreteSpace* space, Heuristic* heuristic)
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

    m_space = searchable;
    m_heur = goal_heuristic;
    return true;
}

void ARAStar::SetAllowPartialSolutions(bool enabled)
{
    m_allow_partial_solutions = enabled;
}

bool ARAStar::AllowPartialSolutions() const
{
    return m_allow_partial_solutions;
}

void ARAStar::SetAllowedRepairTime(double allowed_time_secs)
{
    m_time_params.max_allowed_time = to_duration(allowed_time_secs);
}

double ARAStar::GetAllowedRepairTime() const
{
    return to_seconds(m_time_params.max_allowed_time);
}

void ARAStar::SetTargetEpsilon(double target_eps)
{
    m_final_eps = std::max(target_eps, 1.0);
}

double ARAStar::GetTargetEpsilon() const
{
    return m_final_eps;
}

void ARAStar::SetDeltaEpsilon(double delta_eps)
{
    assert(delta_eps > 0.0);
    m_delta_eps = delta_eps;
}

double ARAStar::GetDeltaEpsilon() const
{
    return m_delta_eps;
}

void ARAStar::SetImproveSolution(bool improve)
{
    m_time_params.improve = improve;
}

bool ARAStar::ImproveSolution() const
{
    return m_time_params.improve;
}

void ARAStar::SetBoundExpansions(bool bound)
{
    m_time_params.bounded = bound;
}

bool ARAStar::BoundExpansions() const
{
    return m_time_params.bounded;
}

ARAStar::~ARAStar()
{
    for (auto* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
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

int ARAStar::Replan(
    const TimeoutCondition& params,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal == NULL) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;

    auto* start_state = GetSearchState(m_start_state_id);
    auto* goal_state = &m_best_goal;

    if (m_start_state_id != m_last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        ReinitSearchState(start_state);
        ReinitSearchState(goal_state, true);

        start_state->g = 0;
        start_state->f = ComputeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_new_goal) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        RecomputeHeuristics();
        ReorderOpen();

        m_new_goal = false;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    while (m_satisfied_eps > m_final_eps) {
        if (m_curr_eps == m_satisfied_eps) {
            if (!m_time_params.improve) {
                break;
            }
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            for (auto* s : m_incons) {
                s->incons = false;
                m_open.push(s);
            }
            ReorderOpen();
            m_incons.clear();
            SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = ImprovePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        SMPL_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open.empty()) {
            auto* next_state = m_open.min();
            ExtractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }

    ExtractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int ARAStar::Replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return Replan(allowed_time, solution, &cost);
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
int ARAStar::Replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    TimeoutCondition tparams = m_time_params;
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    return Replan(tparams, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
void ARAStar::ForcePlanningFromScratchAndFreeMemory()
{
    ForcePlanningFromScratch();
    m_open.clear();
    for (auto* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
    m_states.clear();
    m_states.shrink_to_fit();
}

/// Return the suboptimality bound of the current solution for the current search.
double ARAStar::GetSolutionEps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int ARAStar::GetNumExpansions()
{
    return m_expand_count;
}

/// Return the initial suboptimality bound
double ARAStar::GetInitialEps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double ARAStar::GetElapsedTimeInitialEps()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double ARAStar::GetElapsedTime()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int ARAStar::GetNumExpansionsInitialEps()
{
    return m_expand_count_init;
}

/// Set the desired suboptimality bound for the initial solution.
void ARAStar::SetInitialEps(double eps)
{
    m_initial_eps = eps;
}

/// Set the goal state.
bool ARAStar::UpdateGoal(GoalConstraint* goal)
{
    m_goal = goal;
    m_new_goal = true;
    return true;
}

/// Set the start state.
bool ARAStar::UpdateStart(int start_state_id)
{
    m_start_state_id = start_state_id;
    return true;
}

/// Force the search to forget previous search efforts and start from scratch.
void ARAStar::ForcePlanningFromScratch()
{
    m_last_start_state_id = -1;
    m_new_goal = true;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int ARAStar::SetSearchMode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void ARAStar::UpdateCosts(const StateChangeQuery& changes)
{
    ForcePlanningFromScratch();
}

// Recompute heuristics for all states.
void ARAStar::RecomputeHeuristics()
{
    for (auto* s : m_states) {
        if (s != NULL) {
            s->h = m_heur->GetGoalHeuristic(s->state_id);
        }
    }
}

// Test whether the search has run out of time.
bool ARAStar::TimedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeoutCondition::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeoutCondition::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    case TimeoutCondition::USER:
        return m_time_params.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int ARAStar::ImprovePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    while (!m_open.empty()) {
        auto* min_state = m_open.min();

        auto now = clock::now();
        elapsed_time = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || m_goal->IsGoal(min_state->state_id)/* TODO: IsGoal necessary? */) {
            SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
            return SUCCESS;
        }

        if (TimedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }

        SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        m_open.pop();

        assert(min_state->iteration_closed != m_iteration);
        assert(min_state->g != INFINITECOST);

        min_state->iteration_closed = m_iteration;
        min_state->eg = min_state->g;

        Expand(min_state);

        ++elapsed_expansions;
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void ARAStar::Expand(SearchState* s)
{
    m_succs.clear();
    m_costs.clear();
    m_space->GetSuccs(s->state_id, &m_succs, &m_costs);

    SMPL_DEBUG_NAMED(SELOG, "  %zu successors", m_succs.size());

    for (size_t sidx = 0; sidx < m_succs.size(); ++sidx) {
        auto succ_state_id = m_succs[sidx];
        auto cost = m_costs[sidx];

        auto* succ_state = GetSearchState(succ_state_id);
        ReinitSearchState(succ_state);

        auto new_cost = s->eg + cost;
        SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != m_iteration) {
                succ_state->f = ComputeKey(succ_state);
                if (m_open.contains(succ_state)) {
                    SMPL_DEBUG_NAMED(SELOG, "  Update with new priority %ld", succ_state->f);
                    m_open.decrease(succ_state);
                } else {
                    SMPL_DEBUG_NAMED(SELOG, "  Insert into OPEN with priority %ld", succ_state->f);
                    m_open.push(succ_state);
                }
            } else if (!succ_state->incons) {
                SMPL_DEBUG_NAMED(SELOG, "  Insert into INCONS");
                m_incons.push_back(succ_state);
            }
            if (m_goal->IsGoal(succ_state->state_id)) {
                m_best_goal = *succ_state;
            }
        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void ARAStar::ReorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        (*it)->f = ComputeKey(*it);
    }
    m_open.make();
}

int ARAStar::ComputeKey(SearchState* s) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
auto ARAStar::GetSearchState(int state_id) -> SearchState*
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }

    auto& state = m_states[state_id];
    if (state == NULL) {
        state = CreateState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
auto ARAStar::CreateState(int state_id) -> SearchState*
{
    assert(state_id < m_states.size());

    auto* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;

    return ss;
}

// Lazily (re)initialize a search state.
void ARAStar::ReinitSearchState(SearchState* state, bool goal)
{
    if (state->call_number != m_call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        if (goal) {
            state->h = 0;
        } else {
            state->h = m_heur->GetGoalHeuristic(state->state_id);
        }
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

// Extract the path from the start state up to a new state.
void ARAStar::ExtractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (auto* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

} // namespace smpl
