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

#ifndef SMPL_ARASTAR_H
#define SMPL_ARASTAR_H

// standard includes
#include <functional>
#include <vector>

// project includes
#include <smpl/heap/intrusive_heap.h>
#include <smpl/time.h>
#include <smpl/search/search.h>

namespace smpl {

class DiscreteSpace;
class Heuristic;
class ISearchable;
class IGoalHeuristic;
class GoalConstraint;
class IGoalHeuristic;
struct StateChangeQuery;

/// An implementation of the ARA* (Anytime Repairing A*) search algorithm. This
/// algorithm runs a series of weighted A* searches with decreasing bounds on
/// suboptimality to return the best solution found within a given time bound.
/// The search intelligently reuses its search tree between successive
/// iterations for improved efficiency, rather than starting each new
/// weighted-A* iteration from scratch.
///
/// This class maintains the state of the search procedure between calls to
/// Replan(), allowing the search to resume from where it left off when the
/// scenario (start, goal, and edge costs in the graph) doesn't change between
/// calls. This can be used to dedicate more time to searching in the event the
/// search fails to find a solution within the given time and to allow solutions
/// to be returned quickly and allowing the search to continue improving the
/// solution given more time. To implement this, several assumptions about the
/// implementation of the graph and heuristic are made:
///
/// * The state IDs are constant between calls to Replan(). If the state ID for
///   any state the search has encountered so far (via state expansions or
///   setting the start or goal) changes, the search will be invalid.
///
/// * Changes to the goal state are reflected by changes to the goal state ID.
///   Often, many graph representations that support multiple or underdefined
///   goal states will represent the goal state given to the planner using a
///   single goal state ID. If this is the case, the caller will have to assert
///   whether or not the goal has changed, and force the planner to reinitialize
///   by calls for force_planning_from_scratch (TODO: shouldn't require full
///   reinitialization)
///
/// * The heuristics for any encountered states remain constant, unless the goal
///   state ID has changed.
class ARAStar : public Search
{
public:

    ARAStar();
    ARAStar(ARAStar&&) = default;
    ~ARAStar();

    ARAStar& operator=(ARAStar&&) = default;

    bool Init(DiscreteSpace* space, Heuristic* heuristic);

    /// \name Search Configuration
    ///@{
    void SetAllowPartialSolutions(bool enabled);
    bool AllowPartialSolutions() const;

    void SetAllowedRepairTime(double allowed_time_secs);
    double GetAllowedRepairTime() const;

    void SetTargetEpsilon(double target_eps);
    double GetTargetEpsilon() const;

    void SetDeltaEpsilon(double delta_eps);
    double GetDeltaEpsilon() const;

    void SetInitialEps(double eps);
    double GetInitialEps();

    void SetImproveSolution(bool improve);
    bool ImproveSolution() const;

    void SetBoundExpansions(bool bound);
    bool BoundExpansions() const;

    int SetSearchMode(bool bSearchUntilFirstSolution);
    ///@}

    /// \name Search Statistics
    ///@{
    double GetSolutionEps() const;

    int GetNumExpansions() final;
    int GetNumExpansionsInitialEps();

    double GetElapsedTime() final;
    double GetElapsedTimeInitialEps();
    ///@}

    /// \name Search Queries
    ///@{
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;
    void UpdateCosts(const StateChangeQuery& stateChange);

    void ForcePlanningFromScratch();
    void ForcePlanningFromScratchAndFreeMemory();

    int Replan(double allowed_time_secs, std::vector<int>* solution);
    int Replan(double allowed_time_secs, std::vector<int>* solution, int* cost);
    int Replan(const TimeoutCondition& timeout, std::vector<int>* solution, int* cost);
    ///@}

public:

    struct SearchState : public heap_element
    {
        int state_id;       // corresponding graph state
        int g;              // cost-to-come
        int h;              // estimated cost-to-go
        int f;              // (g + eps * h) at time of insertion into OPEN
        int eg;             // g-value at time of expansion
        short iteration_closed;
        short call_number;
        SearchState* bp;
        bool incons;
    };

    struct SearchStateCompare
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f < s2.f;
        }
    };

    ISearchable* m_space = 0;
    IGoalHeuristic* m_heur = 0;

    TimeoutCondition m_time_params;

    double m_initial_eps = 1.0;
    double m_final_eps = 1.0;
    double m_delta_eps = 1.0;

    bool m_allow_partial_solutions = false;

    std::vector<SearchState*> m_states;

    int m_start_state_id = -1;   // graph state id for the start state
    GoalConstraint* m_goal = NULL;
    int m_goal_state_id = -1;    // graph state id for the goal state

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    intrusive_heap<SearchState, SearchStateCompare> m_open;
    std::vector<SearchState*> m_incons;
    double m_curr_eps = 1.0;
    int m_iteration = 1;

    std::vector<int> m_succs;
    std::vector<int> m_costs;

    int m_call_number = 0;           // for lazy reinitialization of search states
    int m_last_start_state_id = -1;  // for lazy reinitialization of the search tree
    bool m_new_goal = true;          // for updating the search tree when the goal changes
    SearchState m_best_goal;
    double m_last_eps = 1.0;         // for updating the search tree when heuristics change

    int m_expand_count_init = 0;
    clock::duration m_search_time_init;
    int m_expand_count = 0;
    clock::duration m_search_time;

    double m_satisfied_eps;

    bool TimedOut(
        int elapsed_expansions,
        const clock::duration& elapsed_time) const;

    int ImprovePath(
        const clock::time_point& start_time,
        SearchState* goal_state,
        int& elapsed_expansions,
        clock::duration& elapsed_time);

    void Expand(SearchState* s);

    void RecomputeHeuristics();
    void ReorderOpen();
    int ComputeKey(SearchState* s) const;

    auto GetSearchState(int state_id) -> SearchState*;
    auto CreateState(int state_id) -> SearchState*;
    void ReinitSearchState(SearchState* state, bool goal = false);

    void ExtractPath(
        SearchState* to_state,
        std::vector<int>& solution,
        int& cost) const;
};

} // namespace smpl

#endif
