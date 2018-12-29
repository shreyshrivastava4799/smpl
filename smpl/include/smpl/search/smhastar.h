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

namespace smpl {

class DiscreteSpace;
class GoalConstraint;
class Heuristic;
class IGoalHeuristic;
class ISearchable;
struct StateChangeQuery;

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

struct ReplanParams
{
    double initial_eps;
    double final_eps;
    double dec_eps;
    bool return_first_solution;
    double max_time;
    double repair_time;

    ReplanParams(double time)
    {
        max_time = time;
        initial_eps = 5.0;
        final_eps = 1.0;
        dec_eps = 0.2;
        return_first_solution = false;
        repair_time = -1;
    }
};

class SMHAStar
{
public:

    SMHAStar();

    bool Init(
        DiscreteSpace* space,
        Heuristic* anchor,
        Heuristic** heurs,
        int heur_count);

    ~SMHAStar();

    /// \name Search Configuration
    ///@{
    void SetTargetEpsilon(double eps);
    auto GetTargetEpsilon() const -> double;

    auto GetDeltaEpsilon() const -> double;
    void SetDeltaEpsilon(double eps);

    void SetInitialEps(double eps);
    auto GetInitialEps() const -> double;

    int SetSearchMode(bool search_until_first_solution);

    void SetInitialMHAEps(double eps_mha);
    auto GetInitialMHAEps() const -> double;

    void SetMaxExpansions(int expansion_count);
    int GetMaxExpansions() const;

    void SetMaxTime(double max_time);
    auto GetMaxTime() const -> double;
    ///@}

    /// \name Search Statistics
    ///@{
    auto GetSolutionEps() const -> double;

    int GetNumExpansions() const;
    int GetNumExpansionsInitialEps() const;

    auto GetElapsedTime() const -> double;
    auto GetElapsedTimeInitialEps() const -> double;
    ///@}

    /// \name Search Queries
    ///@{
    bool UpdateStart(int state_id);
    bool UpdateGoal(GoalConstraint* goal);

    void UpdateCosts(const StateChangeQuery& changes);
    void UpdateCosts();

    void ForcePlanningFromScratch();
    void ForcePlanningFromScratchAndFreeMemory();

    int Replan(double allowed_time, std::vector<int>* solution);
    int Replan(double allowed_time, std::vector<int>* solution, int* cost);
    int Replan(ReplanParams params, std::vector<int>* solution);
    int Replan(ReplanParams params, std::vector<int>* solution, int* cost);
    ///@}

    ISearchable* m_space = NULL;

    IGoalHeuristic* m_anchor = NULL;
    std::vector<IGoalHeuristic*> m_heurs;
    int m_heur_count = 0;           ///< number of additional heuristics used

    ReplanParams m_params = ReplanParams(0.0);
    double m_initial_eps_mha = 1.0;
    int m_max_expansions = 0;

    double m_eps = 1.0;           ///< current w_1
    double m_eps_mha = 1.0;       ///< current w_2

    /// suboptimality bound satisfied by the last search
    double m_eps_satisfied;

    int m_num_expansions = 0;   ///< current number of expansion
    double m_elapsed = 0.0;       ///< current amount of seconds

    int m_call_number = 0;

    SMHAState* m_start_state = NULL;

    GoalConstraint* m_goal = NULL;
    SMHAState m_best_goal;

    std::vector<SMHAState*> m_search_states;

    struct HeapCompare
    {
        bool operator()(
            const SMHAState::HeapData& s, const SMHAState::HeapData& t) const
        {
            return s.f < t.f;
        }
    };

    using OpenList = intrusive_heap<SMHAState::HeapData, HeapCompare>;
    OpenList* m_open = NULL; ///< sequence of (m_heur_count + 1) open lists
};

} // namespace smpl

#endif
