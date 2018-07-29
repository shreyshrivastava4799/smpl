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

#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/heap.h>

#include <smpl/heap/intrusive_heap.h>

class DiscreteSpaceInformation;
class Heuristic;

namespace smpl {

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

class SMHAStar : public SBPLPlanner
{
public:

    SMHAStar();

    bool Init(
        DiscreteSpaceInformation* space,
        Heuristic* anchor,
        Heuristic** heurs,
        int heur_count);

    virtual ~SMHAStar();

    virtual int set_start(int start_stateID);
    virtual int set_goal(int goal_stateID);

    /// \sa SBPLPlanner::replan(double, std::vector<int>*)
    virtual int replan(
        double allowed_time,
        std::vector<int>* solution);

    /// \sa SBPLPlanner::replan(double, std::vector<int>*, int*)
    virtual int replan(
        double allowed_time,
        std::vector<int>* solution,
        int* solcost);

    /// \sa SBPLPlanner::replan(std::vector<int*>, ReplanParams)
    virtual int replan(
        std::vector<int>* solution,
        ReplanParams params);

    /// \sa SBPLPlanner::replan(std::vector<int>*, ReplanParams, int*)
    virtual int replan(
        std::vector<int>* solution,
        ReplanParams params,
        int* cost);

    /// \sa SBPLPlanner::force_planning_from_scratch()
    /// \return 1 on success; 0 otherwise
    virtual int force_planning_from_scratch();

    /// \sa SBPLPlanner::force_planning_from_scratch_and_free_memory()
    /// \return 1 on success; 0 otherwise
    virtual int force_planning_from_scratch_and_free_memory();

    virtual void costs_changed(const StateChangeQuery& changes);

    /// \sa ARAPlanner::costs_changed()
    virtual void costs_changed();

    /// \name Search Parameter Accessors
    ///@{

    /// \sa SBPLPlanner::set_search_mode(bool)
    virtual int     set_search_mode(bool search_until_first_solution);
    /// \sa SBPLPlanner::set_initialsolution_eps(double)
    virtual void    set_initialsolution_eps(double eps);

    /// \sa SBPLPlanner::get_initial_eps()
    virtual double  get_initial_eps();

    ///@}

    /// \name Search Statistics
    ///@{

    /// \sa SBPLPlanner::get_solution_eps() const
    virtual double  get_solution_eps() const;
    /// \sa SBPLPlanner::get_final_epsilon()
    virtual double  get_final_epsilon();

    /// \sa SBPLPlanner::get_final_eps_planning_time
    virtual double  get_final_eps_planning_time();
    /// \sa SBPLPlanner::get_initial_eps_planning_time
    virtual double  get_initial_eps_planning_time();

    /// \sa SBPLPlanner::get_n_expands() const
    virtual int     get_n_expands() const;
    ///\sa SBPLPlanner::get_n_expands_init_solution()
    virtual int     get_n_expands_init_solution();
    /// \sa SBPLPlanner::get_search_states(std::vector<PlannerStates>*)
    virtual void    get_search_stats(std::vector<PlannerStats>* s);

    ///@}

    /// \name Homogeneous accessor methods for search mode and timing parameters
    // @{

    void    set_initial_eps(double eps) { return set_initialsolution_eps(eps); }
    void    set_initial_mha_eps(double eps_mha);
    void    set_final_eps(double eps);
    void    set_dec_eps(double eps);
    void    set_max_expansions(int expansion_count);
    void    set_max_time(double max_time);

    // double get_initial_eps();
    double  get_initial_mha_eps() const;
    double  get_final_eps() const;
    double  get_dec_eps() const;
    int     get_max_expansions() const;
    double  get_max_time() const;

    ///@}

private:

    // Related objects
    Heuristic* m_anchor = NULL;
    Heuristic** m_heurs = NULL;
    int m_heur_count = 0;           ///< number of additional heuristics used

    ReplanParams m_params = ReplanParams(0.0);
    double m_initial_eps_mha = 1.0;
    int m_max_expansions = 0;

    double m_eps = 1.0;           ///< current w_1
    double m_eps_mha = 1.0;       ///< current w_2

    /// suboptimality bound satisfied by the last search
    double m_eps_satisfied = (double)INFINITECOST;

    int m_num_expansions = 0;   ///< current number of expansion
    double m_elapsed = 0.0;       ///< current amount of seconds

    int m_call_number = 0;

    SMHAState* m_start_state = NULL;
    SMHAState* m_goal_state = NULL;

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

    bool check_params(const ReplanParams& params);

    bool time_limit_reached() const;

    int num_heuristics() const { return m_heur_count + 1; }
    SMHAState* get_state(int state_id);
    void init_state(SMHAState* state, size_t mha_state_idx, int state_id);
    void reinit_state(SMHAState* state);
    void reinit_search();
    void clear_open_lists();
    void clear();
    int compute_key(SMHAState* state, int hidx);
    void expand(SMHAState* state, int hidx);
    SMHAState* state_from_open_state(SMHAState::HeapData* open_state);
    int compute_heuristic(int state_id, int hidx);
    int get_minf(OpenList& pq) const;
    void insert_or_update(SMHAState* state, int hidx);

    void extract_path(std::vector<int>* solution_path, int* solcost);

    bool closed_in_anc_search(SMHAState* state) const;
    bool closed_in_add_search(SMHAState* state) const;
    bool closed_in_any_search(SMHAState* state) const;
};

} // namespace smpl

#endif
