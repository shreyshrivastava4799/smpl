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

// standard includes
#include <vector>

// project includes
#include <smpl/heap/intrusive_heap.h>
#include <smpl/search/search.h>
#include <smpl/time.h>

namespace smpl {

class DiscreteSpace;
class Heuristic;
class GoalConstraint;
class ILazySearchable;
class IGoalHeuristic;

class LARAStar;

bool Init(LARAStar* search, DiscreteSpace* graph, Heuristic* heur);

auto GetInitialEps(const LARAStar* search) -> double;
void SetInitialEps(LARAStar* search, double eps);

auto GetTargetEps(const LARAStar* search) -> double;
void SetTargetEps(LARAStar* search, double eps);

auto GetDeltaEps(const LARAStar* search) -> double;
void SetDeltaEps(LARAStar* search, double eps);

bool UpdateStart(LARAStar* search, int state_id);
bool UpdateGoal(LARAStar* search, GoalConstraint* goal);

void ForcePlanningFromScratch(LARAStar* search);
void ForcePlanningFromScratchAndFreeMemory(LARAStar* search);

int Replan(
    LARAStar* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost);

auto GetSolutionEps(const LARAStar* search) -> double;

int GetNumExpansions(const LARAStar* search);
int GetNumExpansionsInitialEps(const LARAStar* search);

auto GetElapsedTime(const LARAStar* search) -> double;
auto GetElapsedTimeInitialEps(const LARAStar* search) -> double;

struct LARAState;

struct CandidatePred
{
    LARAState* pred;
    int32_t g;
    bool true_cost;
};

// A state in the Lazy ARA* search tree. In Lazy ARA*, multiple copies of a
// state may exist within the OPEN list, with different parent states. Rather
// than needlessly create duplicates of all state information, this state
// keeps a list of parent states and the evaluation statuses of the edges to
// those parents. The key used to order the OPEN list is the minimum f-value
// of all copies of the state.
struct LARAState : public heap_element
{
    using lazy_list_type = std::vector<CandidatePred>;

    lazy_list_type cands;

    LARAState*  bp;             // current best predecessor
    LARAState*  ebp;            // best predecessor upon expansion

    int         state_id;       // graph state
    int         h;              // heuristic value

    int         g;              // current best cost-to-go
    int         eg;             // cost-to-go at upon expansion

    int         call_number;    // scenario when last reinitialized

    bool        true_cost;
    bool        closed;
};

// An implementation of the Lazy ARA* (Anytime Repairing A*) search algorithm.
//
// This algorithm is an anytime algorithm which runs a series of weighted
// A*-like searches, decreasing the weight applied to the heuristic function
// between each iteration to lower the bound on suboptimality. It returns the
// best solution found within a given timeout.
//
// In between iterations, the search repairs the existing search tree rather
// than starting from scratch.
//
// Each iteration runs a variant of weighted-A* which postpones the evaluation
// of edge costs until they are required by the search. The input successor
// function must return a lower-bound on the true cost of the edge in order to
// guarantee the bound on suboptimality.
class LARAStar : public Search
{
public:

    struct StateCompare
    {
        const LARAStar* search_;
        bool operator()(const LARAState& s1, const LARAState& s2) const;
    };

    using open_list_type = intrusive_heap<LARAState, StateCompare>;

    ILazySearchable*    graph = NULL;
    IGoalHeuristic*     heur = NULL;

    GoalConstraint*     goal = NULL;

    double init_eps = 1.0;
    double target_eps = 1.0;
    double delta_eps = 1.0;

    std::vector<LARAState*> states;

    open_list_type open;

    int last_start_state_id = -1;
    int start_state_id = -1;

    bool new_goal = true;

    LARAState   best_goal;

    int     call_number     = 0;
    double  eps             = 1.0;

    int num_expansions = 0;
    smpl::clock::duration elapsed_time = smpl::clock::duration::zero();

    std::vector<int> succs;
    std::vector<int> costs;
    std::vector<bool> true_costs;

    LARAStar();
    ~LARAStar();

    /// \name Search Interface
    ///@{
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;

    void ForcePlanningFromScratch() final;
    void ForcePlanningFromScratchAndFreeMemory() final;

    int Replan(const TimeoutCondition& timeout, std::vector<int>* solution, int* cost) final;

    int GetNumExpansions() final;
    auto GetElapsedTime() -> double final;
    ///@}
};

} // namespace smpl
