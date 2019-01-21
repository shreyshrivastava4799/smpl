#include <smpl/search/lazy_arastar.h>

// project includes
#include <smpl/console/console.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/discrete_space.h>

namespace smpl {

static constexpr auto INFINITECOST = 1000000000;

static const char* LOG = "search.larastar";
static const char* V_LOG = "search.verbose.larastar";
static const char* E_LOG = "search.expansions.larastar";
static const char* S_LOG = "search.successors.larastar";

////////////////////
// Implementation //
////////////////////

static
auto GetState(LARAStar* search, int state_id) -> LARAState*
{
    if (state_id >= (int)search->states.size()) {
        search->states.resize(state_id + 1, NULL);
    }

    if (search->states[state_id] != NULL) {
        return search->states[state_id];
    }

    auto* new_state = new LARAState;
    new_state->state_id = state_id;
    new_state->call_number = 0;
    search->states[state_id] = new_state;

    return new_state;
}

static
void ReinitState(LARAStar* search, LARAState* state, bool goal = false)
{
    if (state->call_number != search->call_number) {
        state->cands.clear();

        if (goal) {
            state->h = 0;
        } else {
            state->h = search->heur->GetGoalHeuristic(state->state_id);
        }

        state->g = INFINITECOST;
        state->bp = NULL;
        state->true_cost = false;
        state->ebp = NULL;
        state->eg = INFINITECOST;
        state->call_number = search->call_number;
        state->iteration_closed = 0;
        state->incons = false;
    }
}

static
bool IsCandDominated(CandidatePred cand, LARAState* state)
{
    for (auto it = begin(state->cands); it != end(state->cands); ++it) {
        if (it->pred != cand.pred && it->true_cost && it->g <= cand.g) {
            return true;
        }
    }
    return false;
}

static
int ComputeFVal(const LARAStar* search, const LARAState* s)
{
    return s->g + (int)(search->curr_eps * (double)s->h);
}

static
void ExpandState(LARAStar* search, LARAState* state)
{
    SMPL_DEBUG_NAMED(E_LOG, "Expand state %d", state->state_id);

    state->eg = state->g;
    state->ebp = state->bp;

    search->succs.clear();
    search->costs.clear();
    search->true_costs.clear();
    search->graph->GetLazySuccs(
            state->state_id,
            &search->succs,
            &search->costs,
            &search->true_costs);

    assert(search->succs.size() == search->costs.size());
    assert(search->succs.size() == search->true_costs.size());

    SMPL_DEBUG_NAMED(E_LOG, "  %zu successors", search->succs.size());

    for (auto i = 0; i < search->succs.size(); ++i) {
        auto succ_id = search->succs[i];
        auto cost = search->costs[i];
        auto true_cost = search->true_costs[i];
        SMPL_DEBUG_NAMED(S_LOG, "    succ: %d, cost: %d, true: %d", succ_id, cost, (int)true_cost);

        auto* succ_state = GetState(search, succ_id);
        ReinitState(search, succ_state);

        // skip if the cost cannot be improved
        auto g_succ_new = state->eg + cost;
        if (succ_state->eg <= g_succ_new) {
            continue;
        }

        // skip if this candidate cannot improve the cost better than another
        // existing candidate
        auto cand = CandidatePred{ state, g_succ_new, true_cost };
        if (IsCandDominated(cand, succ_state)) {
            SMPL_DEBUG_NAMED(S_LOG, "    State %d is dominated", succ_state->state_id);
            continue;
        }

        auto& cands = succ_state->cands;
        cands.push_back(cand);

        auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
            return a.g < b.g;
        };

        auto best_it = std::min_element(begin(cands), end(cands), better_cand);
        assert(best_it != end(cands));

        auto g_succ_old = succ_state->g;
        auto true_cost_old = succ_state->true_cost;

        // Update the (bp, g, true) of the state to its best candidate
        succ_state->bp = best_it->pred;
        succ_state->g = best_it->g;
        succ_state->true_cost = best_it->true_cost;

        // If the best candidate state has a true cost, we can discard all
        // other candidates
        if (succ_state->true_cost) {
            // remove all other candidates?
            succ_state->cands.clear();
        }

        auto update_goal = succ_state->true_cost & ((succ_state->g < g_succ_old) | !true_cost_old);
        if (update_goal) {
            // if the cost to the goal improved
            if (search->goal->IsGoal(succ_id)) {
                SMPL_DEBUG_NAMED(S_LOG, "    State %d is a goal state", succ_state->state_id);
                search->best_goal = *succ_state;
            }
        }

        if (succ_state->iteration_closed == search->iteration) {
            SMPL_DEBUG_NAMED(S_LOG, "    State %d is closed", succ_state->state_id);
            if (!succ_state->incons) {
                SMPL_DEBUG_NAMED(S_LOG, "    Insert into INCONS");
                search->incons.push_back(succ_state);
            }
        } else {
            // insert/update succ_state in the OPEN list with the
            // values of its best candidate predecessor
            if (search->open.contains(succ_state)) {
                SMPL_DEBUG_NAMED(S_LOG, "    -> Update in OPEN with priority %d", ComputeFVal(search, succ_state));
                search->open.update(succ_state);
            } else {
                SMPL_DEBUG_NAMED(S_LOG, "    -> Insert into OPEN with priority %d", ComputeFVal(search, succ_state));
                search->open.push(succ_state);
            }
        }
    }
}

static
void EvaluateState(LARAStar* search, LARAState* s)
{
    assert(!s->true_cost);
    assert(s->iteration_closed != search->iteration);
    assert(!s->cands.empty());
    assert(!search->open.contains(s));

    SMPL_DEBUG_NAMED(E_LOG, "Evaluate transitions %d -> %d", s->bp->state_id, s->state_id);

    // get the best candidate
    auto& cands = s->cands;
    auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
        return a.g < b.g;
    };
    auto best_it = std::min_element(begin(cands), end(cands), better_cand);

    assert(!best_it->true_cost);

    auto cost = search->graph->GetTrueCost(s->bp->state_id, s->state_id);

    SMPL_DEBUG_NAMED(E_LOG, "  cost = %d", cost);

    // remove invalid or now-dominated candidate preds
    if (cost < 1) {
        cands.erase(best_it);
    } else {
        best_it->true_cost = true;
        best_it->g = best_it->pred->eg + cost;
        SMPL_DEBUG_NAMED(E_LOG, "  Update cost-to-go to %d from parent %d", best_it->g, best_it->pred->state_id);
        if (IsCandDominated(*best_it, s)) {
            SMPL_DEBUG_NAMED(E_LOG, "  State %d is dominated", s->state_id);
            cands.erase(best_it);
        }
    }

    best_it = std::min_element(begin(cands), end(cands), better_cand);
    if (best_it == end(cands)) {
        s->bp = NULL;
        s->g = INFINITECOST;
        s->true_cost = false;
        return;
    }

    auto g_old = s->g;
    auto true_cost_old = s->true_cost;

    s->bp = best_it->pred;
    s->g = best_it->g;
    s->true_cost = best_it->true_cost;

    if (s->true_cost) {
        s->cands.clear();
    }

    auto update_goal = s->true_cost & ((s->g < g_old) | !true_cost_old);
    if (update_goal) {
        // if the cost to the goal improved
        if (search->goal->IsGoal(s->state_id)) {
            SMPL_DEBUG_NAMED(E_LOG, "  State %d is a goal state", s->state_id);
            search->best_goal = *s;
        }
    }

    // OPTIMIZATION if this element is the best, remove all elements except this
    // one from the lazy list. Also, we can probably also remove this element
    // and maintain the s's (bp,g,true) as the current best candidate

    // if (best_it != end(cands))
    {
        SMPL_DEBUG_NAMED(E_LOG, "  Reinsert into OPEN with priority %d", ComputeFVal(search, s));
        search->open.push(s);
    }
}

static
void ReconstructPath(const LARAStar* search, std::vector<int>* path, int* cost)
{
    for (auto* state = &search->best_goal; state != NULL; state = state->ebp) {
        path->push_back(state->state_id);
    }

    std::reverse(begin(*path), end(*path));
    *cost = search->best_goal.g;
}

static
void Clear(LARAStar* search)
{
    search->open.clear();
    search->incons.clear();

    for (auto* state : search->states) {
        delete state;
    }

    search->states.clear();
}

static
bool TimedOut(LARAStar* search, const TimeoutCondition& timeout)
{
    if (!timeout.bounded) return false;

    switch (timeout.type) {
    case TimeoutCondition::EXPANSIONS:
        if (search->iteration == 1) {
            return search->num_expansions >= timeout.max_expansions_init;
        } else {
            return search->num_expansions >= timeout.max_expansions;
        }
    case TimeoutCondition::TIME:
        if (search->iteration == 1) {
            return search->elapsed_time >= timeout.max_allowed_time_init;
        } else {
            return search->elapsed_time >= timeout.max_allowed_time;
        }
    case TimeoutCondition::USER:
        return timeout.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(LOG, "Invalid timer type");
        return true;
    }

    return true;
}

static
void ReorderOpen(LARAStar* search)
{
    search->open.make();
}

///////////////
// Interface //
///////////////

bool Init(LARAStar* search, DiscreteSpace* graph, Heuristic* heur)
{
    if (graph == NULL || heur == NULL) {
        return false;
    }

    auto* searchable = graph->GetExtension<ILazySearchable>();
    if (searchable == NULL) {
        return false;
    }

    auto* goal_heur = heur->GetExtension<IGoalHeuristic>();
    if (goal_heur == NULL) {
        return false;
    }

    search->graph = searchable;
    search->heur = goal_heur;
    search->best_goal.call_number = 0;
    return true;
}

auto GetInitialEps(const LARAStar* search) -> double
{
    return search->init_eps;
}

void SetInitialEps(LARAStar* search, double eps)
{
    search->init_eps = eps;
}

auto GetTargetEps(const LARAStar* search) -> double
{
    return search->target_eps;
}

void SetTargetEps(LARAStar* search, double eps)
{
    search->target_eps = eps;
}

auto GetDeltaEps(const LARAStar* search) -> double
{
    return search->delta_eps;
}

void SetDeltaEps(LARAStar* search, double eps)
{
    search->delta_eps = eps;
}

bool UpdateStart(LARAStar* search, int state_id)
{
    search->start_state_id = state_id;
    return true;
}

bool UpdateGoal(LARAStar* search, GoalConstraint* goal)
{
    search->goal = goal;
    search->new_goal = true;
    return true;
}

void ForcePlanningFromScratch(LARAStar* search)
{
}

void ForcePlanningFromScratchAndFreeMemory(LARAStar* search)
{
}

int Replan(
    LARAStar* search,
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(LOG, "Reset search");
    // TODO: lazily initialize search for new/old start state ids
    Clear(search);
    search->call_number++;
    search->num_expansions = 0;
    search->elapsed_time = smpl::clock::duration::zero();
    search->num_expansions_init = 0;
    search->elapsed_time_init = smpl::clock::duration::zero();
    search->iteration = 0;
    search->curr_eps = search->init_eps;
    search->found_eps = std::numeric_limits<double>::infinity();

    auto* start_state = GetState(search, search->start_state_id);
    auto* goal_state = &search->best_goal;
    ReinitState(search, start_state);
    ReinitState(search, goal_state, true);

    SMPL_DEBUG(LOG, "Initialize search");

    start_state->g = 0;
    start_state->true_cost = true;
    search->open.push(start_state);

    auto start_time = smpl::clock::now();

    auto found = true;
    for (;;) {
        if (search->found_eps <= search->target_eps) {
            SMPL_DEBUG_NAMED(LOG, "Target epsilon reached");
            break; // exit iteration loop
        }

        search->iteration++;

        // begin subsequent iteration
        if (search->iteration > 1) {
            // exit iteration loop
            if (!timeout.improve) break;

            search->curr_eps -= search->delta_eps;
            search->curr_eps = std::max(search->curr_eps, search->target_eps);
            for (auto* s : search->incons) {
                s->incons = false;
                search->open.push(s);
            }
            ReorderOpen(search);
            search->incons.clear();
            SMPL_DEBUG_NAMED(LOG, "Begin new search iteration %d with epsilon = %f", search->iteration, search->curr_eps);
        }

        auto num_expansions_before = search->num_expansions;
        auto elapsed_time_before = search->elapsed_time;

        // execute iteration
        for (;;) {
            if (search->open.empty()) {
                SMPL_DEBUG_NAMED(LOG, "Exhausted OPEN");
                found = false;
                break;
            }

            if (TimedOut(search, timeout)) {
                SMPL_DEBUG_NAMED(LOG, "Timed out");
                found = false;
                break;
            }

            auto* min_state = search->open.min();

            auto f_min = ComputeFVal(search, min_state);
            if (ComputeFVal(search, goal_state) <= f_min) {
                SMPL_DEBUG_NAMED(LOG, "Found path");
                goal_state->ebp = goal_state->bp;
                goal_state->eg = goal_state->g;
                break;
            }

            search->open.pop();

            // a state may come up for expansion/evaluation twice
            if (min_state->iteration_closed == search->iteration) {
                SMPL_DEBUG_NAMED(E_LOG, "Skip closed state %d", min_state->state_id);
                continue;
            }

            if (min_state->true_cost) {
                min_state->iteration_closed = search->iteration;
                ExpandState(search, min_state);
                ++search->num_expansions;
            } else {
                EvaluateState(search, min_state);
            }

            search->elapsed_time = smpl::clock::now() - start_time;

            SMPL_DEBUG_NAMED(E_LOG, "Expansions: %d, Elapsed Time: %f", search->num_expansions, to_seconds(search->elapsed_time));
        }

        SMPL_DEBUG_NAMED(LOG, "Iteration %d: { Expansions: %d, Elapsed: %f }", search->iteration, search->num_expansions - num_expansions_before, to_seconds(search->elapsed_time - elapsed_time_before));

        if (search->iteration == 1) {
            // update statistics for first iteration
            search->num_expansions_init = search->num_expansions;
            search->elapsed_time_init = search->elapsed_time;
        }

        if (!found) {
            SMPL_DEBUG_NAMED(LOG, "Path not found during iteration %d", search->iteration);
            break;
        }

        search->found_eps = search->curr_eps;
    }

    // update statistics

    if (search->found_eps == std::numeric_limits<double>::infinity()) {
        return 0;
    }

    ReconstructPath(search, solution, cost);
    return 1;
}

auto GetSolutionEps(const LARAStar* search) -> double
{
    return search->found_eps;
}

int GetNumExpansions(const LARAStar* search)
{
    return search->num_expansions;
}

int GetNumExpansionsInitialEps(const LARAStar* search)
{
    return search->num_expansions_init;
}

auto GetElapsedTime(const LARAStar* search) -> double
{
    return to_seconds(search->elapsed_time);
}

auto GetElapsedTimeInitialEps(const LARAStar* search) -> double
{
    return to_seconds(search->elapsed_time_init);
}

bool LARAStar::StateCompare::operator()(
    const LARAState& s1,
    const LARAState& s2) const
{
    return ComputeFVal(search_, &s1) < ComputeFVal(search_, &s2);
}

LARAStar::LARAStar() : open(StateCompare{this}) { }

LARAStar::~LARAStar() { }

bool LARAStar::UpdateStart(int state_id)
{
    return ::smpl::UpdateStart(this, state_id);
}

bool LARAStar::UpdateGoal(GoalConstraint* goal)
{
    return ::smpl::UpdateGoal(this, goal);
}

void LARAStar::ForcePlanningFromScratch()
{
    return ::smpl::ForcePlanningFromScratch(this);
}

void LARAStar::ForcePlanningFromScratchAndFreeMemory()
{
    return ::smpl::ForcePlanningFromScratchAndFreeMemory(this);
}

int LARAStar::Replan(
    const TimeoutCondition& timeout,
    std::vector<int>* solution,
    int* cost)
{
    return ::smpl::Replan(this, timeout, solution, cost);
}

int LARAStar::GetNumExpansions()
{
    return ::smpl::GetNumExpansions(this);
}

auto LARAStar::GetElapsedTime() -> double
{
    return ::smpl::GetElapsedTime(this);
}


} // namespace smpl
