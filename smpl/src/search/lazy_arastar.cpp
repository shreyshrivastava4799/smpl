#include <smpl/search/lazy_arastar.h>

// project includes
#include <smpl/console/console.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/graph/discrete_space.h>

namespace smpl {

static constexpr auto INFINITECOST = 1000000000;

static const char* LOG = "search";

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
    new_state->cands.clear();
    new_state->state_id = state_id;
    new_state->h = INFINITECOST;
    new_state->g = INFINITECOST;
    new_state->bp = NULL;
    new_state->true_cost = false;
    new_state->ebp = NULL;
    new_state->eg = INFINITECOST;
    new_state->call_number = 0;
    new_state->closed = false;
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
        state->closed = false;
    }
}

static
bool IsPredDominated(CandidatePred cand, LARAState* state)
{
    for (auto it = begin(state->cands); it != end(state->cands); ++it) {
        if (it->pred != cand.pred && it->true_cost && it->g <= cand.g) {
            return true;
        }
    }
    return false;
}

static
void ExpandState(LARAStar* search, LARAState* state)
{
    SMPL_DEBUG_NAMED(LOG, "Expand state %d", state->state_id);

    state->closed = true;

    state->ebp = state->bp;
    state->eg = state->g;

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

    for (auto i = 0; i < search->succs.size(); ++i) {
        auto succ_id = search->succs[i];
        auto cost = search->costs[i];
        auto true_cost = search->true_costs[i];

        auto* succ_state = GetState(search, succ_id);
        ReinitState(search, succ_state);

        if (succ_state->closed) {
            continue;
        }

        auto cand = CandidatePred{ state, state->g + cost, true_cost };

        if (IsPredDominated(cand, succ_state)) {
            continue;
        }

        auto& cands = succ_state->cands;
        cands.push_back(cand);

        auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
            return a.g < b.g;
        };

        auto best_it = std::min_element(begin(cands), end(cands), better_cand);
        assert(best_it != end(cands));

        auto g_old = succ_state->g;

        succ_state->bp = best_it->pred;
        succ_state->g = best_it->g;
        succ_state->true_cost = best_it->true_cost;

        // TODO: check this
        if (succ_state->g < g_old) {
            // if the cost to the goal improved
            if (search->goal->IsGoal(succ_id)) {
                search->best_goal = *succ_state;
            }
        }

        // insert/update succ_state in the OPEN list with the
        // values of its best candidate predecessor
        if (!search->open.contains(succ_state)) {
            search->open.push(succ_state);
        } else {
            search->open.update(succ_state);
        }
    }
}

static
void EvaluateState(LARAStar* search, LARAState* s)
{
    assert(!s->true_cost);
    assert(!s->closed);
    assert(!s->cands.empty());
    assert(!search->open.contains(s));

    // get the best candidate
    auto& cands = s->cands;
    auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
        return a.g < b.g;
    };
    auto best_it = std::min_element(begin(cands), end(cands), better_cand);

    assert(!best_it->true_cost);

    SMPL_DEBUG_NAMED(LOG, "Evaluate transitions %d -> %d", s->bp->state_id, s->state_id);

    auto cost = search->graph->GetTrueCost(s->bp->state_id, s->state_id);

    // remove invalid or now-dominated candidate preds
    if (cost < 0) {
        cands.erase(best_it);
    } else {
        best_it->true_cost = true;
        best_it->g = best_it->pred->g + cost;
        if (IsPredDominated(*best_it, s)) {
            cands.erase(best_it);
        }
    }

    best_it = std::min_element(begin(cands), end(cands), better_cand);
    auto g_old = s->g;
    s->bp = best_it->pred;
    s->g = best_it->g;
    s->true_cost = best_it->true_cost;

    if (s->g < g_old) {
        // if the cost to the goal improved
        if (search->goal->IsGoal(s->state_id)) {
            search->best_goal = *s;
        }
    }

    // OPTIMIZATION if this element is the best, remove all elements except this
    // one from the lazy list. Also, we can probably also remove this element
    // and maintain the s's (bp,g,true) as the current best candidate

    if (best_it != end(cands)) {
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
int ComputeFVal(const LARAStar* search, const LARAState* s)
{
    return s->g + (int)(search->eps * (double)s->h);
}

static
void Clear(LARAStar* search)
{
    search->open.clear();

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
        return search->num_expansions >= timeout.max_expansions;
    case TimeoutCondition::TIME:
        return search->elapsed_time >= timeout.max_allowed_time;
    case TimeoutCondition::USER:
        return timeout.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(LOG, "Invalid timer type");
        return true;
    }

    return true;
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
    // TODO: lazily initialize search for new/old start state ids
    Clear(search);
    search->num_expansions = 0;
    search->elapsed_time = smpl::clock::duration::zero();
    search->call_number++;

    auto* start_state = GetState(search, search->start_state_id);
    auto* goal_state = &search->best_goal;
    ReinitState(search, start_state);
    ReinitState(search, goal_state, true);

    start_state->g = 0;
    start_state->true_cost = true;
    search->open.push(start_state);

    auto start_time = smpl::clock::now();

    for (;;) {
        if (search->open.empty()) {
            break;
        }

        if (TimedOut(search, timeout)) {
            break;
        }

        auto* min_state = search->open.min();
        search->open.pop();

        auto fs = ComputeFVal(search, min_state);
        if (goal_state->true_cost && ComputeFVal(search, goal_state) <= fs) {
            goal_state->ebp = goal_state->bp;
            goal_state->eg = goal_state->g;
            ReconstructPath(search, solution, cost);
            return 1;
        }

        // a state may come up for expansion/evaluation twice
        if (min_state->closed) {
            continue;
        }

        if (min_state->true_cost) {
            ExpandState(search, min_state);
            ++search->num_expansions;
        } else {
            EvaluateState(search, min_state);
        }

        search->elapsed_time = smpl::clock::now() - start_time;
    }

    return 0;
}

auto GetSolutionEps(const LARAStar* search) -> double
{
    return 1.0;
}

int GetNumExpansions(const LARAStar* search)
{
    return 0;
}

int GetNumExpansionsInitialEps(const LARAStar* search)
{
    return 0;
}

auto GetElapsedTime(const LARAStar* search) -> double
{
    return 0.0;
}

auto GetElapsedTimeInitialEps(const LARAStar* search) -> double
{
    return 0.0;
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
