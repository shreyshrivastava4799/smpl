#ifndef SMPL_SEARCH_H
#define SMPL_SEARCH_H

// standard includes
#include <functional>
#include <vector>

// project includes
#include <smpl/time.h>

namespace smpl {

class GoalConstraint;

struct TimeoutCondition
{
    enum Type {
        EXPANSIONS,
        TIME,
        USER
    } type;

    bool bounded;
    bool improve;

    int max_expansions_init;
    int max_expansions;

    clock::duration max_allowed_time_init;
    clock::duration max_allowed_time;

    std::function<bool()> timed_out_fun;
};

class Search
{
public:

    virtual ~Search();

    virtual bool UpdateStart(int state_id);
    virtual bool UpdateGoal(GoalConstraint* goal);

    virtual int Replan(
        const TimeoutCondition& timeout,
        std::vector<int>* solution,
        int* cost) = 0;

    virtual void ForcePlanningFromScratch() = 0;
    virtual void ForcePlanningFromScratchAndFreeMemory() = 0;

    virtual int GetNumExpansions() = 0;
    virtual auto GetElapsedTime() -> double = 0;
};

} // namespace smpl

#endif
