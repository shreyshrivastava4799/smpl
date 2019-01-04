#ifndef SMPL_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_WORKSPACE_LATTICE_ACTION_SPACE_H

// standard includes
#include <vector>

// project includes
#include <smpl/graph/workspace_lattice_types.h>

namespace smpl {

class GoalConstraint;
class Heuristic;

class WorkspaceLatticeActionSpace
{
public:

    virtual ~WorkspaceLatticeActionSpace();

    virtual void Apply(
        const WorkspaceLatticeState& state,
        std::vector<WorkspaceAction>& actions) = 0;

    virtual bool UpdateHeuristics(Heuristic** heuristics, int count);
    virtual bool UpdateStart(int state_id);
    virtual bool UpdateGoal(GoalConstraint* goal);
};

} // namespace smpl

#endif

