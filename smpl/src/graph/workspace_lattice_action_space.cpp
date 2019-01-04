#include <smpl/graph/workspace_lattice_action_space.h>

namespace smpl {

WorkspaceLatticeActionSpace::~WorkspaceLatticeActionSpace()
{
}

bool WorkspaceLatticeActionSpace::UpdateHeuristics(Heuristic** heuristics, int count)
{
    return true;
}

bool WorkspaceLatticeActionSpace::UpdateStart(int state_id)
{
    return true;
}

bool WorkspaceLatticeActionSpace::UpdateGoal(GoalConstraint* goal)
{
    return true;
}

} // namespace smpl
