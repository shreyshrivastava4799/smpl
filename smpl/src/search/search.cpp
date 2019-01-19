#include <smpl/search/search.h>

namespace smpl {

Search::~Search()
{
}

bool Search::UpdateStart(int state_id)
{
    return true;
}

bool Search::UpdateGoal(GoalConstraint* goal)
{
    return true;
}

} // namespace smpl
