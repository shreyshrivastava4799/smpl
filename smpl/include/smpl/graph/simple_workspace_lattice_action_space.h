#ifndef SMPL_SIMPLE_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_SIMPLE_WORKSPACE_LATTICE_ACTION_SPACE_H

// project includes
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/workspace_lattice_action_space.h>

namespace smpl {

struct WorkspaceLattice;
class IMetricGoalHeuristic;
class IGetPose;

class SimpleWorkspaceLatticeActionSpace : public WorkspaceLatticeActionSpace
{
public:

    WorkspaceLattice* space = NULL;
    std::vector<MotionPrimitive> m_prims;
    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    void Apply(
        const WorkspaceLatticeState& state,
        std::vector<WorkspaceAction>& actions) final;

    bool UpdateHeuristics(Heuristic** heuristic, int count) final;
    bool UpdateGoal(GoalConstraint* goal) final;

public:

    IMetricGoalHeuristic* m_goal_heuristic = NULL;

    IGetPose* m_get_goal_pose = NULL;
};

bool InitSimpleWorkspaceLatticeActions(
    WorkspaceLattice* space,
    SimpleWorkspaceLatticeActionSpace* actions);

} // namespace smpl


#endif

