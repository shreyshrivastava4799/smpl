#ifndef SMPL_SIMPLE_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_SIMPLE_WORKSPACE_LATTICE_ACTION_SPACE_H

// project includes
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/workspace_lattice_action_space.h>

namespace smpl {

struct WorkspaceLattice;

class SimpleWorkspaceLatticeActionSpace : public WorkspaceLatticeActionSpace
{
public:

    WorkspaceLattice* space = NULL;
    std::vector<MotionPrimitive> m_prims;
    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    void apply(
        const WorkspaceLatticeState& state,
        std::vector<WorkspaceAction>& actions) override;
};

bool InitSimpleWorkspaceLatticeActions(
    WorkspaceLattice* space,
    SimpleWorkspaceLatticeActionSpace* actions);

} // namespace smpl


#endif

