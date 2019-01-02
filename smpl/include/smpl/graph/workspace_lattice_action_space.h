#ifndef SMPL_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_WORKSPACE_LATTICE_ACTION_SPACE_H

// standard includes
#include <vector>

// project includes
#include <smpl/graph/workspace_lattice_types.h>

namespace smpl {

class WorkspaceLatticeActionSpace
{
public:

    virtual ~WorkspaceLatticeActionSpace() { }

    virtual void Apply(
        const WorkspaceLatticeState& state,
        std::vector<WorkspaceAction>& actions) = 0;
};

} // namespace smpl

#endif

