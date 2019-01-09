#ifndef SMPL_WORKSPACE_LATTICE_TYPES_H
#define SMPL_WORKSPACE_LATTICE_TYPES_H

// standard includes
#include <cstddef>
#include <vector>

// project includes
#include <smpl/types.h>

namespace smpl {

enum WorkspaceVariable
{
    FK_PX,
    FK_PY,
    FK_PZ,
    FK_QX,
    FK_QY,
    FK_QZ,
    WS_FA
};

/// continuous state ( x, y, z, R, P, Y, j1, ..., jn )
using WorkspaceState = std::vector<double>;

/// discrete coordinate ( x, y, z, R, P, Y, j1, ..., jn )
using WorkspaceCoord = std::vector<int>;

using WorkspaceAction = std::vector<WorkspaceState>;

struct WorkspaceLatticeState
{
    WorkspaceCoord coord;
    RobotState state;
};

auto operator<<(std::ostream& o, const WorkspaceLatticeState& s) -> std::ostream&;

inline
bool operator==(const WorkspaceLatticeState& a, const WorkspaceLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace smpl

namespace std {

template <>
struct hash<smpl::WorkspaceLatticeState>
{
    using argument_type = smpl::WorkspaceLatticeState;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type;
};

} // namespace std


#endif

