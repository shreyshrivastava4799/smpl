#ifndef SMPL_MANIP_LATTICE_TYPES_H
#define SMPL_MANIP_LATTICE_TYPES_H

// standard includes
#include <vector>

// system includes
#include <smpl/types.h>

namespace smpl {

using RobotCoord = std::vector<int>;

struct ManipLatticeState
{
    RobotCoord coord;   // discrete coordinates
    RobotState state;   // real coordinates
};

bool operator==(const ManipLatticeState& a, const ManipLatticeState& b);

struct ManipLatticeAction
{
    int action_id;
    std::vector<RobotState> motion; // TODO: waypoints
};

} // namespace smpl

namespace std {

template <>
struct hash<smpl::ManipLatticeState>
{
    using argument_type = smpl::ManipLatticeState;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type;
};

} // namespace std

#endif
