#include <smpl/graph/manip_lattice_types.h>

namespace smpl {

bool operator==(const ManipLatticeState& a, const ManipLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace smpl

auto std::hash<smpl::ManipLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    auto seed = (size_t)0;
    boost::hash_combine(seed, boost::hash_range(begin(s.coord), end(s.coord)));
    return seed;
}
