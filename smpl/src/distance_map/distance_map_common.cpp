////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/distance_map/detail/distance_map_common.h>

namespace smpl {

/// \param[out] neighbors Precomputed array of possibly 27-connected directions.
///     A particular neighbor is indexed via the dirnum(x, y, z, 0) function.
/// \param[out] indices
void CreateNeighborUpdateList(
    std::array<Eigen::Vector3i, 27>& neighbors,
    std::array<int, NEIGHBOR_LIST_SIZE>& indices,
    std::array<std::pair<int, int>, NUM_DIRECTIONS>& ranges)
{
    int i = 0;
    int n = 0;
    for (int edge = 0; edge < 2; ++edge) {
    for (int sx = -1; sx <= 1; ++sx) {
    for (int sy = -1; sy <= 1; ++sy) {
    for (int sz = -1; sz <= 1; ++sz) {
        if (!edge) {
            neighbors[n++] = Eigen::Vector3i(sx, sy, sz);
        }
        int d = dirnum(sx, sy, sz, edge);
        int nfirst = i;
        for (int tx = -1; tx <= 1; ++tx) {
        for (int ty = -1; ty <= 1; ++ty) {
        for (int tz = -1; tz <= 1; ++tz) {
            if (tx == 0 && ty == 0 && tz == 0) {
                continue;
            }
            if (edge) {
                if (
                    !(((tx == -1 && sx == 1) || (tx == 1 && sx == -1)) ||
                    ((ty == -1 && sy == 1) || (ty == 1 && sy == -1)) ||
                    ((tz == -1 && sz == 1) || (tz == 1 && sz == -1))))
                {
                    indices[i++] = dirnum(tx, ty, tz);
                }
            } else {
                if (tx * sx + ty * sy + tz * sz >= 0) {
                    indices[i++] = dirnum(tx, ty, tz);
                }
            }
        }
        }
        }
        int nlast = i;
        ranges[d].first = nfirst;
        ranges[d].second = nlast;
    }
    }
    }
    }
}

} // namespace smpl
