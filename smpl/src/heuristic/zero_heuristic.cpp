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

#include <smpl/heuristic/zero_heuristic.h>

namespace smpl {

bool ZeroHeuristic::Init(DiscreteSpace* space)
{
    return Heuristic::Init(space);
}

int ZeroHeuristic::GetGoalHeuristic(int state_id)
{
    return 0;
}

int ZeroHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int ZeroHeuristic::GetPairwiseHeuristic(int from_id, int to_id)
{
    return 0;
}

auto ZeroHeuristic::GetMetricGoalDistance(double x, double y, double z) -> double
{
    return 0.0;
}

auto ZeroHeuristic::GetMetricStartDistance(double x, double y, double z) -> double
{
    return 0.0;
}

auto ZeroHeuristic::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<Heuristic>() ||
        class_id == GetClassCode<IGoalHeuristic>() ||
        class_id == GetClassCode<IStartHeuristic>() ||
        class_id == GetClassCode<IPairwiseHeuristic>() ||
        class_id == GetClassCode<IMetricGoalHeuristic>() ||
        class_id == GetClassCode<IMetricStartHeuristic>())
    {
        return this;
    }

    return NULL;
}

} // namespace smpl

