////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include <smpl/graph/discrete_space.h>

namespace smpl {

DiscreteSpace::~DiscreteSpace()
{
}

bool DiscreteSpace::UpdateStart(int state_id)
{
    return true;
}

bool DiscreteSpace::UpdateGoal(GoalConstraint* goal)
{
    return true;
}

auto DiscreteSpace::GetRobotModel() -> RobotModel*
{
    return m_robot;
}

auto DiscreteSpace::GetRobotModel() const -> const RobotModel*
{
    return m_robot;
}

auto DiscreteSpace::GetCollisionChecker() -> CollisionChecker*
{
    return m_checker;
}

auto DiscreteSpace::GetCollisionChecker() const -> const CollisionChecker*
{
    return m_checker;
}

bool DiscreteSpace::Init(RobotModel* robot, CollisionChecker* checker)
{
    if (robot == NULL || checker == NULL) {
        return false;
    }

    m_robot = robot;
    m_checker = checker;
    return true;
}

RobotPlanningSpace::~RobotPlanningSpace() { }

IProjectToPoint::~IProjectToPoint() { }

IProjectToPose::~IProjectToPose() { }

auto IProjectToPose::ProjectToPoint(int state_id) -> Vector3
{
    auto pose = ProjectToPose(state_id);
    return Vector3(pose.translation());
}

IExtractRobotState::~IExtractRobotState() { }

ISearchable::~ISearchable() { }

ILazySearchable::~ILazySearchable() { }

} // namespace smpl
