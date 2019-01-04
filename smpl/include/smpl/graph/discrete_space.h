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

#ifndef SMPL_DISCRETE_SPACE_H
#define SMPL_DISCRETE_SPACE_H

// standard includes
#include <vector>

// project includes
#include <smpl/extension.h>
#include <smpl/spatial.h>
#include <smpl/types.h>

namespace smpl {

class CollisionChecker;
class GoalConstraint;
class RobotModel;
class Heuristic;

class DiscreteSpace : public virtual Extension
{
public:

    virtual ~DiscreteSpace();

    bool Init(RobotModel* robot, CollisionChecker* checker);

    virtual bool UpdateHeuristics(Heuristic** heuristics, int count);
    virtual bool UpdateStart(int state_id);
    virtual bool UpdateGoal(GoalConstraint* goal);

    auto GetRobotModel() -> RobotModel*;
    auto GetRobotModel() const -> const RobotModel*;
    auto GetCollisionChecker() -> CollisionChecker*;
    auto GetCollisionChecker() const -> const CollisionChecker*;

    RobotModel*         m_robot     = NULL;
    CollisionChecker*   m_checker   = NULL;
};

class RobotPlanningSpace : public virtual Extension
{
public:

    virtual ~RobotPlanningSpace();

    // Return the ID of a discrete state which represents a continuous robot
    // state.
    virtual int GetStateID(const RobotState& state) = 0;

    // Extract a path from a sequence of discrete states
    virtual bool ExtractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) = 0;
};

class IProjectToPoint : public virtual Extension
{
public:

    virtual ~IProjectToPoint();

    virtual auto ProjectToPoint(int state_id) -> Vector3 = 0;
};

class IProjectToPose : public IProjectToPoint
{
public:

    virtual ~IProjectToPose();

    auto ProjectToPoint(int state_id) -> Vector3 final;

    virtual auto ProjectToPose(int state_id) -> Affine3 = 0;
};

class IExtractRobotState : public virtual Extension
{
public:

    virtual ~IExtractRobotState();

    virtual auto ExtractState(int state_id) -> const RobotState& = 0;
};

class ISearchable : public virtual Extension
{
public:

    virtual ~ISearchable();

    virtual void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) = 0;
};

class ILazySearchable : public virtual Extension
{
public:

    virtual ~ILazySearchable();

    virtual void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) = 0;

    virtual int GetTrueCost(int state_id, int succ_state_id) = 0;
};

} // namespace smpl

#endif
