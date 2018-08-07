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

#include <sbpl_collision_checking/world_collision_detector.h>

// system includes
#include <ros/console.h>

#include "collision_operations.h"

namespace smpl {
namespace collision {

static const char* WCM_LOGGER = "world_collision";

WorldCollisionDetector::WorldCollisionDetector(
    const RobotCollisionModel* rcm,
    const WorldCollisionModel* wcm)
:
    m_rcm(rcm),
    m_wcm(wcm),
    m_vq()
{
}

bool WorldCollisionDetector::checkCollision(
    RobotCollisionState& state,
    const int gidx,
    double& dist) const
{
    if (gidx < 0 || gidx >= state.model()->groupCount()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "World Collision Check is for non-existent group");
        return false;
    }

    return checkRobotSpheresStateCollisions(state, gidx, dist);
}

bool WorldCollisionDetector::checkCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const int gidx,
    double& dist) const
{
    if (gidx < 0 || gidx >= state.model()->groupCount() ||
        gidx >= ab_state.model()->groupCount())
    {
        ROS_ERROR_NAMED(WCM_LOGGER, "World Collision Check is for non-existent group");
        return false;
    }

    return checkRobotSpheresStateCollisions(state, gidx, dist) &&
            checkAttachedBodySpheresStateCollisions(ab_state, gidx, dist);
}

bool WorldCollisionDetector::checkMotionCollision(
    RobotCollisionState& state,
    const RobotMotionCollisionModel& rmcm,
    const std::vector<double>& start,
    const std::vector<double>& finish,
    const int gidx,
    double& dist) const
{
    const double res = 0.05;
    MotionInterpolation interp(m_rcm);
    rmcm.fillMotionInterpolation(start, finish, res, interp);

    RobotState interm;
    for (int i = 0; i < interp.waypointCount(); ++i) {
        interp.interpolate(i, interm);
        state.setJointVarPositions(interm.data());
        if (!checkCollision(state, gidx, dist)) {
            return false;
        }
    }
    return true;
}

bool WorldCollisionDetector::checkMotionCollision(
    RobotCollisionState& state,
    AttachedBodiesCollisionState& ab_state,
    const RobotMotionCollisionModel& rmcm,
    const std::vector<double>& start,
    const std::vector<double>& finish,
    const int gidx,
    double& dist) const
{
    const double res = 0.05;
    MotionInterpolation interp(m_rcm);
    rmcm.fillMotionInterpolation(start, finish, res, interp);

    RobotState interm;
    for (int i = 0; i < interp.waypointCount(); ++i) {
        interp.interpolate(i, interm);
        state.setJointVarPositions(interm.data());
        if (!checkCollision(state, ab_state, gidx, dist)) {
            return false;
        }
    }
    return true;
}

/// logical const, but not thread-safe, since it makes use of an internal
/// stack to traverse the sphere tree hierarchy.
bool WorldCollisionDetector::checkRobotSpheresStateCollisions(
    RobotCollisionState& state,
    int gidx,
    double& dist) const
{
    // TODO: refactor commonality with self collision model here
    auto& q = m_vq;
    q.clear();

    for (const int ssidx : state.groupSpheresStateIndices(gidx)) {
        const auto& ss = state.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);
    }

    return CheckVoxelsCollisions(state, q, *m_wcm->grid(), m_wcm->padding(), dist);
}

bool WorldCollisionDetector::checkAttachedBodySpheresStateCollisions(
    AttachedBodiesCollisionState& state,
    int gidx,
    double& dist) const
{
    // TODO: see note in checkRobotSpheresStateCollisions()
    auto& q = m_vq;
    q.clear();

    for (const int ssidx : state.groupSpheresStateIndices(gidx)) {
        const auto& ss = state.spheresState(ssidx);
        const CollisionSphereState* s = ss.spheres.root();
        q.push_back(s);
    }

    return CheckVoxelsCollisions(state, q, *m_wcm->grid(), m_wcm->padding(), dist);
}

} // namespace collision
} // namespace smpl
