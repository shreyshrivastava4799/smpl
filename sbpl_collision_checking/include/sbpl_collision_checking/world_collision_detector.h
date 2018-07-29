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

#ifndef SBPL_COLLISION_WORLD_COLLISION_DETECTOR_H
#define SBPL_COLLISION_WORLD_COLLISION_DETECTOR_H

// standard includes
#include <vector>

// system includes
#include <smpl/forward.h>

// project includes
#include <sbpl_collision_checking/world_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/attached_bodies_collision_state.h>

namespace smpl {
namespace collision {

SBPL_CLASS_FORWARD(WorldCollisionDetector);

class WorldCollisionDetector
{
public:

    WorldCollisionDetector(
        const RobotCollisionModel* rcm,
        const WorldCollisionModel* wcm);

    bool checkCollision(
        RobotCollisionState& state,
        const int gidx,
        double& dist) const;

    bool checkCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist) const;

    bool checkMotionCollision(
        RobotCollisionState& state,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist) const;

    bool checkMotionCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist) const;

private:

    const RobotCollisionModel* m_rcm;
    const WorldCollisionModel* m_wcm;

    mutable std::vector<const CollisionSphereState*> m_vq;

    bool checkRobotSpheresStateCollisions(
        RobotCollisionState& state,
        int gidx,
        double& dist) const;

    bool checkAttachedBodySpheresStateCollisions(
        AttachedBodiesCollisionState& state,
        int gidx,
        double& dist) const;
};

#endif

} // namespace collision
} // namespace smpl
