////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Benjamin Cohen
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

#ifndef sbpl_collision_types_h
#define sbpl_collision_types_h

// standrad includes
#include <stdio.h>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/world.h>
#include <moveit_msgs/CollisionObject.h>

namespace sbpl {
namespace collision {

struct Sphere
{
    Eigen::Vector3d center;
    double          radius;
};

typedef collision_detection::World::Object Object;
typedef collision_detection::World::ObjectPtr ObjectPtr;
typedef collision_detection::World::ObjectConstPtr ObjectConstPtr;

typedef collision_detection::AllowedCollisionMatrix AllowedCollisionMatrix;

namespace AllowedCollision {
typedef collision_detection::AllowedCollision::Type Type;
} // namespace Allowed Collision

typedef Eigen::aligned_allocator<Eigen::Affine3d> Affine3dAllocator;
typedef std::vector<Eigen::Affine3d, Affine3dAllocator> Affine3dVector;

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

inline
std::string AffineToString(const Eigen::Affine3d& t)
{
    const Eigen::Vector3d pos(t.translation());
    const Eigen::Quaterniond rot(t.rotation());
    const int ENOUGH = 1024;
    char buff[ENOUGH] = { 0 };
    snprintf(buff, ENOUGH, "{ pos = (%0.3f, %0.3f, %0.3f), rot = (%0.3f, %0.3f, %0.3f, %0.3f) }", pos.x(), pos.y(), pos.z(), rot.w(), rot.x(), rot.y(), rot.z());
    return std::string(buff);
}

ObjectConstPtr ConvertCollisionObjectToObject(
    const moveit_msgs::CollisionObject& co);

} // namespace collision
} // namespace sbpl

#endif
