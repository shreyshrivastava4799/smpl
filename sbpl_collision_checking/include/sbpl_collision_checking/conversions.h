#ifndef SCDL_CONVERSIONS_H
#define SCDL_CONVERSIONS_H

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>

#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>
#include <moveit_msgs/CollisionObject.h>

#include <sbpl_collision_checking/shapes.h>

namespace smpl {
namespace collision {

/// moveit_msgs::CollisionObject -> collision_detection::World::Object
void ConvertCollisionObjectMsgToWorldObject(
    const moveit_msgs::CollisionObject& in,
    collision_detection::World::Object& out);

/// moveit_msgs::CollisionObject -> scdl::CollisionObject
void ConvertCollisionObjectMsgToCollisionObject(
    const moveit_msgs::CollisionObject& in,
    CollisionObject& out);

/// collision_detection::World::Object -> moveit_msgs::CollisionObject
void ConvertWorldObjectToCollisionObjectMsg(
    const collision_detection::World::Object& in,
    moveit_msgs::CollisionObject& out);

/// collision_detection::World::Object -> scdl::CollisionObject
/// This function allocates new shapes and makes deep copies of the data
/// associated with each shape stored in the collision_detection::World::Object,
/// including octomap and mesh data. The data is expected to be freed explicitly
/// by the caller or using the FreeManagedCollisionObject function.
void ConvertWorldObjectToCollisionObject(
    const collision_detection::World::Object& in,
    CollisionObject& out);

void ConvertWorldObjectToCollisionObjectShallow(
    const collision_detection::World::Object& in,
    CollisionObject& out);

/// scdl::CollisionObject -> moveit_msgs::CollisionObject
void ConvertCollisionObjectToCollisionObjectMsg(
    const CollisionObject& in,
    moveit_msgs::CollisionObject& out);

/// scdl::CollisionObject -> collision_detection::World::Object
/// This function makes deep copies of all shapes and associated shape data
/// stored in the CollisionObject, so that those copies can be managed by
/// collision_detection::World::Object.
void ConvertCollisionObjectToWorldObject(
    const CollisionObject& in,
    collision_detection::World::Object& out);

/// Free the shapes and data associated with deep copies of shapes, owned by
/// CollisionObjects converted from moveit_msgs::CollisionObject or deep copies
/// of collision_detection::World::Object.
void FreeManagedCollisionObject(CollisionObject& co);

void FreeManagedCollisionObjectShallow(CollisionObject& co);

} // namespace collision
} // namespace smpl

#endif
