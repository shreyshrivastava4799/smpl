#ifndef SBPL_COLLISION_CHECKING_SHAPE_VISUALIZATION_H
#define SBPL_COLLISION_CHECKING_SHAPE_VISUALIZATION_H

#include <visualization_msgs/Marker.h>

#include <sbpl_collision_checking/shapes.h>

namespace smpl {
namespace collision {

bool MakeCollisionShapeMarker(
    const CollisionShape& shape,
    visualization_msgs::Marker& m);

bool MakeSphereShapeMarker(
    const SphereShape& shape,
    visualization_msgs::Marker& m);

bool MakeCylinderShapeMarker(
    const CylinderShape& shape,
    visualization_msgs::Marker& m);

bool MakeConeShapeMarker(
    const ConeShape& shape,
    visualization_msgs::Marker& m);

bool MakeBoxShapeMarker(
    const BoxShape& shape,
    visualization_msgs::Marker& m);

bool MakePlaneShapeMarker(
    const PlaneShape& shape,
    visualization_msgs::Marker& m);

bool MakeMeshShapeMarker(
    const MeshShape& shape,
    visualization_msgs::Marker& m);

bool MakeOcTreeShapeMarker(
    const OcTreeShape& shape,
    visualization_msgs::Marker& m);

} // namespace collision
} // namespace smpl

#endif
