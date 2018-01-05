#include <sbpl_collision_checking/shape_visualization.h>

namespace sbpl {
namespace collision {

bool MakeCollisionShapeMarker(
    const CollisionShape& shape,
    visualization_msgs::Marker& m)
{
    switch (shape.type) {
    case ShapeType::Sphere:
        return MakeSphereShapeMarker((SphereShape&)shape, m);
    case ShapeType::Cylinder:
        return MakeCylinderShapeMarker((CylinderShape&)shape, m);
    case ShapeType::Cone:
        return MakeConeShapeMarker((ConeShape&)shape, m);
    case ShapeType::Box:
        return MakeBoxShapeMarker((BoxShape&)shape, m);
    case ShapeType::Plane:
        return MakePlaneShapeMarker((PlaneShape&)shape, m);
    case ShapeType::Mesh:
        return MakeMeshShapeMarker((MeshShape&)shape, m);
    case ShapeType::OcTree:
        return MakeOcTreeShapeMarker((OcTreeShape&)shape, m);
    }
}

bool MakeSphereShapeMarker(
    const SphereShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakeCylinderShapeMarker(
    const CylinderShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakeConeShapeMarker(
    const ConeShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakeBoxShapeMarker(
    const BoxShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakePlaneShapeMarker(
    const PlaneShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakeMeshShapeMarker(
    const MeshShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

bool MakeOcTreeShapeMarker(
    const OcTreeShape& shape,
    visualization_msgs::Marker& m)
{
    return false;
}

} // namespace collision
} // namespace sbpl
