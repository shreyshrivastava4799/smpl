#include <sbpl_collision_checking/shape_visualization.h>

#include <smpl/geometry/mesh_utils.h>

namespace smpl {
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
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = shape.radius;
    return true;
}

bool MakeCylinderShapeMarker(
    const CylinderShape& shape,
    visualization_msgs::Marker& m)
{
    m.type = visualization_msgs::Marker::CYLINDER;
    m.scale.x = m.scale.y = 2.0 * shape.radius;
    m.scale.z = shape.height;
    return true;
}

bool MakeConeShapeMarker(
    const ConeShape& shape,
    visualization_msgs::Marker& m)
{
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.scale.x = m.scale.y = m.scale.z = 1.0;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> indices;
    smpl::geometry::CreateIndexedConeMesh(shape.radius, shape.height, vertices, indices);

    std::vector<geometry_msgs::Point> dense_vertices;
    dense_vertices.reserve(indices.size());

    for (int i = 0; i < indices.size(); i += 3) {
        int ii = i;
        int jj = i + 1;
        int kk = i + 2;

        auto& v1 = vertices[ii];
        auto& v2 = vertices[jj];
        auto& v3 = vertices[kk];

        geometry_msgs::Point p;

        p.x = v1.x(); p.y = v1.y(); p.z = v1.z();
        dense_vertices.push_back(p);

        p.x = v2.x(); p.y = v2.y(); p.z = v2.z();
        dense_vertices.push_back(p);

        p.x = v3.x(); p.y = v3.y(); p.z = v3.z();
        dense_vertices.push_back(p);
    }

    m.points = std::move(dense_vertices);

    return true;
}

bool MakeBoxShapeMarker(
    const BoxShape& shape,
    visualization_msgs::Marker& m)
{
    m.type = visualization_msgs::Marker::CUBE;
    m.scale.x = shape.size[0];
    m.scale.y = shape.size[1];
    m.scale.z = shape.size[2];
    return true;
}

bool MakePlaneShapeMarker(
    const PlaneShape& shape,
    visualization_msgs::Marker& m)
{
    assert(0);
    return false;
}

bool MakeMeshShapeMarker(
    const MeshShape& shape,
    visualization_msgs::Marker& m)
{
    assert(0);
    return false;
}

bool MakeOcTreeShapeMarker(
    const OcTreeShape& shape,
    visualization_msgs::Marker& m)
{
    assert(shape.octree != NULL);

    m.type = visualization_msgs::Marker::CUBE_LIST;

    std::vector<geometry_msgs::Point> occupied_points;
    geometry_msgs::Point p;

    auto& tree = *shape.octree;

    double res = tree.getResolution();

    for (auto lit = tree.begin_leafs(); lit != tree.end_leafs(); ++lit) {
        if (tree.isNodeOccupied(*lit)) {
            if (lit.getSize() <= res) {
                p.x = lit.getX();
                p.y = lit.getY();
                p.z = lit.getZ();
                occupied_points.push_back(p);
            } else {
                double ceil_val = ceil(lit.getSize() / res) * res;
                for (double x = lit.getX() - ceil_val; x < lit.getX() + ceil_val; x += res) {
                for (double y = lit.getY() - ceil_val; y < lit.getY() + ceil_val; y += res) {
                for (double z = lit.getZ() - ceil_val; z < lit.getZ() + ceil_val; z += res) {
                    p.x = x; p.y = y; p.z = z;
                    occupied_points.push_back(p);
                }
                }
                }
            }
        }
    }

    m.scale.x = m.scale.y = m.scale.z = res;
    m.points = std::move(occupied_points);

    return true;
}

} // namespace collision
} // namespace smpl
