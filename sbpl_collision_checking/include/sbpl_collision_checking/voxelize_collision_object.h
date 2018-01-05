#ifndef SBPL_COLLISION_CHECKING_VOXELIZE_COLLISION_OBJECT_H
#define SBPL_COLLISION_CHECKING_VOXELIZE_COLLISION_OBJECT_H

#include <vector>

#include <Eigen/Dense>

#include <sbpl_collision_checking/shapes.h>

namespace sbpl {
namespace collision {

bool VoxelizeObject(
    const CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeObject(
    const CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeShape(
    const CollisionShape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeShape(
    const CollisionShape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeSphere(
    const SphereShape& sphere,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCylinder(
    const CylinderShape& cylinder,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCone(
    const ConeShape& cone,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeBox(
    const BoxShape& box,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizePlane(
    const PlaneShape& plane,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeMesh(
    const MeshShape& mesh,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeOcTree(
    const OcTreeShape& octree,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

} // namespace collision
} // namespace sbpl

#endif
