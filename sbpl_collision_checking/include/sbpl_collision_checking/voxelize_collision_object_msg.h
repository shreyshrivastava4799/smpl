#ifndef SBPL_COLLISION_CHECKING_VOXELIZE_COLLISION_OBJECT_MSG_H
#define SBPL_COLLISION_CHECKING_VOXELIZE_COLLISION_OBJECT_MSG_H

#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/SolidPrimitive.h>

namespace smpl {
namespace collision {

bool VoxelizeCollisionObject(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeCollisionObject(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeSolidPrimitive(
    const shape_msgs::SolidPrimitive& prim,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeBox(
    const shape_msgs::SolidPrimitive& box,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeSphere(
    const shape_msgs::SolidPrimitive& sphere,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCylinder(
    const shape_msgs::SolidPrimitive& cylinder,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCone(
    const shape_msgs::SolidPrimitive& cone,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeMesh(
    const shape_msgs::Mesh& mesh,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizePlane(
    const shape_msgs::Plane& plane,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels);

} // namespace collision
} // namespace smpl

#endif

