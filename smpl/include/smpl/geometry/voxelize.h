//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2014, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
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
//////////////////////////////////////////////////////////////////////////////

#ifndef SMPL_VOXELIZE_H
#define SMPL_VOXELIZE_H

// standard includes
#include <vector>

// project includes
#include <smpl/geometry/voxel_grid.h>
#include <smpl/spatial.h>

namespace smpl {
namespace geometry {

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Affine3& pose,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Affine3& pose,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    const Affine3& pose,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeSphere(
    double radius,
    const Affine3& pose,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    const Affine3& pose,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCylinder(
    double radius,
    double height,
    const Affine3& pose,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    const Affine3& pose,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeCone(
    double radius,
    double height,
    const Affine3& pose,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Vector3>& vertices,
    const std::vector<std::uint32_t>& indices,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Vector3>& vertices,
    const std::vector<std::uint32_t>& indices,
    const Affine3& pose,
    double res,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Vector3>& vertices,
    const std::vector<std::uint32_t>& indices,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizeMesh(
    const std::vector<Vector3>& vertices,
    const std::vector<std::uint32_t>& indices,
    const Affine3& pose,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels,
    bool fill = false);

void VoxelizePlane(
    double a, double b, double c, double d,
    const Vector3& min,
    const Vector3& max,
    double res,
    std::vector<Vector3>& voxels);

void VoxelizePlane(
    double a, double b, double c, double d,
    const Vector3& min,
    const Vector3& max,
    double res,
    const Vector3& voxel_origin,
    std::vector<Vector3>& voxels);

void VoxelizeSphereList(
    const std::vector<double>& radii,
    const std::vector<Affine3>& poses,
    double res,
    std::vector<Vector3>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

void VoxelizeSphereListQAD(
    const std::vector<double>& radii,
    const std::vector<Affine3>& poses,
    double res,
    std::vector<Vector3>& voxels,
    double& volume,
    bool unique,
    bool fill = false);

bool ComputeAxisAlignedBoundingBox(
    const std::vector<Vector3>& vertices,
    Vector3& min,
    Vector3& max);

double Distance(
    const Vector3& p,
    const Vector3& q,
    double radius_sqrd,
    const Vector3& x);

template <typename Discretizer>
void VoxelizeTriangle(
    const Vector3& a,
    const Vector3& b,
    const Vector3& c,
    VoxelGrid<Discretizer>& vg);

} // namespace geometry
} // namespace smpl

#include "detail/voxelize.hpp"

#endif
