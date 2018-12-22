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

#include <smpl/geometry/voxelize.h>

// standard includes
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <utility>

// project includes
#include <smpl/console/console.h>
#include <smpl/geometry/intersect.h>
#include <smpl/geometry/mesh_utils.h>
#include <smpl/geometry/triangle.h>

namespace smpl {
namespace geometry {

//////////////////////////////////
// Static Function Declarations //
//////////////////////////////////

template <typename Discretizer>
static void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    VoxelGrid<Discretizer>& vg,
    bool fill = false);

template <typename Discretizer>
static void VoxelizeMeshAwesome(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    VoxelGrid<Discretizer>& vg);

template <typename Discretizer>
void VoxelizeMeshNaive(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& triangles,
    VoxelGrid<Discretizer>& vg);

template <typename Discretizer>
void ExtractVoxels(
    const VoxelGrid<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& voxels);

static bool IsInDiscreteBoundingBox(
    const MemoryCoord& mc,
    const MemoryCoord& minmc,
    const MemoryCoord& maxmc);

/// \brief Fill the interior of a voxel grid via scanning
template <typename Discretizer>
static void ScanFill(VoxelGrid<Discretizer>& vg);

static void TransformVertices(
    const Eigen::Affine3d& transform,
    std::vector<Eigen::Vector3d>& vertices);

static double Distance(const Eigen::Vector3d& n, double d, const Eigen::Vector3d& x);

double Distance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    double radius_sqrd,
    const Eigen::Vector3d& x);

static bool CompareX(const Eigen::Vector3d& u, const Eigen::Vector3d& v);
static bool CompareY(const Eigen::Vector3d& u, const Eigen::Vector3d& v);
static bool CompareZ(const Eigen::Vector3d& u, const Eigen::Vector3d& v);

/////////////////////////////////
// Static Function Definitions //
/////////////////////////////////

template <typename Discretizer>
static void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    VoxelGrid<Discretizer>& vg,
    bool fill)
{
    const bool awesome = true;
    if (awesome) {
        VoxelizeMeshAwesome(vertices, indices, vg);
    }
    else {
        VoxelizeMeshNaive(vertices, indices, vg);
    }

    if (fill) {
        ScanFill(vg);
    }
}

template <typename Discretizer>
void VoxelizeMeshAwesome(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    VoxelGrid<Discretizer>& vg)
{
    for (size_t i = 0; i < indices.size(); i += 3) {
        auto& a = vertices[indices[i + 0]];
        auto& b = vertices[indices[i + 1]];
        auto& c = vertices[indices[i + 2]];
        VoxelizeTriangle(a, b, c, vg);
    }
}

template <typename Discretizer>
void VoxelizeMeshNaive(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& triangles,
    VoxelGrid<Discretizer>& vg)
{
    // create a triangle mesh for the voxel grid surrounding the mesh
    // TODO: use indexed mesh
    std::vector<Eigen::Vector3d> voxel_mesh;
    CreateGridMesh(vg, voxel_mesh);

    for (auto tidx = 0; tidx < (int)triangles.size(); tidx += 3) {
        // get the vertices of the triangle as Point
        auto& pt1 = vertices[triangles[tidx + 0]];
        auto& pt2 = vertices[triangles[tidx + 1]];
        auto& pt3 = vertices[triangles[tidx + 2]];

        // pack those vertices into my Triangle struct
        Triangle triangle(pt1, pt2, pt3);

        // get the bounding box of the triangle
        std::vector<Eigen::Vector3d> triPointV = { pt1, pt2, pt3 };
        Eigen::Vector3d tri_min;
        Eigen::Vector3d tri_max;
        if (!ComputeAxisAlignedBoundingBox(triPointV, tri_min, tri_max)) {
            SMPL_ERROR("Failed to compute AABB of triangle");
            continue; // just skip this triangle; it's bogus
        }

        // compute the bounding voxel grid
        auto minwc = WorldCoord(tri_min.x(), tri_min.y(), tri_min.z());
        auto maxwc = WorldCoord(tri_max.x(), tri_max.y(), tri_max.z());
        auto minmc = vg.worldToMemory(minwc);
        auto maxmc = vg.worldToMemory(maxwc);

        // voxels in the voxel mesh are ordered by the memory index
        for (auto a = 0; a < (int)voxel_mesh.size() / 3; ++a) {
            auto voxelNum = a / 12; // there are 12 mesh triangles per voxel

            auto mi = MemoryIndex(voxelNum);
            auto mc = vg.indexToMemory(mi);

            // if not already filled, is in the bounding voxel grid of the
            // triangle, and this voxel mesh triangle Intersects the current
            // triangle, fill in the voxel

            auto t = Triangle(voxel_mesh[3 * a], voxel_mesh[3 * a + 1], voxel_mesh[3 * a + 2]);

            if (!vg[mi] &&
                IsInDiscreteBoundingBox(mc, minmc, maxmc) &&
                Intersects(triangle, t))
            {
                vg[mi] = true;
            }
        }
    }
}

template <typename Discretizer>
void ExtractVoxels(
    const VoxelGrid<Discretizer>& vg,
    std::vector<Eigen::Vector3d>& voxels)
{
    for (int x = 0; x < vg.sizeX(); x++) {
    for (int y = 0; y < vg.sizeY(); y++) {
    for (int z = 0; z < vg.sizeZ(); z++) {
        MemoryCoord mc(x, y, z);
        if (vg[mc]) {
            WorldCoord wc = vg.memoryToWorld(mc);
            voxels.push_back(Eigen::Vector3d(wc.x, wc.y, wc.z));
        }
    }
    }
    }
}

bool ComputeAxisAlignedBoundingBox(
    const std::vector<Eigen::Vector3d>& vertices,
    Eigen::Vector3d& min,
    Eigen::Vector3d& max)
{
    if (vertices.empty()) {
        return false;
    }

    min.x() = max.x() = vertices[0].x();
    min.y() = max.y() = vertices[0].y();
    min.z() = max.z() = vertices[0].z();

    for (const Eigen::Vector3d& vertex : vertices) {
        if (vertex.x() < min.x()) {
            min.x() = vertex.x();
        }
        if (vertex.x() > max.x()) {
            max.x() = vertex.x();
        }
        if (vertex.y() < min.y()) {
            min.y() = vertex.y();
        }
        if (vertex.y() > max.y()) {
            max.y() = vertex.y();
        }
        if (vertex.z() < min.z()) {
            min.z() = vertex.z();
        }
        if (vertex.z() > max.z()) {
            max.z() = vertex.z();
        }
    }

    return true;
}

bool IsInDiscreteBoundingBox(
    const MemoryCoord& mc,
    const MemoryCoord& minmc,
    const MemoryCoord& maxmc)
{
    bool inside = true;
    inside &= mc.x >= minmc.x && mc.x <= maxmc.x;
    inside &= mc.y >= minmc.y && mc.y <= maxmc.y;
    inside &= mc.z >= minmc.z && mc.z <= maxmc.z;
    return inside;
}


template <typename Discretizer>
void ScanFill(VoxelGrid<Discretizer>& vg)
{
    for (int x = 0; x < vg.sizeX(); x++) {
        for (int y = 0; y < vg.sizeY(); y++) {
            const int OUTSIDE = 0;
            const int ON_BOUNDARY_FROM_OUTSIDE = 1;
            const int INSIDE = 2;
            const int ON_BOUNDARY_FROM_INSIDE = 4;

            int scan_state = OUTSIDE;

            for (int z = 0; z < vg.sizeZ(); z++) {
                if (scan_state == OUTSIDE && vg[MemoryCoord(x, y, z)]) {
                    scan_state = ON_BOUNDARY_FROM_OUTSIDE;
                }
                else if (scan_state == ON_BOUNDARY_FROM_OUTSIDE &&
                    !vg[MemoryCoord(x, y, z)])
                {
                    bool allEmpty = true;
                    for (int l = z; l < vg.sizeZ(); l++) {
                        allEmpty &= !vg[MemoryCoord(x, y, l)];
                    }
                    if (allEmpty) {
                        scan_state = OUTSIDE;
                    }
                    else {
                        scan_state = INSIDE;
                        vg[MemoryCoord(x, y, z)] = true;
                    }
                }
                else if (scan_state == INSIDE && !vg[MemoryCoord(x, y, z)]) {
                    vg[MemoryCoord(x, y, z)] = true;
                }
                else if (scan_state == INSIDE && vg[MemoryCoord(x, y, z)]) {
                    scan_state = ON_BOUNDARY_FROM_INSIDE;
                }
                else if (scan_state == ON_BOUNDARY_FROM_INSIDE &&
                    !vg[MemoryCoord(x, y, z)])
                {
                    scan_state = OUTSIDE;
                }
            }
        }
    }
}

void TransformVertices(
    const Eigen::Affine3d& transform,
    std::vector<Eigen::Vector3d>& vertices)
{
    for (std::size_t i = 0; i < vertices.size(); i++) {
        vertices[i] = transform * vertices[i];
    }
}

double Distance(
    const Eigen::Vector3d& n, double d,
    const Eigen::Vector3d& x)
{
    return (n.dot(x) + d) / n.norm();
}

/// \brief Compute the distance between point (x, y, z) and the capsule defined
///     by (p1x, p1y, p1z), (p2x, p2y, p2z), and radius_sqrd
double Distance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    double radius_sqrd,
    const Eigen::Vector3d& x)
{
    Eigen::Vector3d pq = q - p;
    Eigen::Vector3d px = x - p;

    double d = px.dot(pq);

    if (d < 0.0 || d > pq.squaredNorm()) {
        return -1.0;
    }
    else {
        double dsq = px.squaredNorm() - (d * d) / pq.squaredNorm();
        if (dsq > radius_sqrd) {
            return -1.0;
        }
        else {
            return dsq;
        }
    }
}

bool CompareX(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.x() < v.x();
}

bool CompareY(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.y() < v.y();
}

bool CompareZ(const Eigen::Vector3d& u, const Eigen::Vector3d& v)
{
    return u.z() < v.z();
}

/////////////////////////////////
// Public Function Definitions //
/////////////////////////////////

/// \brief Voxelize a box at the origin
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a box at a given pose
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

void VoxelizeBox(
    double length,
    double width,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

void VoxelizeBox(
    double length,
    double width,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a sphere at the origin
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeSphere(
    double radius,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a sphere at a given pose
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a sphere at the origin using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeSphere(
    double radius,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a sphere at a given pose using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeSphere(
    double radius,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make lng_count and lat_count lines configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedSphereMesh(radius, 7, 8, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a cylinder at the origin
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCylinder(
    double radius,
    double length,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedCylinderMesh(radius, length, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a cylinder at a given pose
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCylinder(
    double radius,
    double length,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedCylinderMesh(radius, length, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a cylinder at the origin using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCylinder(
    double radius,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedCylinderMesh(radius, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a cylinder at a given pose using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCylinder(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: make rim_count configurable or parameters
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedCylinderMesh(radius, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a cone at the origin
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCone(
    double radius,
    double height,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedConeMesh(radius, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a cone at a given pose
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCone(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedConeMesh(radius, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxels, fill);
}

/// \brief Voxelize a cone at the origin using a specified origin for the voxel
///     grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCone(
    double radius,
    double height,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedConeMesh(radius, height, vertices, triangles);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a cone at a given pose using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeCone(
    double radius,
    double height,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    // TODO: implement
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedConeMesh(radius, height, vertices, triangles);
    TransformVertices(pose, vertices);
    VoxelizeMesh(vertices, triangles, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a mesh at the origin
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    if (((int)indices.size()) % 3 != 0) {
        SMPL_ERROR("Incorrect indexed triangles format");
        return;
    }

    Eigen::Vector3d min;
    Eigen::Vector3d max;
    if (!ComputeAxisAlignedBoundingBox(vertices, min, max)) {
        SMPL_ERROR("Failed to compute AABB of mesh vertices");
        return;
    }

    const Eigen::Vector3d size = max - min;
    HalfResVoxelGrid vg(min, size, Eigen::Vector3d(res, res, res));

    VoxelizeMesh(vertices, indices, vg, fill);
    ExtractVoxels(vg, voxels);
}

/// \brief Voxelize a mesh at a given pose
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& triangles,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> v_copy = vertices;
    TransformVertices(pose, v_copy);
    VoxelizeMesh(v_copy, triangles, res, voxels, fill);
}

/// \brief Voxelize a mesh at the origin using a specified origin for the voxel
///     grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& triangles,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    assert(triangles.size() % 3 == 0);

    Eigen::Vector3d min;
    Eigen::Vector3d max;
    if (!ComputeAxisAlignedBoundingBox(vertices, min, max)) {
        SMPL_ERROR("Failed to compute AABB of mesh vertices");
        return;
    }

    const Eigen::Vector3d size = max - min;
//    PivotVoxelGrid vg(min, size, Eigen::Vector3d(res, res, res), voxel_origin);
    PivotVoxelGrid vg(min, max, Eigen::Vector3d(res, res, res), voxel_origin, 0);

    VoxelizeMesh(vertices, triangles, vg, fill);
    ExtractVoxels(vg, voxels);
}

/// \brief Voxelize a mesh at a given pose using a specified origin for the
///     voxel grid
///
/// Output voxels are appended to the input voxel vector.
void VoxelizeMesh(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels,
    bool fill)
{
    std::vector<Eigen::Vector3d> v_copy = vertices;
    TransformVertices(pose, v_copy);
    VoxelizeMesh(v_copy, indices, res, voxel_origin, voxels, fill);
}

/// \brief Voxelize a plane within a given bounding box
void VoxelizePlane(
    double a, double b, double c, double d,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    double res,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> indices;
    CreateIndexedPlaneMesh(a, b, c, d, min, max, vertices, indices);
    VoxelizeMesh(vertices, indices, res, voxels);
}

/// \brief Voxelize a plane within a given bounding box using a specified voxel
///     grid origin
void VoxelizePlane(
    double a, double b, double c, double d,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    double res,
    const Eigen::Vector3d& voxel_origin,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> indices;
    CreateIndexedPlaneMesh(a, b, c, d, min, max, vertices, indices);
    VoxelizeMesh(vertices, indices, res, voxel_origin, voxels);
}

/// \brief Encloses a list of spheres with a set of voxels of a given size
///
/// Encloses a list of spheres with a set of voxels of a given size. The generated voxels appear in the frame the
/// spheres are described in.
///
/// \param[in] spheres The list of spheres to voxelize
/// \param[in] res The resolution of the voxel cells
/// \param[in] removeDuplicates Whether to remove duplicate voxels
/// \param[out] voxels The vector in which to store the voxels
/// \param[out] volume The combined volume of all the spheres
void VoxelizeSphereList(
    const std::vector<double>& radii,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill)
{
    if (radii.size() != poses.size()) {
        return;
    }

    for (std::size_t i = 0; i < radii.size(); i++) {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::uint32_t> indices;
        CreateIndexedSphereMesh(radii[i], 9, 10, vertices, indices);

        TransformVertices(poses[i], vertices);

        std::vector<Eigen::Vector3d> sphere_voxels;
        VoxelizeMesh(vertices, indices, res, sphere_voxels, fill);

        voxels.insert(voxels.end(), sphere_voxels.begin(), sphere_voxels.end());
    }

    int duplicateIdx = (int)voxels.size();
    for (int i = 0; i < duplicateIdx; i++) {
        for (int j = i + 1; j < duplicateIdx; j++) {
            Eigen::Vector3d dx = voxels[i] = voxels[j];

            // since all voxels are aligned on the same grid, if the distance is
            // greater than half the resolution, it has to be the same voxel
            // (really if distance is just less than the resolution)
            if (dx.squaredNorm() < (res * res) / 4.0) {
                std::swap(voxels[duplicateIdx - 1], voxels[j]);
                duplicateIdx--;
            }
        }
    }

    volume = duplicateIdx * res * res * res;

    if (unique) {
        voxels.resize(duplicateIdx);
    }
}

/// \brief a Quick And Dirty (QAD) enclosure of a list of spheres with a set of
///     voxels of a given size
///
/// Encloses a list of spheres with a set of voxels of a given size. The
/// generated voxels appear in the frame the spheres are described in.
///
/// \param[in] spheres The list of spheres to voxelize
/// \param[in] res The resolution of the voxel cells
/// \param[in] removeDuplicates Whether to remove duplicate voxels
/// \param[out] voxels The vector in which to store the voxels
/// \param[out] volume The combined volume of all the spheres
void VoxelizeSphereListQAD(
    const std::vector<double>& spheres,
    const std::vector<Eigen::Affine3d>& poses,
    double res,
    std::vector<Eigen::Vector3d>& voxels,
    double& volume,
    bool unique,
    bool fill)
{
//    // compute the continuous bounding box of all spheres
//    double minXc = 1000000000.0;
//    double minYc = 1000000000.0;
//    double minZc = 1000000000.0;
//    double maxXc = -1000000000.0;
//    double maxYc = -1000000000.0;
//    double maxZc = -1000000000.0;
//    for (std::size_t i = 0; i < spheres.size(); i++) {
//        const double x = spheres[i][0];
//        const double y = spheres[i][1];
//        const double z = spheres[i][2];
//        const double r = spheres[i][3];
//        if (x - r < minXc) minXc = x - r;
//        if (y - r < minYc) minYc = y - r;
//        if (z - r < minZc) minZc = z - r;
//        if (x + r > maxXc) maxXc = x + r;
//        if (y + r > maxYc) maxYc = y + r;
//        if (z + r > maxZc) maxZc = z + r;
//    }
//
//    // compute discrete grid bounds and dimensions
//    HalfResDiscretizer disc(res);
//    const int minXd = disc.discretize(minXc);
//    const int minYd = disc.discretize(minYc);
//    const int minZd = disc.discretize(minZc);
//    const int maxXd = disc.discretize(maxXc);
//    const int maxYd = disc.discretize(maxYc);
//    const int maxZd = disc.discretize(maxZc);
//
//    const int sx = (maxXd - minXd) + 1;
//    const int sy = (maxYd - minYd) + 1;
//    const int sz = (maxZd - minZd) + 1;
//
//    // create an empty voxel grid
//    std::vector<int> grid(sx * sy * sz, 0);
//    Indexer indexer(sx, sy, sz);
//
//    // for each of sphere
//    for (std::size_t i = 0; i < spheres.size(); i++) {
//        const double x = spheres[i][0];
//        const double y = spheres[i][1];
//        const double z = spheres[i][2];
//        const double r = spheres[i][3];
//        const double r2 = r * r;
//
//        // iterate over grids in discrete bounding box of sphere
//        for (int xd = disc.discretize(x - r);
//            xd <= disc.discretize(x + r); ++xd)
//        {
//            for (int yd = disc.discretize(y - r);
//                yd <= disc.discretize(y + r); ++yd)
//            {
//                for (int zd = disc.discretize(z - r);
//                    zd <= disc.discretize(z + r); ++zd)
//                {
//                    const double dx = disc.continuize(xd) - x;
//                    const double dy = disc.continuize(yd) - y;
//                    const double dz = disc.continuize(zd) - z;
//                    const double d2 = dx * dx + dy * dy + dz * dz;
//                    if (d2 <= r2) {
//                        const int ix = xd - minXd;
//                        const int iy = yd - minYd;
//                        const int iz = zd - minZd;
//                        // count the number of spheres that enclose this cell
//                        // center
//                        ++grid[indexer.to_index(ix, iy, iz)];
//                    }
//                }
//            }
//        }
//    }
//
//    // run through the voxel grid and sum the volume
//    std::vector<double> p(3, 0);
//    volume = 0;
//    const double cellVolume = res * res * res;
//    for (int x = 0; x < sx; x++) {
//        for (int y = 0; y < sy; y++) {
//            for (int z = 0; z < sz; z++) {
//                const int gc = grid[indexer.to_index(x, y, z)];
//                if (gc > 0) {
//                    volume += cellVolume;
//
//                    if (unique) {
//                        p[0] = disc.continuize(minXd + x);
//                        p[1] = disc.continuize(minYd + y);
//                        p[2] = disc.continuize(minZd + z);
//                        voxels.push_back(p);
//                    }
//                    else {
//                        for (int i = 0; i < gc; ++i) {
//                            p[0] = disc.continuize(minXd + x);
//                            p[1] = disc.continuize(minYd + y);
//                            p[2] = disc.continuize(minZd + z);
//                            voxels.push_back(p);
//                        }
//                    }
//                }
//            }
//        }
//    }
}

} // namespace geometry
} // namespace smpl
