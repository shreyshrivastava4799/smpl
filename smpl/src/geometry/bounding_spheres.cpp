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

#include <smpl/geometry/bounding_spheres.h>

// standard includes
#include <cstdio>
#include <algorithm>

// system includes
#include <Eigen/Core>

// project includes
#include <smpl/geometry/voxelize.h>
#include <smpl/geometry/mesh_utils.h>

namespace sbpl {
namespace geometry {

/// \brief Cover the surface of a box with a set of spheres.
///
/// The box has dimensions length x width x height and is centered at the
/// origin. This function will only append sphere centers to the output vector.
void ComputeBoxBoundingSpheres(
    double length, double width, double height,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedBoxMesh(length, width, height, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a sphere with a set of spheres.
///
/// The sphere is centered at the origin. This function will only append sphere
/// centers to the output vector.
void ComputeSphereBoundingSpheres(
    double cradius,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedSphereMesh(cradius, 7, 8, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a cylinder with a set of spheres.
///
/// The cylinder is centered at the origin with the height along the z-axis.
/// This function will only append sphere centers to the output vector.
void ComputeCylinderBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedCylinderMesh(cradius, cheight, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

/// \brief Cover the surface of a cone with a set of spheres.
///
/// The cone is centered at the origin with the height along the z-axis. This
/// function will only append sphere centers to the output vector.
void ComputeConeBoundingSpheres(
    double cradius, double cheight,
    double radius, std::vector<Eigen::Vector3d>& centers)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> triangles;
    CreateIndexedConeMesh(cradius, cheight, vertices, triangles);
    ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
}

struct EigenVertexArrayIndexer {
    const std::vector<Eigen::Vector3d>& vertices;

    EigenVertexArrayIndexer(const std::vector<Eigen::Vector3d>& vertices) :
        vertices(vertices) { }

    double x(int index) const { return vertices[index].x(); }
    double y(int index) const { return vertices[index].y(); }
    double z(int index) const { return vertices[index].z(); }
};

struct DoubleArrayVertexArrayIndexer {
    const double* arr;

    DoubleArrayVertexArrayIndexer(const double* arr) : arr(arr) { }

    double x(int index) const { return arr[3 * index]; }
    double y(int index) const { return arr[3 * index + 1]; }
    double z(int index) const { return arr[3 * index + 2]; }
};

template <class VertexIndexer, class Callable>
void ComputeMeshBoundingSpheresInternal(
    VertexIndexer indexer,
    const std::uint32_t* indices,
    size_t triangle_count,
    double radius,
    Callable proc)
{
    for (int tidx = 0; tidx < triangle_count; ++tidx) {
        std::uint32_t iv1 = indices[3 * tidx];
        std::uint32_t iv2 = indices[3 * tidx + 1];
        std::uint32_t iv3 = indices[3 * tidx + 2];

        Eigen::Vector3d a(indexer.x(iv1), indexer.y(iv1), indexer.z(iv1));
        Eigen::Vector3d b(indexer.x(iv2), indexer.y(iv2), indexer.z(iv2));
        Eigen::Vector3d c(indexer.x(iv3), indexer.y(iv3), indexer.z(iv3));

        //  compute the pose of the triangle
        const double a2 = (b - c).squaredNorm();
        const double b2 = (a - c).squaredNorm();
        const double c2 = (a - b).squaredNorm();
        double bc1 = a2 * (b2 + c2 - a2);
        double bc2 = b2 * (c2 + a2 - b2);
        double bc3 = c2 * (a2 + b2 - c2);
        const double s = bc1 + bc2 + bc3;
        bc1 /= s;
        bc2 /= s;
        bc3 /= s;

        Eigen::Vector3d p = bc1 * a + bc2 * b + bc3 * c;

        if ((p - a).squaredNorm() <= radius * radius &&
            (p - b).squaredNorm() <= radius * radius &&
            (p - c).squaredNorm() <= radius * radius)
        {
            proc(p, tidx);
            continue;
        }

        Eigen::Vector3d z = (c - b).cross(b - a);

        // normalize or skip z
        auto len = z.norm();
        if (len < 1e-6) {
            continue;
        }
        z /= len;

        Eigen::Vector3d x;
        if (a2 > b2 && a2 > c2) {
            x = b - c;
        }
        else if (b2 > c2) {
            x = a - c;
        }
        else {
            x = a - b;
        }

        // normalize or skip
        len = x.norm();
        if (len < 1e-6) {
            continue;
        }
        x /= len;

        Eigen::Vector3d y = z.cross(x);
        Eigen::Affine3d T_mesh_triangle;
        T_mesh_triangle(0, 0) = x[0];
        T_mesh_triangle(1, 0) = x[1];
        T_mesh_triangle(2, 0) = x[2];
        T_mesh_triangle(3, 0) = 0.0;

        T_mesh_triangle(0, 1) = y[0];
        T_mesh_triangle(1, 1) = y[1];
        T_mesh_triangle(2, 1) = y[2];
        T_mesh_triangle(3, 1) = 0.0;

        T_mesh_triangle(0, 2) = z[0];
        T_mesh_triangle(1, 2) = z[1];
        T_mesh_triangle(2, 2) = z[2];
        T_mesh_triangle(3, 2) = 0.0;

        T_mesh_triangle(0, 3) = p[0];
        T_mesh_triangle(1, 3) = p[1];
        T_mesh_triangle(2, 3) = p[2];
        T_mesh_triangle(3, 3) = 1.0;

        Eigen::Affine3d T_triangle_mesh = T_mesh_triangle.inverse();

        //  transform the triangle vertices into the triangle frame
        Eigen::Vector3d at = T_triangle_mesh * a;
        Eigen::Vector3d bt = T_triangle_mesh * b;
        Eigen::Vector3d ct = T_triangle_mesh * c;

        double minx = std::min(at.x(), std::min(bt.x(), ct.x()));
        double miny = std::min(at.y(), std::min(bt.y(), ct.y()));
        double maxx = std::max(at.x(), std::max(bt.x(), ct.x()));
        double maxy = std::max(at.y(), std::max(bt.y(), ct.y()));

        //  voxelize the triangle
        PivotVoxelGrid vg(
                Eigen::Vector3d(minx, miny, 0.0),
                Eigen::Vector3d(maxx - minx, maxy - miny, 0.0),
                Eigen::Vector3d(radius, radius, radius),
                Eigen::Vector3d::Zero());
        VoxelizeTriangle(at, bt, ct, vg);

        // extract filled voxels and append as sphere centers
        for (int x = 0; x < vg.sizeX(); ++x) {
            for (int y = 0; y < vg.sizeY(); ++y) {
                MemoryCoord mc(x, y, 0);
                if (vg[mc]) {
                    WorldCoord wc = vg.memoryToWorld(mc);
                    proc(T_mesh_triangle * Eigen::Vector3d(wc.x, wc.y, wc.z),
                            tidx);
                }
            }
        }
    }
}

/// \brief Cover the surface of a mesh with a set of spheres.
///
/// This function will only append sphere centers to the output vector.
void ComputeMeshBoundingSpheres(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    double radius,
    std::vector<Eigen::Vector3d>& centers)
{
    ComputeMeshBoundingSpheresInternal(
            EigenVertexArrayIndexer(vertices),
            indices.data(),
            indices.size() / 3,
            radius,
            [&](const Eigen::Vector3d& center, int tidx) {
                centers.push_back(center);
            });
}

void ComputeMeshBoundingSpheres(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::uint32_t>& indices,
    double radius,
    std::vector<Eigen::Vector3d>& centers,
    std::vector<std::uint32_t>& triangle_indices)
{
    ComputeMeshBoundingSpheresInternal(
            EigenVertexArrayIndexer(vertices),
            indices.data(),
            indices.size() / 3, 
            radius,
            [&](const Eigen::Vector3d& center, int tidx) {
                centers.push_back(center);
                triangle_indices.push_back(tidx);
            });
}

void ComputeMeshBoundingSpheres(
    const double* vertex_data,
    size_t vertex_count,
    const std::uint32_t* triangle_data,
    size_t triangle_count,
    double radius,
    std::vector<Eigen::Vector3d>& centers,
    std::vector<std::uint32_t>& triangle_indices)
{
    ComputeMeshBoundingSpheresInternal(
        DoubleArrayVertexArrayIndexer(vertex_data),
        triangle_data,
        triangle_count,
        radius,
        [&](const Eigen::Vector3d& center, int tidx) {
            centers.push_back(center);
            triangle_indices.push_back(tidx);
        });
}

} // namespace geometry
} // namespace sbpl
