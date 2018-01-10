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

#include <smpl/geometry/intersect.h>

// standard includes
#include <cmath>
#include <algorithm>

// project includes
#include <smpl/geometry/triangle.h>

namespace sbpl {
namespace geometry {

static bool PointOnTriangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d bc = c - b;
    Eigen::Vector3d ca = a - c;

    Eigen::Vector3d n = ab.cross(bc);

    Eigen::Vector3d abn = n.cross(ab);
    Eigen::Vector3d bcn = n.cross(bc);
    Eigen::Vector3d can = n.cross(ca);

    Eigen::Vector3d p1 = a - p;
    Eigen::Vector3d p2 = b - p;
    Eigen::Vector3d p3 = c - p;

    if (p1.dot(abn) < 0.0 && p2.dot(bcn) < 0.0 && p3.dot(can) < 0.0) {
        return true;
    }

    if (p1.dot(abn) > 0.0 && p2.dot(bcn) > 0.0 && p3.dot(can) > 0.0) {
        return true;
    }

    return false;
}

static bool CoplanarTrianglesIntersect(
    const Triangle& tr1,
    const Triangle& tr2,
    double eps)
{
    auto& v10 = tr1.a;
    auto& v11 = tr1.b;
    auto& v12 = tr1.c;
    auto& v20 = tr2.a;
    auto& v21 = tr2.b;
    auto& v22 = tr2.c;

    //////////////////////////////////////////////////////////////////
    /// Perform tests to see if coplanar triangles are in collision //
    //////////////////////////////////////////////////////////////////

    // TODO: project onto the axis-aligned plane where the areas of the
    // triangles are maximized

    // Project the coplanar triangles into the X-Y plane
    std::vector<Eigen::Vector2d> firstTri2DVertices;
    Eigen::Vector2d u1;
    u1[0] = v10[0]; u1[1] = v10[1];
    firstTri2DVertices.push_back(u1);
    u1[0] = v11[0]; u1[1] = v11[1];
    firstTri2DVertices.push_back(u1);
    u1[0] = v12[0]; u1[1] = v12[1];
    firstTri2DVertices.push_back(u1);

    std::vector<Eigen::Vector2d> secondTri2DVertices;
    Eigen::Vector2d u2;
    u2[0] = v20[0]; u2[1] = v20[1];
    secondTri2DVertices.push_back(u2);
    u2[0] = v21[0]; u2[1] = v21[1];
    secondTri2DVertices.push_back(u2);
    u2[0] = v22[0]; u2[1] = v22[1];
    secondTri2DVertices.push_back(u2);

    ////////////////////////////////////////////////////////////////////
    /// Perform collision tests between two-dimensional line segments //
    ////////////////////////////////////////////////////////////////////

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // the points defining the first segment
            auto& pt1 = firstTri2DVertices[i];
            auto& pt2 = firstTri2DVertices[(i + 1) % 3];
            // the points defining the second segment
            auto& pt3 = secondTri2DVertices[j];
            auto& pt4 = secondTri2DVertices[(j + 1) % 3];

            double denom = (pt1[0] - pt2[0]) * (pt3[1] - pt4[1]) - (pt1[1] - pt2[1]) * (pt3[0] - pt4[0]);
            double tmp1 = pt1[0] * pt2[1] - pt1[1] * pt2[0];
            double tmp2 = pt3[0] * pt4[1] - pt3[1] * pt4[0];

            // if lines arent parallel
            if (std::fabs(denom) > eps) {
                // the point of intersection between the two lines (from the segments)
                Eigen::Vector2d intersection;
                intersection[0] = tmp1 * (pt3[0] - pt4[0]) - (pt1[0] - pt2[0]) * tmp2;
                intersection[0] /= denom;
                intersection[1] = tmp1 * (pt3[1] - pt4[1]) - (pt1[1] - pt2[1]) * tmp2;
                intersection[1] /= denom;

                // check if the point of between the lines intersection lies on the line segments
                double vx1 = pt2[0] - pt1[0];
                double vx2 = pt4[0] - pt3[0];
                if ((intersection[0] - pt1[0]) / vx1 >= 0 && (intersection[0] - pt1[0]) / vx1 <= 1.0 &&
                    (intersection[0] - pt3[0]) / vx2 >= 0 && (intersection[0] - pt3[0]) / vx2 <= 1.0)
                {
                    return true;
                }
            }
            else
            {
                // ANDREW: bad assumption, they could be the same line
                //std::printf("Line segments are parallel and won't intersect\n");
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Check for containment of one triangle within another
    ////////////////////////////////////////////////////////////////////////////////

    // compute center point of first triangle
    Eigen::Vector2d firstTriangleCenter(0.0, 0.0);
    for (int i = 0; i < 3; i++) {
        firstTriangleCenter[0] += firstTri2DVertices[i][0];
        firstTriangleCenter[1] += firstTri2DVertices[i][1];
    }
    firstTriangleCenter[0] /= 3;
    firstTriangleCenter[1] /= 3;

    // compute center point of second triangle
    Eigen::Vector2d secondTriangleCenter(0.0, 0.0);
    for (int i = 0; i < 3; i++) {
        secondTriangleCenter[0] += secondTri2DVertices[i][0];
        secondTriangleCenter[1] += secondTri2DVertices[i][1];
    }
    secondTriangleCenter[0] /= 3;
    secondTriangleCenter[1] /= 3;

    Eigen::Vector3d firstTriangleCenterTo3D(firstTriangleCenter[0], firstTriangleCenter[1], 0.0);
    Eigen::Vector3d secondTriangleCenterTo3D(secondTriangleCenter[0], secondTriangleCenter[1], 0.0);
    Eigen::Vector3d u10To3D(firstTri2DVertices[0][0], firstTri2DVertices[0][1], 0.0);
    Eigen::Vector3d u11To3D(firstTri2DVertices[1][0], firstTri2DVertices[1][1], 0.0);
    Eigen::Vector3d u12To3D(firstTri2DVertices[2][0], firstTri2DVertices[2][1], 0.0);
    Eigen::Vector3d u20To3D(secondTri2DVertices[0][0], secondTri2DVertices[0][1], 0.0);
    Eigen::Vector3d u21To3D(secondTri2DVertices[1][0], secondTri2DVertices[1][1], 0.0);
    Eigen::Vector3d u22To3D(secondTri2DVertices[2][0], secondTri2DVertices[2][1], 0.0);

    // Awesome code re-use
    if (PointOnTriangle(firstTriangleCenterTo3D, u20To3D, u21To3D, u22To3D) ||
        PointOnTriangle(secondTriangleCenterTo3D, u10To3D, u11To3D, u12To3D))
    {
        return true;
    }

    return false;
}

bool Intersects(const Triangle& tr1, const Triangle& tr2, double eps)
{
    // Vertices 0, 1, and 2 on triangle 1
    Eigen::Vector3d v10(tr1.a.x(), tr1.a.y(), tr1.a.z());
    Eigen::Vector3d v11(tr1.b.x(), tr1.b.y(), tr1.b.z());
    Eigen::Vector3d v12(tr1.c.x(), tr1.c.y(), tr1.c.z());

    // Vertices 0, 1, and 2 on triangle 2
    Eigen::Vector3d v20(tr2.a.x(), tr2.a.y(), tr2.a.z());
    Eigen::Vector3d v21(tr2.b.x(), tr2.b.y(), tr2.b.z());
    Eigen::Vector3d v22(tr2.c.x(), tr2.c.y(), tr2.c.z());

    ////////////////////////////////////////////////////////////////////////////////
    /// Reject if Triangle 1's vertices are all on the same side of Triangle 2
    ////////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 2 lies
    Eigen::Vector3d n2 = (v21 - v20).cross(v22 - v20);
    n2.normalize();
    double d2 = (-n2).dot(v20);

    // The distances from the vertices of triangle 1 to the plane of triangle 2
    double dv10 = n2.dot(v10) + d2;
    double dv11 = n2.dot(v11) + d2;
    double dv12 = n2.dot(v12) + d2;

    // approx. dv10 != 0.0, dv11 != 0.0, dv12 != 0.0
    if (std::fabs(dv10) > eps && std::fabs(dv11) > eps && std::fabs(dv12) > eps) {
        // all points lie to one side of the triangle
        if ((dv10 > 0 && dv11 > 0 && dv12 > 0) || (dv10 < 0 && dv11 < 0 && dv12 < 0)) {
            return false;
        }
    } else if (std::fabs(dv10) < eps && std::fabs(dv11) < eps && std::fabs(dv12) < eps) {
        return CoplanarTrianglesIntersect(tr1, tr2, eps);
    } else {
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Reject if Triangle 2's vertices are all on the same side of Triangle 1
    ////////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 1 lies
    Eigen::Vector3d n1 = (v11 - v10).cross(v12 - v10);
    n1.normalize();
    double d1 = (-n1).dot(v10);

    // The distances from the vertices of triangle 2 to the plane of triangle 1
    double dv20 = n1.dot(v20) + d1;
    double dv21 = n1.dot(v21) + d1;
    double dv22 = n1.dot(v22) + d1;

    // approx. dv20 != 0.0, dv21 != 0.0, dv22 != 0.0
    if (std::fabs(dv20) > eps && std::fabs(dv21) > eps && std::fabs(dv22) > eps) {
        // all points lie to one side of the triangle
        if ((dv20 > 0 && dv21 > 0 && dv22 > 0) || (dv20 < 0 && dv21 < 0 && dv22 < 0)) {
            return false;
        }
    }
    else {
        // ANDREW: should be covered by coplanar case above
        return false;
    }

    // The direction of the line of intersection between the two planes
    Eigen::Vector3d D = n1.cross(n2);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 1 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

    double t1;
    double t2;

    double pv10 = D.dot(v10);
    double pv11 = D.dot(v11);
    double pv12 = D.dot(v12);

    if ((dv10 > 0 && dv11 < 0 && dv12 < 0) ||
        (dv10 < 0 && dv11 > 0 && dv12 > 0))
    {
        // vertex 0 of triangle 1 is on the opposite side
        // vertices 1 and 2 are on the same side
        t1 = pv11 + (pv10 - pv11) * (dv11 / (dv11 - dv10));
        t2 = pv12 + (pv10 - pv12) * (dv12 / (dv12 - dv10));
    } else if (
        (dv11 > 0 && dv10 < 0 && dv12 < 0) ||
        (dv11 < 0 && dv10 > 0 && dv12 > 0))
    {
        // vertex 1 of triangle 1 is on the opposite side
        // vertices 0 and 2 are on the same side
        t1 = pv10 + (pv11 - pv10) * (dv10 / (dv10 - dv11));
        t2 = pv12 + (pv11 - pv12) * (dv12 / (dv12 - dv11));
    } else { //if((dv12 > 0 && dv10 < 0 && dv11 < 0) || (dv12 < 0 && dv10 > 0 && dv11 > 0))
        // vertex 2 of triangle 1 is on the opposite side
        // vertices 0 and 1 are on the same side
        t1 = pv10 + (pv12 - pv10) * (dv10 / (dv10 - dv12));
        t2 = pv11 + (pv12 - pv11) * (dv11 / (dv11 - dv12));
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 2 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

    double t3;
    double t4;

    double pv20 = D.dot(v20);
    double pv21 = D.dot(v21);
    double pv22 = D.dot(v22);

    if ((dv20 > 0 && dv21 < 0 && dv22 < 0) ||
        (dv20 < 0 && dv21 > 0 && dv22 > 0))
    {
        // vertex 0 of Triangle 2 is on the opposite side
        // vertices 1 and 2 are on the same side
        t3 = pv21 + (pv20 - pv21) * (dv21 / (dv21 - dv20));
        t4 = pv22 + (pv20 - pv22) * (dv22 / (dv22 - dv20));
    } else if (
        (dv21 > 0 && dv20 < 0 && dv22 < 0) ||
        (dv21 < 0 && dv20 > 0 && dv22 > 0))
    {
        // Vertex 1 of Triangle 2 is on the opposite side
        // vertices 0 and 2 are on the same side
        t3 = pv20 + (pv21 - pv20) * (dv20 / (dv20 - dv21));
        t4 = pv22 + (pv21 - pv22) * (dv22 / (dv22 - dv21));
    } else { //if((dv22 > 0 && dv20 < 0 && dv21 < 0) || (dv22 < 0 && dv20 > 0 && dv21 > 0))
        // Vertex 2 of triangle 2 is on the opposite side
        // vertices 0 and 1 are on the same side
        t3 = pv20 + (pv22 - pv20) * (dv20 / (dv20 - dv22));
        t4 = pv21 + (pv22 - pv21) * (dv21 / (dv21 - dv22));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Test for intersection of the above intervals
    ////////////////////////////////////////////////////////////////////////////////

    double temp = t1;
    t1 = std::min(t1, t2);
    t2 = std::max(temp, t2);

    temp = t3;
    t3 = std::min(t3, t4);
    t4 = std::max(temp, t4);

    // if intervals overlap, triangles intersect
    bool overlap = false;
    overlap |= (t3 <= t2 && t2 <= t4);
    overlap |= (t3 <= t1 && t1 <= t4);
    overlap |= (t1 <= t3 && t3 <= t2);
    overlap |= (t1 <= t4 && t4 <= t2);

    if (overlap) {
        return true;
    }

    return false;
}

} // namespace geometry
} // namespace sbpl
