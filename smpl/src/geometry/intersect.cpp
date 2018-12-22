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
#include <smpl/spatial.h>

namespace smpl {
namespace geometry {

static bool PointOnTriangle(
    const Vector3& p,
    const Vector3& a,
    const Vector3& b,
    const Vector3& c)
{
    Vector3 ab = b - a;
    Vector3 bc = c - b;
    Vector3 ca = a - c;

    Vector3 n = ab.cross(bc);

    Vector3 abn = n.cross(ab);
    Vector3 bcn = n.cross(bc);
    Vector3 can = n.cross(ca);

    Vector3 p1 = a - p;
    Vector3 p2 = b - p;
    Vector3 p3 = c - p;

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
    Vector2 t1_verts_2d[3];
    t1_verts_2d[0] = Vector2(v10.x(), v10.y());
    t1_verts_2d[1] = Vector2(v11.x(), v11.y());
    t1_verts_2d[2] = Vector2(v12.x(), v12.y());

    Vector2 t2_verts_2d[3];
    t2_verts_2d[0] = Vector2(v20.x(), v20.y());
    t2_verts_2d[1] = Vector2(v21.x(), v21.y());
    t2_verts_2d[2] = Vector2(v22.x(), v22.y());

    ////////////////////////////////////////////////////////////////////
    /// Perform collision tests between two-dimensional line segments //
    ////////////////////////////////////////////////////////////////////

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // the points defining the first segment
            auto& pt1 = t1_verts_2d[i];
            auto& pt2 = t1_verts_2d[(i + 1) % 3];
            // the points defining the second segment
            auto& pt3 = t2_verts_2d[j];
            auto& pt4 = t2_verts_2d[(j + 1) % 3];

            double denom = (pt1[0] - pt2[0]) * (pt3[1] - pt4[1]) - (pt1[1] - pt2[1]) * (pt3[0] - pt4[0]);
            double tmp1 = pt1[0] * pt2[1] - pt1[1] * pt2[0];
            double tmp2 = pt3[0] * pt4[1] - pt3[1] * pt4[0];

            // if lines arent parallel
            if (std::fabs(denom) > eps) {
                // the point of intersection between the two lines (from the segments)
                Vector2 intersection;
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
    Vector2 t1_center = t1_verts_2d[0] + t1_verts_2d[1] + t1_verts_2d[2];
    t1_center /= 3.0;

    // compute center point of second triangle
    Vector2 t2_center = t2_verts_2d[0] + t2_verts_2d[1] + t2_verts_2d[2];
    t2_center /= 3.0;

    Vector3 firstTriangleCenterTo3D(t1_center[0], t1_center[1], 0.0);
    Vector3 secondTriangleCenterTo3D(t2_center[0], t2_center[1], 0.0);
    Vector3 u10To3D(t1_verts_2d[0][0], t1_verts_2d[0][1], 0.0);
    Vector3 u11To3D(t1_verts_2d[1][0], t1_verts_2d[1][1], 0.0);
    Vector3 u12To3D(t1_verts_2d[2][0], t1_verts_2d[2][1], 0.0);
    Vector3 u20To3D(t2_verts_2d[0][0], t2_verts_2d[0][1], 0.0);
    Vector3 u21To3D(t2_verts_2d[1][0], t2_verts_2d[1][1], 0.0);
    Vector3 u22To3D(t2_verts_2d[2][0], t2_verts_2d[2][1], 0.0);

    return
        PointOnTriangle(firstTriangleCenterTo3D, u20To3D, u21To3D, u22To3D) ||
        PointOnTriangle(secondTriangleCenterTo3D, u10To3D, u11To3D, u12To3D);
}

bool Intersects(const Triangle& tr1, const Triangle& tr2, double eps)
{
    // Vertices 0, 1, and 2 on triangle 1
    auto& v10(tr1.a); auto& v11(tr1.b); auto& v12(tr1.c);
    auto& v20(tr2.a); auto& v21(tr2.b); auto& v22(tr2.c);

    ////////////////////////////////////////////////////////////////////////////
    // Reject if Triangle 1's vertices are all on the same side of Triangle 2 //
    ////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 2 lies
    Vector3 n2 = (v21 - v20).cross(v22 - v20);
    n2.normalize();
    double d2 = (-n2).dot(v20);

    // The distances from the vertices of triangle 1 to the plane of triangle 2
    double dv10 = n2.dot(v10) + d2;
    double dv11 = n2.dot(v11) + d2;
    double dv12 = n2.dot(v12) + d2;

    // approx. dv10 != 0.0, dv11 != 0.0, dv12 != 0.0
    if (std::fabs(dv10) > eps && std::fabs(dv11) > eps && std::fabs(dv12) > eps) {
        // all points lie to one side of the triangle
        if ((dv10 > 0.0 && dv11 > 0.0 && dv12 > 0.0) ||
            (dv10 < 0.0 && dv11 < 0.0 && dv12 < 0.0))
        {
            return false;
        }
    } else if (std::fabs(dv10) < eps && std::fabs(dv11) < eps && std::fabs(dv12) < eps) {
        return CoplanarTrianglesIntersect(tr1, tr2, eps);
    } else {
        return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Reject if Triangle 2's vertices are all on the same side of Triangle 1 //
    ////////////////////////////////////////////////////////////////////////////

    // Calculate parameters for the plane in which triangle 1 lies
    Vector3 n1 = (v11 - v10).cross(v12 - v10);
    n1.normalize();
    double d1 = (-n1).dot(v10);

    // The distances from the vertices of triangle 2 to the plane of triangle 1
    double dv20 = n1.dot(v20) + d1;
    double dv21 = n1.dot(v21) + d1;
    double dv22 = n1.dot(v22) + d1;

    // approx. dv20 != 0.0, dv21 != 0.0, dv22 != 0.0
    if (std::fabs(dv20) > eps && std::fabs(dv21) > eps && std::fabs(dv22) > eps) {
        // all points lie to one side of the triangle
        if ((dv20 > 0.0 && dv21 > 0.0 && dv22 > 0.0) ||
            (dv20 < 0.0 && dv21 < 0.0 && dv22 < 0.0))
        {
            return false;
        }
    }
    else {
        // ANDREW: should be covered by coplanar case above
        return false;
    }

    // The direction of the line of intersection between the two planes
    Vector3 D = n1.cross(n2);

#define USE_MAX_COMP 1
#if USE_MAX_COMP
    int max_comp;
    {
        max_comp = 0;
        double d_max_comp = std::fabs(D.x());
        const double ymag = std::fabs(D.y());
        const double zmag = std::fabs(D.z());
        if (ymag > d_max_comp) {
            max_comp = 1;
            d_max_comp = ymag;
        }
        if (zmag > d_max_comp) {
            max_comp = 2;
            d_max_comp = zmag;
        }
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 1 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

#if USE_MAX_COMP
    double pv10 = v10[max_comp];
    double pv11 = v11[max_comp];
    double pv12 = v12[max_comp];
#else
    double pv10 = D.dot(v10);
    double pv11 = D.dot(v11);
    double pv12 = D.dot(v12);
#endif

    double t1;
    double t2;

    if ((dv10 > 0.0 && dv11 < 0.0 && dv12 < 0.0) ||
        (dv10 < 0.0 && dv11 > 0.0 && dv12 > 0.0))
    {
        // vertex 0 of triangle 1 is on the opposite side
        // vertices 1 and 2 are on the same side
        t1 = pv11 + (pv10 - pv11) * (dv11 / (dv11 - dv10));
        t2 = pv12 + (pv10 - pv12) * (dv12 / (dv12 - dv10));
    } else if (
        (dv11 > 0.0 && dv10 < 0.0 && dv12 < 0.0) ||
        (dv11 < 0.0 && dv10 > 0.0 && dv12 > 0.0))
    {
        // vertex 1 of triangle 1 is on the opposite side
        // vertices 0 and 2 are on the same side
        t1 = pv10 + (pv11 - pv10) * (dv10 / (dv10 - dv11));
        t2 = pv12 + (pv11 - pv12) * (dv12 / (dv12 - dv11));
    } else {
        // vertex 2 of triangle 1 is on the opposite side
        // vertices 0 and 1 are on the same side
        t1 = pv10 + (pv12 - pv10) * (dv10 / (dv10 - dv12));
        t2 = pv11 + (pv12 - pv11) * (dv11 / (dv11 - dv12));
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Get the interval on the line where Triangle 2 Intersects
    ////////////////////////////////////////////////////////////////////////////////////////

#if USE_MAX_COMP
    double pv20 = v20[max_comp];
    double pv21 = v21[max_comp];
    double pv22 = v22[max_comp];
#else
    double pv20 = D.dot(v20);
    double pv21 = D.dot(v21);
    double pv22 = D.dot(v22);
#endif
#undef USE_MAX_COMP

    double t3;
    double t4;
    if ((dv20 > 0.0 && dv21 < 0.0 && dv22 < 0.0) ||
        (dv20 < 0.0 && dv21 > 0.0 && dv22 > 0.0))
    {
        // vertex 0 of Triangle 2 is on the opposite side
        // vertices 1 and 2 are on the same side
        t3 = pv21 + (pv20 - pv21) * (dv21 / (dv21 - dv20));
        t4 = pv22 + (pv20 - pv22) * (dv22 / (dv22 - dv20));
    } else if (
        (dv21 > 0.0 && dv20 < 0.0 && dv22 < 0.0) ||
        (dv21 < 0.0 && dv20 > 0.0 && dv22 > 0.0))
    {
        // Vertex 1 of Triangle 2 is on the opposite side
        // vertices 0 and 2 are on the same side
        t3 = pv20 + (pv21 - pv20) * (dv20 / (dv20 - dv21));
        t4 = pv22 + (pv21 - pv22) * (dv22 / (dv22 - dv21));
    } else {
        // Vertex 2 of triangle 2 is on the opposite side
        // vertices 0 and 1 are on the same side
        t3 = pv20 + (pv22 - pv20) * (dv20 / (dv20 - dv22));
        t4 = pv21 + (pv22 - pv21) * (dv21 / (dv21 - dv22));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Test for intersection of the above intervals
    ////////////////////////////////////////////////////////////////////////////////

    if (t1 > t2) {
        std::swap(t1, t2);
    }

    if (t3 > t4) {
        std::swap(t3, t4);
    }

    // if intervals overlap, triangles intersect
    return t2 >= t3 && t1 <= t4;
}

} // namespace geometry
} // namespace smpl
