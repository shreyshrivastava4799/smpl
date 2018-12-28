////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
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
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/heuristic/euclid_dist_heuristic.h>

// standard includes
#include <cmath>

// project includes
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <smpl/types.h>
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>

namespace smpl {

static const char* LOG = "heuristic.euclid_dist";

static inline
double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    auto dx = x2 - x1;
    auto dy = y2 - y1;
    auto dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static
double WeightedEuclideanDistance(
    const Vector3& u,
    const Vector3& v,
    const double* weights)
{
    auto sqrd = [](double d) { return d * d; };
    auto diff = Vector3(v - u);
    return weights[0] * sqrd(diff.x()) +
            weights[1] * sqrd(diff.y()) +
            weights[2] * sqrd(diff.z());
}

static
double WeightedEuclideanDistance(
    const Affine3& a,
    const Affine3& b,
    const double* weights)
{
    auto sqrd = [](double d) { return d * d; };

    auto diff = Vector3(b.translation() - a.translation());

    auto dp2 =
            weights[0] * sqrd(diff.x()) +
            weights[1] * sqrd(diff.y()) +
            weights[2] * sqrd(diff.z());

    auto qb = Quaternion(b.rotation());
    auto qa = Quaternion(a.rotation());

    auto dot = qa.dot(qb);
    if (dot < 0.0) {
        qb = Quaternion(-qb.w(), -qb.x(), -qb.y(), -qb.z());
        dot = qa.dot(qb);
    }

    auto dr2 = angles::normalize_angle(2.0 * std::acos(dot));
    dr2 *= (weights[3] * dr2);

    SMPL_DEBUG_NAMED(LOG, "Compute Distance: sqrt(%f + %f)", dp2, dr2);

    return std::sqrt(dp2 + dr2);
}

bool EuclidDistHeuristic::Init(DiscreteSpace* space)
{
    if (!Heuristic::Init(space)) {
        return false;
    }

    m_pose_ext = space->GetExtension<IProjectToPose>();
    if (m_pose_ext == NULL) {
        return false;
    }

    return true;
}

void EuclidDistHeuristic::SetWeightX(double wx)
{
    m_weights[0] = wx;
}

void EuclidDistHeuristic::SetWeightY(double wy)
{
    m_weights[1] = wy;
}

void EuclidDistHeuristic::SetWeightZ(double wz)
{
    m_weights[2] = wz;
}

void EuclidDistHeuristic::SetWeightRot(double wr)
{
    m_weights[3] = wr;
}

auto EuclidDistHeuristic::GetMetricGoalDistance(double x, double y, double z)
    -> double
{
    auto goal_pose = m_goal_pose->GetPose();
    return EuclideanDistance(
            x, y, z,
            goal_pose.translation()[0],
            goal_pose.translation()[1],
            goal_pose.translation()[2]);
}

int EuclidDistHeuristic::GetGoalHeuristic(int state_id)
{
    auto p = m_pose_ext->ProjectToPose(state_id);
    auto goal_pose = m_goal_pose->GetPose();
    auto dist = WeightedEuclideanDistance(p, goal_pose, m_weights);
    auto h = ToFixedPoint(dist);
    SMPL_DEBUG_NAMED(LOG, "h(%d) = %d", state_id, h);
    return h;
}

int EuclidDistHeuristic::GetPairwiseHeuristic(int from_id, int to_id)
{
    auto a = m_pose_ext->ProjectToPose(from_id);
    auto b = m_pose_ext->ProjectToPose(to_id);
    auto dist = WeightedEuclideanDistance(a, b, m_weights);
    auto h = ToFixedPoint(dist);
    SMPL_DEBUG_NAMED(LOG, "h(%d, %d) = %d", from_id, to_id);
    return h;
}

bool EuclidDistHeuristic::UpdateGoal(GoalConstraint* goal)
{
    auto* goal_pose = goal->GetExtension<IGetPose>();
    if (goal_pose == NULL) {
        return false;
    }
    this->m_goal_pose = goal_pose;
    return true;
}

auto EuclidDistHeuristic::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>() ||
        class_code == GetClassCode<IPairwiseHeuristic>() ||
        class_code == GetClassCode<IMetricGoalHeuristic>())
    {
        return this;
    }

    return NULL;
}

bool EuclidDist3DHeuristic::Init(DiscreteSpace* space)
{
    if (!Heuristic::Init(space)) {
        return false;
    }

    m_point_ext = space->GetExtension<IProjectToPoint>();
    if (m_point_ext == NULL) {
        return false;
    }
    return true;
}

void EuclidDist3DHeuristic::SetWeightX(double wx)
{
    m_weights[0] = wx;
}

void EuclidDist3DHeuristic::SetWeightY(double wy)
{
    m_weights[1] = wy;
}

void EuclidDist3DHeuristic::SetWeightZ(double wz)
{
    m_weights[2] = wz;
}

int EuclidDist3DHeuristic::GetGoalHeuristic(int state_id)
{
    auto p = m_point_ext->ProjectToPoint(state_id);
    auto gp = m_goal_position->GetPosition();
    auto dist = WeightedEuclideanDistance(p, gp, m_weights);
    auto h = ToFixedPoint(dist);
    SMPL_DEBUG_NAMED(LOG, "h(%d) = %d", state_id, h);
    return h;
}

int EuclidDist3DHeuristic::GetPairwiseHeuristic(int from_id, int to_id)
{
    auto a = m_point_ext->ProjectToPoint(from_id);
    auto b = m_point_ext->ProjectToPoint(to_id);
    auto dist = WeightedEuclideanDistance(a, b, m_weights);
    auto h = ToFixedPoint(dist);
    SMPL_DEBUG_NAMED(LOG, "h(%d, %d) = %d", from_id, to_id, h);
    return h;
}

bool EuclidDist3DHeuristic::UpdateGoal(GoalConstraint* goal)
{
    auto* goal_position = goal->GetExtension<IGetPosition>();
    if (goal_position == NULL) {
        return false;
    }
    this->m_goal_position = goal_position;
    return true;
}

auto EuclidDist3DHeuristic::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>()) {
        return this;
    }
    return NULL;
}

} // namespace smpl
