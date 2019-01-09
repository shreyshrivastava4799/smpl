////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/post_processing.h>

// standard includes
#include <chrono>
#include <numeric>

// project includes
#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/geometry/shortcut.h>
#include <smpl/robot_model.h>
#include <smpl/spatial.h>
#include <smpl/time.h>

namespace smpl {

double distance(
    const RobotModel& robot,
    const RobotState& from,
    const RobotState& to)
{
    auto dist = 0.0;
    for (auto vidx = 0; vidx < robot.getPlanningJoints().size(); ++vidx) {
        if (!robot.hasPosLimit(vidx)) {
            dist += shortest_angle_dist(to[vidx], from[vidx]);
        } else {
            dist += fabs(to[vidx] - from[vidx]);
        }
    }
    return dist;
}

double pv_distance(
    const RobotModel& robot,
    const RobotState& from,
    const RobotState& to)
{
    auto sqrd = [](double d) { return d * d; };
    auto pweight = 1.0;
    auto vweight = 1.0;

    auto var_count = (int)robot.getPlanningJoints().size();

    auto dist = 0.0;
    for (auto vidx = 0; vidx < var_count; ++vidx) {
        if (!robot.hasPosLimit(vidx)) {
            dist += shortest_angle_dist(to[vidx], from[vidx]);
        }
        else {
            dist += fabs(to[vidx] - from[vidx]);
        }
    }

    auto vdist = 0.0;
    for (auto vidx = 0; vidx < var_count; ++vidx) {
        vdist += fabs(to[var_count + vidx] - from[var_count + vidx]);
    }

    return pweight * dist + vweight * vdist;
}

class JointPositionShortcutPathGenerator
{
public:

    JointPositionShortcutPathGenerator(RobotModel* rm, CollisionChecker* cc) :
        m_robot(rm),
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& finish,
        OutputIt ofirst, double& cost) const
    {
        auto var_count = m_robot->getPlanningJoints().size();
        if (m_cc->isStateToStateValid(start, finish)) {
            *ofirst++ = start;
            *ofirst++ = finish;
            cost = distance(*m_robot, start, finish);
            return true;
        } else {
            return false;
        }
    }

private:

    RobotModel* m_robot;
    CollisionChecker* m_cc;
};

class JointPositionVelocityShortcutPathGenerator
{
public:

    JointPositionVelocityShortcutPathGenerator(
        RobotModel* rm,
        CollisionChecker* cc)
    :
        m_robot(rm),
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& finish,
        OutputIt ofirst, double& cost) const
    {
        auto var_count = m_robot->getPlanningJoints().size();
        auto pstart = RobotState(start.begin(), start.begin() + var_count);
        auto pend = RobotState(finish.begin(), finish.begin() + var_count);
        if (m_cc->isStateToStateValid(pstart, pend)) {
            *ofirst++ = start;
            *ofirst++ = finish;
            cost = pv_distance(*m_robot, start, finish);
            return true;
        }
        else {
            return false;
        }
    }

private:

    RobotModel* m_robot;
    CollisionChecker* m_cc;
};

class EuclidShortcutPathGenerator
{
public:

    EuclidShortcutPathGenerator(RobotModel* rm, CollisionChecker* cc) :
        m_rm(rm),
        m_cc(cc),
        m_fk_iface(NULL),
        m_ik_iface(NULL)
    {
        m_fk_iface = rm->GetExtension<IForwardKinematics>();
        m_ik_iface = rm->GetExtension<IInverseKinematics>();
    }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& end,
        OutputIt ofirst, double& cost) const
    {
        if (m_fk_iface == NULL || m_ik_iface == NULL) {
            return false;
        }

        // compute forward kinematics for the start an end configurations
        auto from_pose = m_fk_iface->computeFK(start);
        auto to_pose = m_fk_iface->computeFK(end);

        auto pstart = Vector3(from_pose.translation());
        auto pend = Vector3(to_pose.translation());

        auto qstart = Quaternion(from_pose.rotation());
        auto qend = Quaternion(to_pose.rotation());

        // compute the number of path waypoints
        auto posdiff = (pend - pstart).norm();
        auto rotdiff = GetAngularDistance(qstart, qend);

        auto interp_pres = 0.01;
        auto interp_rres = to_radians(5.0);

        auto num_points = 2;
        num_points = std::max(num_points, (int)ceil(posdiff / interp_pres));
        num_points = std::max(num_points, (int)ceil(rotdiff / interp_rres));

        auto cpath = std::vector<RobotState>();

        cpath.push_back(start);
        auto dist = 0.0;
        for (auto i = 1; i < num_points; ++i) {
            // compute the intermediate pose
            auto alpha = (double)i / (double)(num_points - 1);
            auto ppos = Vector3((1.0 - alpha) * pstart + alpha * pend);
            auto prot = Quaternion(qstart.slerp(alpha, qend));

            auto ptrans = Affine3(Translation3(ppos) * prot);

            // run inverse kinematics with the previous pose as the seed state
            auto& prev_wp = cpath.back();
            auto wp = RobotState(m_rm->getPlanningJoints().size(), 0.0);
            if (!m_ik_iface->computeIK(ptrans, prev_wp, wp)) {
                return false;
            }

            // check the path segment for collisions
            if (!m_cc->isStateToStateValid(prev_wp, wp)) {
                return false;
            }

            dist += distance(*m_rm, prev_wp, wp);

            cpath.push_back(wp);
        }

        for (auto& point : cpath) {
            *ofirst++ = std::move(point);
        }
        cost = dist;
        return true;
    }

private:

    RobotModel* m_rm;
    CollisionChecker* m_cc;

    IForwardKinematics* m_fk_iface;
    IInverseKinematics* m_ik_iface;
};

void ShortcutPath(
    RobotModel* rm,
    CollisionChecker* cc,
    const std::vector<RobotState>& pin,
    std::vector<RobotState>& pout,
    ShortcutType type)
{
    if (pin.size() < 2) {
        pout = pin;
        return;
    }

    auto then = clock::now();
    auto prev_cost = 0.0;
    auto next_cost = 0.0;
    switch (type) {
    case ShortcutType::JOINT_SPACE:
    {
        auto costs = std::vector<double>();
        ComputePositionPathCosts(rm, pin, costs);
        JointPositionShortcutPathGenerator generators[] =
        {
            JointPositionShortcutPathGenerator(rm, cc)
        };
        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
    }   break;
    case ShortcutType::EUCLID_SPACE:
    {
        auto costs = std::vector<double>();
        ComputePositionPathCosts(rm, pin, costs);

        EuclidShortcutPathGenerator generators[] =
        {
            EuclidShortcutPathGenerator(rm, cc)
        };

        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
    }   break;
    case ShortcutType::JOINT_POSITION_VELOCITY_SPACE:
    {
        auto pv_path = std::vector<RobotState>();
        CreatePositionVelocityPath(rm, pin, pv_path);

        auto costs = std::vector<double>();
        ComputePositionVelocityPathCosts(rm, pv_path, costs);
        prev_cost = std::accumulate(costs.begin(), costs.end(), 0.0);

        JointPositionVelocityShortcutPathGenerator generators[] =
        {
            JointPositionVelocityShortcutPathGenerator(rm, cc)
        };

        auto opvpath = std::vector<RobotState>();
        shortcut::ShortcutPath(
                pv_path.begin(), pv_path.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(opvpath));

        auto opvpath_dnc = std::vector<RobotState>();
        shortcut::DivideAndConquerShortcutPath(
                pv_path.begin(), pv_path.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(opvpath_dnc));

        auto new_costs = std::vector<double>();
        ComputePositionVelocityPathCosts(rm, opvpath, new_costs);
        next_cost = std::accumulate(new_costs.begin(), new_costs.end(), 0.0);

        auto new_costs_dnc = std::vector<double>();
        ComputePositionVelocityPathCosts(rm, opvpath_dnc, new_costs_dnc);
        auto new_cost_dnc = std::accumulate(new_costs_dnc.begin(), new_costs_dnc.end(), 0.0);
        if (new_cost_dnc < next_cost) {
            SMPL_INFO("Divide and Conquer Wins! (%0.3f < %0.3f)", new_cost_dnc, next_cost);
            next_cost = new_cost_dnc;
            opvpath = std::move(opvpath_dnc);
        }

        ExtractPositionPath(rm, opvpath, pout);
    }   break;
    default:
        break;
    }

    auto now = clock::now();
    SMPL_INFO("Path shortcutting took %0.3f seconds", std::chrono::duration<double>(now - then).count());

    SMPL_INFO("Original path: waypoint count: %zu, cost: %0.3f", pin.size(), prev_cost);
    SMPL_INFO("Shortcutted path: waypount_count: %zu, cost: %0.3f", pout.size(), next_cost);
}

bool CreatePositionVelocityPath(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<RobotState>& opath)
{
    auto var_count = (int)rm->getPlanningJoints().size();

    // position + zero velocity for the first point
    auto pv_path = std::vector<RobotState>(path.size());
    if (!pv_path.empty()) {
        pv_path[0].resize(2 * var_count);
        // p_0 = p_in_0, v_0 = { 0, ..., 0 }
        std::copy(path[0].begin(), path[0].begin() + var_count, pv_path[0].begin());
        std::generate(pv_path[0].begin() + var_count, pv_path[0].end(), []() { return 0.0; });
    }

    // position + velocities for the remaining points
    for (auto i = 1; i < path.size(); ++i) {
        auto& from = path[i - 1];
        auto& to = path[i];
        pv_path[i].resize(2 * var_count);
        std::copy(to.begin(), to.begin() + var_count, pv_path[i].begin());
        for (auto vidx = 0; vidx < var_count; ++vidx) {
            if (!rm->hasPosLimit(vidx)) {
                pv_path[i][var_count + vidx] = std::copysign(1.0, shortest_angle_diff(to[vidx], from[vidx]));
            }
            else {
                pv_path[i][var_count + vidx] = std::copysign(1.0, to[vidx] - from[vidx]);
            }
        }
    }

    opath = std::move(pv_path);
    return true;
}

bool ExtractPositionPath(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<RobotState>& path)
{
    path.resize(pv_path.size());
    auto var_count = (int)rm->getPlanningJoints().size();
    for (auto pidx = 0; pidx < pv_path.size(); ++pidx) {
        path[pidx].resize(var_count);
        std::copy(&pv_path[pidx][0], &pv_path[pidx][0] + var_count, &path[pidx][0]);
    }
    return true;
}

bool ComputePositionPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<double>& costs)
{
    if (path.empty()) {
        return false;
    }

    costs.clear();
    costs.resize(path.size() - 1);
    for (auto i = 1; i < path.size(); ++i) {
        costs[i - 1] = distance(*rm, path[i - 1], path[i]);
    }
    return true;
}

bool ComputePositionVelocityPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<double>& costs)
{
    if (pv_path.empty()) {
        return false;
    }

    costs.clear();
    costs.resize(pv_path.size() - 1);
    for (auto i = 1; i < pv_path.size(); ++i) {
        costs[i - 1] = pv_distance(*rm, pv_path[i - 1], pv_path[i]);
    }
    return true;
}

bool InterpolatePath(CollisionChecker& cc, std::vector<RobotState>& path)
{
    if (path.empty()) {
        return true;
    }

    auto num_joints = path.front().size();
    for (auto& pt : path) {
        if (pt.size() != num_joints) {
            SMPL_ERROR("Failed to interpolate trajectory. Input trajectory is malformed");
            return false;
        }
    }

    auto opath = std::vector<RobotState>();

    // tack on the first point of the trajectory
    opath.push_back(path.front());

    // iterate over path segments
    for (auto i = 0; i < path.size() - 1; ++i) {
        auto& curr = path[i];
        auto& next = path[i + 1];

        SMPL_DEBUG_STREAM("Interpolating between " << curr << " and " << next);

        auto ipath = std::vector<RobotState>();
        if (!cc.interpolatePath(curr, next, ipath)) {
            SMPL_ERROR("Failed to interpolate between waypoint %d and %d because it's infeasible given the limits.", i, i + 1);
            return false;
        }

        // check the interpolated path for collisions, as the interpolator may
        // take a slightly different
        auto collision = false;
        for (auto& point : ipath) {
            if (!cc.isStateValid(point, false)) {
                collision = true;
                break;
            }
        }

        if (collision) {
            SMPL_ERROR("Interpolated path collides. Resorting to original waypoints");
            opath.push_back(next);
            continue;
        }

        if (!ipath.empty()) {
            // concatenate current path and the intermediate path (we already
            // have the first waypoint in the path from last iteration)
            opath.insert(end(opath), std::next(begin(ipath)), end(ipath));
        }

        SMPL_DEBUG("[%zu] path length: %zu", i, opath.size());
    }

    SMPL_INFO("Original path length: %zu   Interpolated path length: %zu", path.size(), opath.size());
    path = std::move(opath);
    return true;
}

} // namespace smpl
