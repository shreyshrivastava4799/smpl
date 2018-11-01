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

#include <smpl/graph/workspace_lattice_base.h>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/spatial.h>

namespace smpl {

bool WorkspaceLatticeBase::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const Params& _params)
{
    if (!RobotPlanningSpace::init(_robot, checker)) {
        return false;
    }

    m_fk_iface = _robot->getExtension<ForwardKinematicsInterface>();
    if (!m_fk_iface) {
        SMPL_WARN("Workspace Lattice requires Forward Kinematics Interface extension");
        return false;
    }

    m_ik_iface = _robot->getExtension<InverseKinematicsInterface>();
    if (!m_ik_iface) {
        SMPL_WARN("Workspace Lattice requires Inverse Kinematics Interface extension");
        return false;
    }

    m_rm_iface = _robot->getExtension<RedundantManipulatorInterface>();
    if (!m_rm_iface) {
        SMPL_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return false;
    }

    m_fangle_indices.resize(m_rm_iface->redundantVariableCount());
    m_fangle_min_limits.resize(m_rm_iface->redundantVariableCount());
    m_fangle_max_limits.resize(m_rm_iface->redundantVariableCount());
    m_fangle_bounded.resize(m_rm_iface->redundantVariableCount());
    m_fangle_continuous.resize(m_rm_iface->redundantVariableCount());

    SMPL_DEBUG_NAMED(G_LOG, "%zu free angles", m_fangle_indices.size());
    for (size_t i = 0; i < m_fangle_indices.size(); ++i) {
        m_fangle_indices[i] = m_rm_iface->redundantVariableIndex(i);
        m_fangle_min_limits[i] = _robot->minPosLimit(m_fangle_indices[i]);
        m_fangle_max_limits[i] = _robot->maxPosLimit(m_fangle_indices[i]);
        m_fangle_bounded[i] = _robot->hasPosLimit(m_fangle_indices[i]);
        m_fangle_continuous[i] = _robot->isContinuous(m_fangle_indices[i]);
        SMPL_DEBUG_NAMED(G_LOG, "  name = %s, index = %zu, min = %f, max = %f, bounded = %d, continuous = %d",
                m_rm_iface->getPlanningJoints()[m_fangle_indices[i]].c_str(),
                m_fangle_indices[i],
                m_fangle_min_limits[i],
                m_fangle_max_limits[i],
                (int)m_fangle_bounded[i],
                (int)m_fangle_continuous[i]);
    }

    m_dof_count = 6 + m_fangle_indices.size();

    m_res.resize(m_dof_count);
    m_val_count.resize(m_dof_count);

    m_val_count[0] = std::numeric_limits<int>::max();
    m_val_count[1] = std::numeric_limits<int>::max();
    m_val_count[2] = std::numeric_limits<int>::max();
    m_val_count[3] = _params.R_count;
    m_val_count[4] = _params.P_count;
    m_val_count[5] = _params.Y_count;
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        if (m_fangle_continuous[i]) {
            m_val_count[6 + i] = (int)std::round((2.0 * M_PI) / _params.free_angle_res[i]);
        } else if (m_fangle_bounded[i]) {
            auto span = std::fabs(m_fangle_max_limits[i] - m_fangle_min_limits[i]);
            m_val_count[6 + i] = std::max(1, (int)std::round(span / _params.free_angle_res[i]));
        } else {
            m_val_count[6 + i] = std::numeric_limits<int>::max();
        }
    }

    m_res[0] = _params.res_x;
    m_res[1] = _params.res_y;
    m_res[2] = _params.res_z;
    // TODO: limit these ranges and handle discretization appropriately
    m_res[3] = 2.0 * M_PI / _params.R_count;
    m_res[4] = M_PI       / (_params.P_count - 1);
    m_res[5] = 2.0 * M_PI / _params.Y_count;

    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        if (m_fangle_continuous[i]) {
            m_res[6 + i] = (2.0 * M_PI) / (double)m_val_count[6 + i];
        } else if (m_fangle_bounded[i]) {
            auto span = std::fabs(m_fangle_max_limits[i] - m_fangle_min_limits[i]);
            m_res[6 + i] = span / m_val_count[6 + i];
        } else {
            m_res[6 + i] = _params.free_angle_res[i];
        }
    }

    SMPL_DEBUG_NAMED(G_LOG, "discretization of workspace lattice:");
    SMPL_DEBUG_NAMED(G_LOG, "  x: { res: %f, count: %d }", m_res[0], m_val_count[0]);
    SMPL_DEBUG_NAMED(G_LOG, "  y: { res: %f, count: %d }", m_res[1], m_val_count[1]);
    SMPL_DEBUG_NAMED(G_LOG, "  z: { res: %f, count: %d }", m_res[2], m_val_count[2]);
    SMPL_DEBUG_NAMED(G_LOG, "  R: { res: %f, count: %d }", m_res[3], m_val_count[3]);
    SMPL_DEBUG_NAMED(G_LOG, "  P: { res: %f, count: %d }", m_res[4], m_val_count[4]);
    SMPL_DEBUG_NAMED(G_LOG, "  Y: { res: %f, count: %d }", m_res[5], m_val_count[5]);
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        SMPL_DEBUG_NAMED(G_LOG, "  J%d: { res: %f, count: %d }", i, m_res[6 + i], m_val_count[6 + i]);
    }

    return true;
}

bool WorkspaceLatticeBase::initialized() const
{
    return (bool)m_fk_iface;
}

void WorkspaceLatticeBase::stateRobotToWorkspace(
    const RobotState& state,
    WorkspaceState& ostate) const
{
    auto pose = m_fk_iface->computeFK(state);

    ostate.resize(m_dof_count);
    ostate[0] = pose.translation().x();
    ostate[1] = pose.translation().y();
    ostate[2] = pose.translation().z();

    get_euler_zyx(pose.rotation(), ostate[5], ostate[4], ostate[3]);

    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        ostate[6 + fai] = state[m_fangle_indices[fai]];
    }
}

void WorkspaceLatticeBase::stateRobotToCoord(
    const RobotState& state,
    WorkspaceCoord& coord) const
{
    WorkspaceState ws_state;
    stateRobotToWorkspace(state, ws_state);
    stateWorkspaceToCoord(ws_state, coord);
}

bool WorkspaceLatticeBase::stateWorkspaceToRobot(
    const WorkspaceState& state,
    RobotState& ostate) const
{
    RobotState seed(robot()->jointVariableCount(), 0);
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        seed[m_fangle_indices[fai]] = state[6 + fai];
    }

    Affine3 pose =
            Translation3(state[0], state[1], state[2]) *
            AngleAxis(state[5], Vector3::UnitZ()) *
            AngleAxis(state[4], Vector3::UnitY()) *
            AngleAxis(state[3], Vector3::UnitX());

    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceLatticeBase::stateWorkspaceToCoord(
    const WorkspaceState& state,
    WorkspaceCoord& coord) const
{
    coord.resize(m_dof_count);
    posWorkspaceToCoord(&state[0], &coord[0]);
    rotWorkspaceToCoord(&state[3], &coord[3]);
    favWorkspaceToCoord(&state[6], &coord[6]);
}

bool WorkspaceLatticeBase::stateCoordToRobot(
    const WorkspaceCoord& coord,
    RobotState& state) const
{
    return false;
}

void WorkspaceLatticeBase::stateCoordToWorkspace(
    const WorkspaceCoord& coord,
    WorkspaceState& state) const
{
    state.resize(m_dof_count);
    posCoordToWorkspace(&coord[0], &state[0]);
    rotCoordToWorkspace(&coord[3], &state[3]);
    favCoordToWorkspace(&coord[6], &state[6]);
}

bool WorkspaceLatticeBase::stateWorkspaceToRobot(
    const WorkspaceState& state,
    const RobotState& seed,
    RobotState& ostate) const
{
    Affine3 pose =
            Translation3(state[0], state[1], state[2]) *
            AngleAxis(state[5], Vector3::UnitZ()) *
            AngleAxis(state[4], Vector3::UnitY()) *
            AngleAxis(state[3], Vector3::UnitX());

    // TODO: unrestricted variant?
    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceLatticeBase::posWorkspaceToCoord(const double* wp, int* gp) const
{
    if (wp[0] >= 0.0) {
        gp[0] = (int)(wp[0] / m_res[0]);
    } else {
        gp[0] = (int)(wp[0] / m_res[0]) - 1;
    }

    if (wp[1] >= 0.0) {
        gp[1] = (int)(wp[1] / m_res[1]);
    } else {
        gp[1] = (int)(wp[1] / m_res[1]) - 1;
    }

    if (wp[2] >= 0.0) {
        gp[2] = (int)(wp[2] / m_res[2]);
    } else {
        gp[2] = (int)(wp[2] / m_res[2]) - 1;
    }
}

void WorkspaceLatticeBase::posCoordToWorkspace(const int* gp, double* wp) const
{
    wp[0] = gp[0] * m_res[0] + 0.5 * m_res[0];
    wp[1] = gp[1] * m_res[1] + 0.5 * m_res[1];
    wp[2] = gp[2] * m_res[2] + 0.5 * m_res[2];
}

void WorkspaceLatticeBase::rotWorkspaceToCoord(const double* wr, int* gr) const
{
    gr[0] = (int)((angles::normalize_angle_positive(wr[0]) + m_res[3] * 0.5) / m_res[3]) % m_val_count[3];
    gr[1] = (int)((angles::normalize_angle(wr[1]) + (0.5 * M_PI) + m_res[4] * 0.5) / m_res[4]) % m_val_count[4];
    gr[2] = (int)((angles::normalize_angle_positive(wr[2]) + m_res[5] * 0.5) / m_res[5]) % m_val_count[5];
}

void WorkspaceLatticeBase::rotCoordToWorkspace(const int* gr, double* wr) const
{
    // TODO: this normalize is probably not necessary
    wr[0] = angles::normalize_angle((double)gr[0] * m_res[3]);
    wr[1] = angles::normalize_angle(-0.5 * M_PI + (double)gr[1] * m_res[4]);
    wr[2] = angles::normalize_angle((double)gr[2] * m_res[5]);
}

void WorkspaceLatticeBase::poseWorkspaceToCoord(const double* wp, int* gp) const
{
    posWorkspaceToCoord(wp, gp);
    rotWorkspaceToCoord(wp + 3, gp + 3);
}

void WorkspaceLatticeBase::poseCoordToWorkspace(const int* gp, double* wp) const
{
    posCoordToWorkspace(gp, wp);
    rotCoordToWorkspace(gp + 3, wp + 3);
}

void WorkspaceLatticeBase::favWorkspaceToCoord(const double* wa, int* ga) const
{
    for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
        if (m_fangle_continuous[fai]) {
            auto pos_angle = angles::normalize_angle_positive(wa[fai]);

            ga[fai] = (int)((pos_angle + m_res[6 + fai] * 0.5) / m_res[6 + fai]);

            if (ga[fai] == m_val_count[6 + fai]) {
                ga[fai] = 0;
            }
        } else if (!m_fangle_bounded[fai]) {
            if (wa[fai] >= 0.0) {
                ga[fai] = (int)(wa[fai] / m_res[6 + fai] + 0.5);
            } else {
                ga[fai] = (int)(wa[fai] / m_res[6 + fai] - 0.5);
            }
        } else {
            ga[fai] = (int)(((wa[fai] - m_fangle_min_limits[fai]) / m_res[6 + fai]) + 0.5);
        }
    }
}

void WorkspaceLatticeBase::favCoordToWorkspace(const int* ga, double* wa) const
{
    for (size_t i = 0; i < freeAngleCount(); ++i) {
        if (m_fangle_continuous[i]) {
            wa[i] = (double)ga[i] * m_res[6 + i];
        } else if (!m_fangle_bounded[i]) {
            wa[i] = (double)ga[i] * m_res[6 + i];// + 0.5 * m_res[6 + i];
        } else {
            wa[i] = m_fangle_min_limits[i] + ga[i] * m_res[6 + i];
        }
    }
}

} // namespace smpl
