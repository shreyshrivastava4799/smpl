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
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/spatial.h>

namespace smpl {

bool WorkspaceProjection::Init(RobotModel* robot, const Params& params)
{
    if (robot == NULL) return false;

    auto* fk_iface = robot->GetExtension<IForwardKinematics>();
    if (fk_iface == NULL) {
        SMPL_WARN("Workspace Lattice requires Forward Kinematics Interface extension");
        return false;
    }

    auto* ik_iface = robot->GetExtension<IInverseKinematics>();
    if (ik_iface == NULL) {
        SMPL_WARN("Workspace Lattice requires Inverse Kinematics Interface extension");
        return false;
    }

    auto* rm_iface = robot->GetExtension<IRedundantManipulator>();
    if (rm_iface == NULL) {
        SMPL_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return false;
    }

    m_robot_model = robot;
    m_fk_iface = fk_iface;
    m_ik_iface = ik_iface;
    m_rm_iface = rm_iface;

    m_fangle_indices.resize(m_rm_iface->redundantVariableCount());
    m_fangle_min_limits.resize(m_rm_iface->redundantVariableCount());
    m_fangle_max_limits.resize(m_rm_iface->redundantVariableCount());
    m_fangle_bounded.resize(m_rm_iface->redundantVariableCount());
    m_fangle_continuous.resize(m_rm_iface->redundantVariableCount());

    SMPL_DEBUG_NAMED(G_LOG, "%zu free angles", m_fangle_indices.size());
    for (auto i = 0; i < (int)m_fangle_indices.size(); ++i) {
        m_fangle_indices[i] = m_rm_iface->redundantVariableIndex(i);
        m_fangle_min_limits[i] = robot->minPosLimit(m_fangle_indices[i]);
        m_fangle_max_limits[i] = robot->maxPosLimit(m_fangle_indices[i]);
        m_fangle_bounded[i] = robot->hasPosLimit(m_fangle_indices[i]);
        m_fangle_continuous[i] = robot->isContinuous(m_fangle_indices[i]);
        SMPL_DEBUG_NAMED(G_LOG, "  name = %s, index = %zu, min = %f, max = %f, bounded = %d, continuous = %d",
                m_rm_iface->getPlanningJoints()[m_fangle_indices[i]].c_str(),
                m_fangle_indices[i],
                m_fangle_min_limits[i],
                m_fangle_max_limits[i],
                (int)m_fangle_bounded[i],
                (int)m_fangle_continuous[i]);
    }

    m_dof_count = 6 + (int)m_fangle_indices.size();

    m_res.resize(m_dof_count);
    m_val_count.resize(m_dof_count);

    m_val_count[0] = std::numeric_limits<int>::max();
    m_val_count[1] = std::numeric_limits<int>::max();
    m_val_count[2] = std::numeric_limits<int>::max();
    m_val_count[3] = params.R_count;
    m_val_count[4] = params.P_count;
    m_val_count[5] = params.Y_count;
    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        if (m_fangle_continuous[i]) {
            m_val_count[6 + i] = (int)std::round((2.0 * M_PI) / params.free_angle_res[i]);
        } else if (m_fangle_bounded[i]) {
            auto span = std::fabs(m_fangle_max_limits[i] - m_fangle_min_limits[i]);
            m_val_count[6 + i] = std::max(1, (int)std::round(span / params.free_angle_res[i]));
        } else {
            m_val_count[6 + i] = std::numeric_limits<int>::max();
        }
    }

    m_res[0] = params.res_x;
    m_res[1] = params.res_y;
    m_res[2] = params.res_z;
    // TODO: limit these ranges and handle discretization appropriately
    m_res[3] = 2.0 * M_PI / params.R_count;
    m_res[4] = M_PI       / (params.P_count - 1);
    m_res[5] = 2.0 * M_PI / params.Y_count;

    for (int i = 0; i < m_fangle_indices.size(); ++i) {
        if (m_fangle_continuous[i]) {
            m_res[6 + i] = (2.0 * M_PI) / (double)m_val_count[6 + i];
        } else if (m_fangle_bounded[i]) {
            auto span = std::fabs(m_fangle_max_limits[i] - m_fangle_min_limits[i]);
            m_res[6 + i] = span / m_val_count[6 + i];
        } else {
            m_res[6 + i] = params.free_angle_res[i];
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

bool WorkspaceProjection::Initialized() const
{
    return m_fk_iface != NULL;
}

void WorkspaceProjection::StateRobotToWorkspace(
    const RobotState& state,
    WorkspaceState& ostate) const
{
    auto pose = m_fk_iface->computeFK(state);

    ostate.resize(m_dof_count);
    ostate[0] = pose.translation().x();
    ostate[1] = pose.translation().y();
    ostate[2] = pose.translation().z();

    get_euler_zyx(pose.rotation(), ostate[5], ostate[4], ostate[3]);

    for (auto fai = 0; fai < FreeAngleCount(); ++fai) {
        ostate[6 + fai] = state[m_fangle_indices[fai]];
    }
}

void WorkspaceProjection::StateRobotToCoord(
    const RobotState& state,
    WorkspaceCoord& coord) const
{
    auto ws_state = WorkspaceState();
    StateRobotToWorkspace(state, ws_state);
    StateWorkspaceToCoord(ws_state, coord);
}

bool WorkspaceProjection::StateWorkspaceToRobot(
    const WorkspaceState& state,
    RobotState& ostate) const
{
    auto seed = RobotState(m_robot_model->jointVariableCount(), 0);
    for (auto fai = 0; fai < FreeAngleCount(); ++fai) {
        seed[m_fangle_indices[fai]] = state[6 + fai];
    }

    auto pose = MakeAffine(
            state[0], state[1], state[2], state[5], state[4], state[3]);

    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceProjection::StateWorkspaceToCoord(
    const WorkspaceState& state,
    WorkspaceCoord& coord) const
{
    coord.resize(m_dof_count);
    PosWorkspaceToCoord(&state[0], &coord[0]);
    RotWorkspaceToCoord(&state[3], &coord[3]);
    FavWorkspaceToCoord(&state[6], &coord[6]);
}

bool WorkspaceProjection::StateCoordToRobot(
    const WorkspaceCoord& coord,
    RobotState& state) const
{
    return false;
}

void WorkspaceProjection::StateCoordToWorkspace(
    const WorkspaceCoord& coord,
    WorkspaceState& state) const
{
    state.resize(m_dof_count);
    PosCoordToWorkspace(&coord[0], &state[0]);
    RotCoordToWorkspace(&coord[3], &state[3]);
    FavCoordToWorkspace(&coord[6], &state[6]);
}

bool WorkspaceProjection::StateWorkspaceToRobot(
    const WorkspaceState& state,
    const RobotState& seed,
    RobotState& ostate) const
{
    auto pose = MakeAffine(
            state[0], state[1], state[2], state[5], state[4], state[3]);

    // TODO: unrestricted variant?
    return m_rm_iface->computeFastIK(pose, seed, ostate);
}

void WorkspaceProjection::PosWorkspaceToCoord(const double* wp, int* gp) const
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

void WorkspaceProjection::PosCoordToWorkspace(const int* gp, double* wp) const
{
    wp[0] = gp[0] * m_res[0] + 0.5 * m_res[0];
    wp[1] = gp[1] * m_res[1] + 0.5 * m_res[1];
    wp[2] = gp[2] * m_res[2] + 0.5 * m_res[2];
}

void WorkspaceProjection::RotWorkspaceToCoord(const double* wr, int* gr) const
{
    gr[0] = (int)((normalize_angle_positive(wr[0]) + m_res[3] * 0.5) / m_res[3]) % m_val_count[3];
    gr[1] = (int)((normalize_angle(wr[1]) + (0.5 * M_PI) + m_res[4] * 0.5) / m_res[4]) % m_val_count[4];
    gr[2] = (int)((normalize_angle_positive(wr[2]) + m_res[5] * 0.5) / m_res[5]) % m_val_count[5];
}

void WorkspaceProjection::RotCoordToWorkspace(const int* gr, double* wr) const
{
    // TODO: this normalize is probably not necessary
    wr[0] = normalize_angle((double)gr[0] * m_res[3]);
    wr[1] = normalize_angle(-0.5 * M_PI + (double)gr[1] * m_res[4]);
    wr[2] = normalize_angle((double)gr[2] * m_res[5]);
}

void WorkspaceProjection::PoseWorkspaceToCoord(const double* wp, int* gp) const
{
    PosWorkspaceToCoord(wp, gp);
    RotWorkspaceToCoord(wp + 3, gp + 3);
}

void WorkspaceProjection::PoseCoordToWorkspace(const int* gp, double* wp) const
{
    PosCoordToWorkspace(gp, wp);
    RotCoordToWorkspace(gp + 3, wp + 3);
}

void WorkspaceProjection::FavWorkspaceToCoord(const double* wa, int* ga) const
{
    for (size_t fai = 0; fai < FreeAngleCount(); ++fai) {
        if (m_fangle_continuous[fai]) {
            auto pos_angle = normalize_angle_positive(wa[fai]);

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

void WorkspaceProjection::FavCoordToWorkspace(const int* ga, double* wa) const
{
    for (size_t i = 0; i < FreeAngleCount(); ++i) {
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
