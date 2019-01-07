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

#include <smpl/graph/workspace_projection.h>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/spatial.h>

namespace smpl {

bool InitWorkspaceProjection(
    WorkspaceProjection* proj,
    RobotModel* robot,
    const WorkspaceProjectionParams& params)
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

    proj->robot_model = robot;
    proj->fk_iface = fk_iface;
    proj->ik_iface = ik_iface;
    proj->rm_iface = rm_iface;

    proj->fa_indices.resize(rm_iface->redundantVariableCount());
    proj->fa_min_limits.resize(rm_iface->redundantVariableCount());
    proj->fa_max_limits.resize(rm_iface->redundantVariableCount());
    proj->fa_bounded.resize(rm_iface->redundantVariableCount());
    proj->fa_continuous.resize(rm_iface->redundantVariableCount());

    SMPL_DEBUG_NAMED(G_LOG, "%zu free angles", proj->fa_indices.size());
    for (auto i = 0; i < (int)proj->fa_indices.size(); ++i) {
        proj->fa_indices[i] = proj->rm_iface->redundantVariableIndex(i);
        proj->fa_min_limits[i] = robot->minPosLimit(proj->fa_indices[i]);
        proj->fa_max_limits[i] = robot->maxPosLimit(proj->fa_indices[i]);
        proj->fa_bounded[i] = robot->hasPosLimit(proj->fa_indices[i]);
        proj->fa_continuous[i] = robot->isContinuous(proj->fa_indices[i]);
        SMPL_DEBUG_NAMED(G_LOG, "  name = %s, index = %zu, min = %f, max = %f, bounded = %d, continuous = %d",
                proj->rm_iface->getPlanningJoints()[proj->fa_indices[i]].c_str(),
                proj->fa_indices[i],
                proj->fa_min_limits[i],
                proj->fa_max_limits[i],
                (int)proj->fa_bounded[i],
                (int)proj->fa_continuous[i]);
    }

    proj->dof_count = 6 + (int)proj->fa_indices.size();

    proj->res.resize(proj->dof_count);
    proj->val_count.resize(proj->dof_count);

    proj->val_count[0] = std::numeric_limits<int>::max();
    proj->val_count[1] = std::numeric_limits<int>::max();
    proj->val_count[2] = std::numeric_limits<int>::max();
    proj->val_count[3] = params.R_count;
    proj->val_count[4] = params.P_count;
    proj->val_count[5] = params.Y_count;
    for (int i = 0; i < proj->fa_indices.size(); ++i) {
        if (proj->fa_continuous[i]) {
            proj->val_count[6 + i] = (int)std::round((2.0 * M_PI) / params.free_angle_res[i]);
        } else if (proj->fa_bounded[i]) {
            auto span = std::fabs(proj->fa_max_limits[i] - proj->fa_min_limits[i]);
            proj->val_count[6 + i] = std::max(1, (int)std::round(span / params.free_angle_res[i]));
        } else {
            proj->val_count[6 + i] = std::numeric_limits<int>::max();
        }
    }

    proj->res[0] = params.res_x;
    proj->res[1] = params.res_y;
    proj->res[2] = params.res_z;
    // TODO: limit these ranges and handle discretization appropriately
    proj->res[3] = 2.0 * M_PI / params.R_count;
    proj->res[4] = M_PI       / (params.P_count - 1);
    proj->res[5] = 2.0 * M_PI / params.Y_count;

    for (auto i = 0; i < proj->fa_indices.size(); ++i) {
        if (proj->fa_continuous[i]) {
            proj->res[6 + i] = (2.0 * M_PI) / (double)proj->val_count[6 + i];
        } else if (proj->fa_bounded[i]) {
            auto span = std::fabs(proj->fa_max_limits[i] - proj->fa_min_limits[i]);
            proj->res[6 + i] = span / proj->val_count[6 + i];
        } else {
            proj->res[6 + i] = params.free_angle_res[i];
        }
    }

    SMPL_DEBUG_NAMED(G_LOG, "discretization of workspace lattice:");
    SMPL_DEBUG_NAMED(G_LOG, "  x: { res: %f, count: %d }", proj->res[0], proj->val_count[0]);
    SMPL_DEBUG_NAMED(G_LOG, "  y: { res: %f, count: %d }", proj->res[1], proj->val_count[1]);
    SMPL_DEBUG_NAMED(G_LOG, "  z: { res: %f, count: %d }", proj->res[2], proj->val_count[2]);
    SMPL_DEBUG_NAMED(G_LOG, "  R: { res: %f, count: %d }", proj->res[3], proj->val_count[3]);
    SMPL_DEBUG_NAMED(G_LOG, "  P: { res: %f, count: %d }", proj->res[4], proj->val_count[4]);
    SMPL_DEBUG_NAMED(G_LOG, "  Y: { res: %f, count: %d }", proj->res[5], proj->val_count[5]);
    for (int i = 0; i < proj->fa_indices.size(); ++i) {
        SMPL_DEBUG_NAMED(G_LOG, "  J%d: { res: %f, count: %d }", i, proj->res[6 + i], proj->val_count[6 + i]);
    }

    return true;
}

auto GetResolutions(const WorkspaceProjection* proj) -> const std::vector<double>&
{
    return proj->res;
}

auto GetNumDOFs(const WorkspaceProjection* proj) -> int
{
    return proj->dof_count;
}

auto GetNumFreeAngles(const WorkspaceProjection* proj) -> int
{
    return (int)proj->fa_indices.size();
}

void StateRobotToWorkspace(
    const WorkspaceProjection* proj,
    const RobotState& state,
    WorkspaceState& ostate)
{
    auto pose = proj->fk_iface->computeFK(state);

    ostate.resize(proj->dof_count);
    ostate[0] = pose.translation().x();
    ostate[1] = pose.translation().y();
    ostate[2] = pose.translation().z();

    get_euler_zyx(pose.rotation(), ostate[5], ostate[4], ostate[3]);

    for (auto fai = 0; fai < GetNumFreeAngles(proj); ++fai) {
        ostate[6 + fai] = state[proj->fa_indices[fai]];
    }
}

void StateRobotToCoord(
    const WorkspaceProjection* proj,
    const RobotState& state,
    WorkspaceCoord& coord)
{
    auto ws_state = WorkspaceState();
    StateRobotToWorkspace(proj, state, ws_state);
    StateWorkspaceToCoord(proj, ws_state, coord);
}

bool StateWorkspaceToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    RobotState& ostate)
{
    auto seed = RobotState(proj->robot_model->jointVariableCount(), 0);
    for (auto fai = 0; fai < GetNumFreeAngles(proj); ++fai) {
        seed[proj->fa_indices[fai]] = state[6 + fai];
    }

    auto pose = MakeAffine(
            state[0], state[1], state[2], state[5], state[4], state[3]);

    return proj->rm_iface->computeFastIK(pose, seed, ostate);
}

void StateWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    WorkspaceCoord& coord)
{
    coord.resize(proj->dof_count);
    PosWorkspaceToCoord(proj, &state[0], &coord[0]);
    RotWorkspaceToCoord(proj, &state[3], &coord[3]);
    FavWorkspaceToCoord(proj, &state[6], &coord[6]);
}

bool StateCoordToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceCoord& coord,
    RobotState& state)
{
    return false;
}

void StateCoordToWorkspace(
    const WorkspaceProjection* proj,
    const WorkspaceCoord& coord,
    WorkspaceState& state)
{
    state.resize(proj->dof_count);
    PosCoordToWorkspace(proj, &coord[0], &state[0]);
    RotCoordToWorkspace(proj, &coord[3], &state[3]);
    FavCoordToWorkspace(proj, &coord[6], &state[6]);
}

bool StateWorkspaceToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    const RobotState& seed,
    RobotState& ostate)
{
    auto pose = MakeAffine(
            state[0], state[1], state[2], state[5], state[4], state[3]);

    // TODO: unrestricted variant?
    return proj->rm_iface->computeFastIK(pose, seed, ostate);
}

void PosWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wp,
    int* gp)
{
    if (wp[0] >= 0.0) {
        gp[0] = (int)(wp[0] / proj->res[0]);
    } else {
        gp[0] = (int)(wp[0] / proj->res[0]) - 1;
    }

    if (wp[1] >= 0.0) {
        gp[1] = (int)(wp[1] / proj->res[1]);
    } else {
        gp[1] = (int)(wp[1] / proj->res[1]) - 1;
    }

    if (wp[2] >= 0.0) {
        gp[2] = (int)(wp[2] / proj->res[2]);
    } else {
        gp[2] = (int)(wp[2] / proj->res[2]) - 1;
    }
}

void PosCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gp,
    double* wp)
{
    wp[0] = gp[0] * proj->res[0] + 0.5 * proj->res[0];
    wp[1] = gp[1] * proj->res[1] + 0.5 * proj->res[1];
    wp[2] = gp[2] * proj->res[2] + 0.5 * proj->res[2];
}

void RotWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wr,
    int* gr)
{
    gr[0] = (int)((normalize_angle_positive(wr[0]) + proj->res[3] * 0.5) / proj->res[3]) % proj->val_count[3];
    gr[1] = (int)((normalize_angle(wr[1]) + (0.5 * M_PI) + proj->res[4] * 0.5) / proj->res[4]) % proj->val_count[4];
    gr[2] = (int)((normalize_angle_positive(wr[2]) + proj->res[5] * 0.5) / proj->res[5]) % proj->val_count[5];
}

void RotCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gr,
    double* wr)
{
    // TODO: this normalize is probably not necessary
    wr[0] = normalize_angle((double)gr[0] * proj->res[3]);
    wr[1] = normalize_angle(-0.5 * M_PI + (double)gr[1] * proj->res[4]);
    wr[2] = normalize_angle((double)gr[2] * proj->res[5]);
}

void PoseWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wp,
    int* gp)
{
    PosWorkspaceToCoord(proj, wp, gp);
    RotWorkspaceToCoord(proj, wp + 3, gp + 3);
}

void PoseCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gp,
    double* wp)
{
    PosCoordToWorkspace(proj, gp, wp);
    RotCoordToWorkspace(proj, gp + 3, wp + 3);
}

void FavWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wa,
    int* ga)
{
    for (auto fai = 0; fai < GetNumFreeAngles(proj); ++fai) {
        if (proj->fa_continuous[fai]) {
            auto pos_angle = normalize_angle_positive(wa[fai]);

            ga[fai] = (int)((pos_angle + proj->res[6 + fai] * 0.5) / proj->res[6 + fai]);

            if (ga[fai] == proj->val_count[6 + fai]) {
                ga[fai] = 0;
            }
        } else if (!proj->fa_bounded[fai]) {
            if (wa[fai] >= 0.0) {
                ga[fai] = (int)(wa[fai] / proj->res[6 + fai] + 0.5);
            } else {
                ga[fai] = (int)(wa[fai] / proj->res[6 + fai] - 0.5);
            }
        } else {
            ga[fai] = (int)(((wa[fai] - proj->fa_min_limits[fai]) / proj->res[6 + fai]) + 0.5);
        }
    }
}

void FavCoordToWorkspace(const WorkspaceProjection* proj, const int* ga, double* wa)
{
    for (size_t i = 0; i < GetNumFreeAngles(proj); ++i) {
        if (proj->fa_continuous[i]) {
            wa[i] = (double)ga[i] * proj->res[6 + i];
        } else if (!proj->fa_bounded[i]) {
            wa[i] = (double)ga[i] * proj->res[6 + i];// + 0.5 * res[6 + i];
        } else {
            wa[i] = proj->fa_min_limits[i] + ga[i] * proj->res[6 + i];
        }
    }
}

} // namespace smpl
