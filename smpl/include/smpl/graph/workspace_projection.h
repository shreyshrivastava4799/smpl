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

#ifndef SMPL_WORKSPACE_LATTICE_BASE_H
#define SMPL_WORKSPACE_LATTICE_BASE_H

// project includes
#include <smpl/graph/workspace_lattice_types.h>

namespace smpl {

class IForwardKinematics;
class IInverseKinematics;
class IRedundantManipulator;
class RobotModel;

struct WorkspaceProjectionParams
{
    double res_x;
    double res_y;
    double res_z;

    int R_count;
    int P_count;
    int Y_count;

    std::vector<double> free_angle_res;
};

/// Base class for graph representations that represent states via a 1:1 mapping
/// from joint space states to states in SE(3) alongside an array of redundant
/// variables used to uniquely determine the state. This class is responsible
/// for handling those transformations and discretization of the resulting
/// SE(3) + <free angle array> space.
struct WorkspaceProjection
{
    RobotModel*             robot_model = NULL;
    IForwardKinematics*     fk_iface = NULL;
    IInverseKinematics*     ik_iface = NULL;
    IRedundantManipulator*  rm_iface = NULL;

    std::vector<double> res;
    std::vector<int>    val_count;
    int                 dof_count = 0;
    std::vector<int>    fa_indices;
    std::vector<double> fa_min_limits;
    std::vector<double> fa_max_limits;
    std::vector<bool>   fa_continuous;
    std::vector<bool>   fa_bounded;
};

bool InitWorkspaceProjection(
    WorkspaceProjection* proj,
    RobotModel* robot,
    const WorkspaceProjectionParams& params);

auto GetResolutions(const WorkspaceProjection* proj) -> const std::vector<double>&;

auto GetNumDOFs(const WorkspaceProjection* proj) -> int;

auto GetNumFreeAngles(const WorkspaceProjection* proj) -> int;

void StateRobotToWorkspace(
    const WorkspaceProjection* proj,
    const RobotState& state,
    WorkspaceState& ostate);

void StateRobotToCoord(
    const WorkspaceProjection* proj,
    const RobotState& state,
    WorkspaceCoord& coord);

bool StateWorkspaceToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    RobotState& ostate);

void StateWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    WorkspaceCoord& coord);

bool StateCoordToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceCoord& coord,
    RobotState& state);

void StateCoordToWorkspace(
    const WorkspaceProjection* proj,
    const WorkspaceCoord& coord,
    WorkspaceState& state);

bool StateWorkspaceToRobot(
    const WorkspaceProjection* proj,
    const WorkspaceState& state,
    const RobotState& seed,
    RobotState& ostate);

// TODO: variants of workspace -> robot that don't restrict redundant angles
// TODO: variants of workspace -> robot that take in a full seed state

// conversions from discrete coordinates to continuous states
void PosWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wp,
    int* gp);

void PosCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gp,
    double* wp);

void RotWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wr,
    int* gr);

void RotCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gr,
    double* wr);

void PoseWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wp,
    int* gp);

void PoseCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* gp,
    double* wp);

void FavWorkspaceToCoord(
    const WorkspaceProjection* proj,
    const double* wa,
    int* ga);

void FavCoordToWorkspace(
    const WorkspaceProjection* proj,
    const int* ga,
    double* wa);

} // namespace smpl

#endif
