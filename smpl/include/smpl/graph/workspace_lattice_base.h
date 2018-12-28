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

/// Base class for graph representations that represent states via a 1:1 mapping
/// from joint space states to states in SE(3) alongside an array of redundant
/// variables used to uniquely determine the state. This class is responsible
/// for handling those transformations and discretization of the resulting
/// SE(3) + <free angle array> space.
struct WorkspaceLatticeBase
{
    RobotModel*             m_robot_model = NULL;
    IForwardKinematics*     m_fk_iface = NULL;
    IInverseKinematics*     m_ik_iface = NULL;
    IRedundantManipulator*  m_rm_iface = NULL;

    std::vector<double> m_res;
    std::vector<int>    m_val_count;
    int                 m_dof_count = 0;
    std::vector<int>    m_fangle_indices;
    std::vector<double> m_fangle_min_limits;
    std::vector<double> m_fangle_max_limits;
    std::vector<bool>   m_fangle_continuous;
    std::vector<bool>   m_fangle_bounded;

    struct Params
    {
        double res_x;
        double res_y;
        double res_z;

        int R_count;
        int P_count;
        int Y_count;

        std::vector<double> free_angle_res;
    };

    bool Init(RobotModel* robot, const Params& params);

    bool Initialized() const;

    auto Resolution() const -> const std::vector<double>& { return m_res; }
    int DofCount() const { return m_dof_count; }

    auto FreeAngleCount() const { return (int)m_fangle_indices.size(); }

    // conversions between robot states, workspace states, and workspace coords
    void StateRobotToWorkspace(const RobotState& state, WorkspaceState& ostate) const;
    void StateRobotToCoord(const RobotState& state, WorkspaceCoord& coord) const;
    bool StateWorkspaceToRobot(const WorkspaceState& state, RobotState& ostate) const;
    void StateWorkspaceToCoord(const WorkspaceState& state, WorkspaceCoord& coord) const;
    bool StateCoordToRobot(const WorkspaceCoord& coord, RobotState& state) const;
    void StateCoordToWorkspace(const WorkspaceCoord& coord, WorkspaceState& state) const;

    bool StateWorkspaceToRobot(
        const WorkspaceState& state, const RobotState& seed, RobotState& ostate) const;

    // TODO: variants of workspace -> robot that don't restrict redundant angles
    // TODO: variants of workspace -> robot that take in a full seed state

    // conversions from discrete coordinates to continuous states
    void PosWorkspaceToCoord(const double* wp, int* gp) const;
    void PosCoordToWorkspace(const int* gp, double* wp) const;
    void RotWorkspaceToCoord(const double* wr, int* gr) const;
    void RotCoordToWorkspace(const int* gr, double* wr) const;
    void PoseWorkspaceToCoord(const double* wp, int* gp) const;
    void PoseCoordToWorkspace(const int* gp, double* wp) const;
    void FavWorkspaceToCoord(const double* wa, int* ga) const;
    void FavCoordToWorkspace(const int* ga, double* wa) const;
};

} // namespace smpl

#endif
