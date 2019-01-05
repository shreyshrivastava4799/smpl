////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen
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

#ifndef SBPL_PR2_ROBOT_MODEL_PR2_KDL_ROBOT_MODEL_H
#define SBPL_PR2_ROBOT_MODEL_PR2_KDL_ROBOT_MODEL_H

// standard includes
#include <memory>
#include <string>

// system includes
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// project includes
#include <sbpl_pr2_robot_model/orientation_solver.h>

namespace smpl {

namespace urdf {
struct Link;
struct JointVariable;
} // namespace urdf

// Implements the SMPL RobotModel interface and extensions for the PR2 arm.
// Forward kinematics are provided by an embedded KDLRobotModel (which itself
// provides them via URDFRobotModel). 6-DOF Inverse kinematics are provided by
// an embedded PR2ArmIKSolver and 3-DOF Orientation-only inverse kinematics are
// provided by a custom RPYSolver implementation. Any missing inverse kinematics
// capabilities (misconfigured rpy solver or other ik constraints) are forwarded
// to KDLRobotModel, which may or may not be able to support them.
class PR2RobotModel;

bool InitPR2RobotModel(
    PR2RobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle = -1);

bool InitPR2RobotModel(
    PR2RobotModel* model,
    const ::urdf::Model* urdf,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle = -1);

auto GetBaseLink(const PR2RobotModel* model) -> const std::string&;
auto GetPlanningLink(const PR2RobotModel* model) -> const std::string&;

int GetJointCount(const PR2RobotModel* model);
auto GetPlanningJoints(const PR2RobotModel* model)
    -> const std::vector<std::string>&;

int GetJointVariableCount(const PR2RobotModel* model);
auto GetPlanningJointVariables(const PR2RobotModel* model)
    -> const std::vector<std::string>&;

int GetRedundantVariableCount(const PR2RobotModel* model);
int GetRedundantVariableIndex(const PR2RobotModel* model, int vidx);

bool HasPosLimit(const PR2RobotModel* model, int jidx);
bool IsContinuous(const PR2RobotModel* model, int jidx);
double GetMinPosLimit(const PR2RobotModel* model, int jidx);
double GetMaxPosLimit(const PR2RobotModel* model, int jidx);
double GetVelLimit(const PR2RobotModel* model, int jidx);
double GetAccLimit(const PR2RobotModel* model, int jidx);

void SetReferenceState(PR2RobotModel* model, const double* positions);

bool CheckJointLimits(
    PR2RobotModel* model,
    const smpl::RobotState& state,
    bool verbose = false);

auto ComputeFK(PR2RobotModel* model, const smpl::RobotState& state)
    -> smpl::Affine3;

void PrintRobotModelInformation(const PR2RobotModel* model);

bool ComputeIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option = ik_option::UNRESTRICTED);

bool ComputeIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option = ik_option::UNRESTRICTED);

bool ComputeFastIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution);

auto GetExtension(PR2RobotModel* model, size_t class_code) -> Extension*;

class PR2RobotModel :
    public virtual RobotModel,
    public IForwardKinematics,
    public IInverseKinematics,
    public IRedundantManipulator
{
public:

    /// \name RobotModel Interface
    ///@{
    auto minPosLimit(int jidx) const -> double final;
    auto maxPosLimit(int jidx) const -> double final;
    bool hasPosLimit(int jidx) const final;
    bool isContinuous(int jidx) const final;
    auto velLimit(int jidx) const -> double final;
    auto accLimit(int jidx) const -> double final;
    bool checkJointLimits(const smpl::RobotState& state, bool verbose = false) final;
    ///@}

    /// \name IForwardKinematics Interface
    ///@{
    auto computeFK(const smpl::RobotState& state) -> smpl::Affine3 final;
    ///@}

    /// \name IInverseKinematics Interface
    ///@{
    bool computeIK(
        const smpl::Affine3& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) final;

    bool computeIK(
        const smpl::Affine3& pose,
        const RobotState& start,
        std::vector<RobotState>& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED) final;
    ///@}

    /// \name IRedundantManipulator Interface
    /// @{
    const int redundantVariableCount() const final;
    const int redundantVariableIndex(int vidx) const final;
    bool computeFastIK(
        const smpl::Affine3& pose,
        const RobotState& start,
        RobotState& solution) final;
    /// @}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

public:

    KDLRobotModel kdl_model;
    int free_angle;

    std::unique_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_ik_solver;

    std::unique_ptr<RPYSolver> rpy_solver;

    const urdf::Link*           forearm_roll_link;
    const urdf::JointVariable*  wrist_pitch_joint;
    const urdf::Link*           end_effector_link;
};

} // namespace smpl

#endif
