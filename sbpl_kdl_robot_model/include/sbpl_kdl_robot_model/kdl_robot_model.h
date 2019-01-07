////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#ifndef SBPL_KDL_ROBOT_MODEL_KDL_ROBOT_MODEL_H
#define SBPL_KDL_ROBOT_MODEL_KDL_ROBOT_MODEL_H

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <smpl/robot_model.h>
#include <smpl_urdf_robot_model/smpl_urdf_robot_model.h>

namespace urdf {
class ModelInterface;
} // namespace urdf

namespace smpl {

class KDLRobotModel;

bool InitKDLRobotModel(
    KDLRobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle = -1);

bool InitKDLRobotModel(
    KDLRobotModel* model,
    const ::urdf::ModelInterface* urdf,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle = -1);

auto GetRobotModel(const KDLRobotModel* model) -> const urdf::RobotModel*;

auto GetBaseLink(const KDLRobotModel* model) -> const std::string&;
auto GetPlanningLink(const KDLRobotModel* model) -> const std::string&;

int GetJointCount(const KDLRobotModel* model);
auto GetPlanningJoints(const KDLRobotModel* model)
    -> const std::vector<std::string>&;

int GetJointVariableCount(const KDLRobotModel* model);
auto GetPlanningJointVariables(const KDLRobotModel* model)
    -> const std::vector<std::string>&;

int GetRedundantVariableCount(const KDLRobotModel* model);
int GetRedundantVariableIndex(const KDLRobotModel* model, int vidx);

bool HasPosLimit(const KDLRobotModel* model, int jidx);
bool IsContinuous(const KDLRobotModel* model, int jidx);
double GetMinPosLimit(const KDLRobotModel* model, int jidx);
double GetMaxPosLimit(const KDLRobotModel* model, int jidx);
double GetVelLimit(const KDLRobotModel* model, int jidx);
double GetAccLimit(const KDLRobotModel* model, int jidx);

void SetReferenceState(KDLRobotModel* model, const double* positions);

bool CheckJointLimits(
    KDLRobotModel* model,
    const smpl::RobotState& state,
    bool verbose = false);

auto ComputeFK(KDLRobotModel* model, const smpl::RobotState& state) -> smpl::Affine3;

void PrintRobotModelInformation(const KDLRobotModel* model);

bool ComputeIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option = ik_option::UNRESTRICTED);

bool ComputeIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option = ik_option::UNRESTRICTED);

bool ComputeFastIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution);

auto GetExtension(KDLRobotModel* model, size_t class_code) -> Extension*;

class KDLRobotModel :
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

    urdf::URDFRobotModel urdf_model;

    urdf::RobotModel robot_model;

    const urdf::Link* kinematics_link = NULL;

    std::string base_link;
    std::string tip_link;

    KDL::Tree tree;
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive>    fk_solver;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv>         ik_vel_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>        ik_solver;

    // ik solver settings
    int     max_iterations;
    double  kdl_eps;

    // temporary storage
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;

    // ik search configuration
    int     free_angle;
    double  search_discretization;
    double  timeout;
};

} // namespace smpl

#endif
