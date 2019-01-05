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

#include <sbpl_pr2_robot_model/pr2_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/tree.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/stl/memory.h>

namespace smpl {

static
auto GetKDLRobotModel(PR2RobotModel* model) -> smpl::KDLRobotModel*
{
    return &model->kdl_model;
}

static
auto GetKDLRobotModel(const PR2RobotModel* model) -> const smpl::KDLRobotModel*
{
    return &model->kdl_model;
}

static
auto GetURDFRobotModel(PR2RobotModel* model) -> smpl::urdf::URDFRobotModel*
{
    return &model->kdl_model.urdf_model;
}

static
auto GetURDFRobotModel(const PR2RobotModel* model)
    -> const smpl::urdf::URDFRobotModel*
{
    return &model->kdl_model.urdf_model;
}

static
auto GetRobotModel(PR2RobotModel* model) -> smpl::urdf::RobotModel*
{
    return &model->kdl_model.robot_model;
}

static
auto GetRobotModel(const PR2RobotModel* model) -> const smpl::urdf::RobotModel*
{
    return &model->kdl_model.robot_model;
}

static
auto GetRobotState(PR2RobotModel* model) -> smpl::urdf::RobotState*
{
    return &model->kdl_model.urdf_model.robot_state;
}

static
auto GetRobotState(const PR2RobotModel* model) -> const smpl::urdf::RobotState*
{
    return &model->kdl_model.urdf_model.robot_state;
}

static
void NormalizeAngles(const PR2RobotModel* model, KDL::JntArray* q)
{
    auto* urdf_model = GetURDFRobotModel(model);
    for (auto i = 0; i < GetJointVariableCount(model); ++i) {
        if (urdf_model->vprops[i].continuous) {
            (*q)(i) = smpl::angles::normalize_angle((*q)(i));
        }
    }
}

bool InitPR2RobotModel(
    PR2RobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    ROS_INFO("Initialize URDF from string");
    ::urdf::Model urdf;
    if (!urdf.initString(robot_description)) {
        ROS_WARN("Failed to initialize URDF");
        return false;
    }

    if (free_angle < 0) {
        free_angle = 2;
    }

    ROS_INFO("Initialize KDL Robot Model");
    if (!InitKDLRobotModel(
            &model->kdl_model, &urdf, base_link, tip_link, free_angle))
    {
        ROS_WARN("Failed to initialize KDL Robot Model");
        return false;
    }

    ROS_INFO("Initialize PR2 Arm IK Solver");
    auto search_discretization_angle = 0.02;
    auto pr2_ik_solver = smpl::make_unique<pr2_arm_kinematics::PR2ArmIKSolver>(
            urdf, base_link, tip_link, search_discretization_angle, free_angle);
    if (pr2_ik_solver->active_) {
        ROS_WARN("Failed to initialize PR2 Arm IK Solver");
        return false;
    }

    auto rpy_solver = std::unique_ptr<RPYSolver>();
    const urdf::Link* forearm_roll_link = NULL;
    const urdf::JointVariable* wrist_var = NULL;
    const urdf::Link* end_effector_link = NULL;

    if (tip_link == "r_gripper_palm_link" || tip_link == "l_gripper_palm_link") {
        ROS_INFO("Initialize PR2 RPY Solver");
        std::string forearm_roll_link_name;
        std::string wrist_pitch_joint_name;
        std::string end_effector_link_name;
        if (base_link.substr(0, 1) == "r") {
            forearm_roll_link_name = "r_forearm_roll_link";
            wrist_pitch_joint_name = "r_wrist_flex_joint";
            end_effector_link_name = "r_gripper_palm_link";
        } else {
            forearm_roll_link_name = "l_forearm_roll_link";
            wrist_pitch_joint_name = "l_wrist_flex_joint";
            end_effector_link_name = "l_gripper_palm_link";
        }

        auto* robot_model = GetRobotModel(model);

        forearm_roll_link = GetLink(robot_model, &forearm_roll_link_name);
        if (forearm_roll_link == NULL) {
            ROS_WARN("Failed to initialize RPY Solver. Missing link %s", forearm_roll_link_name.c_str());
            return false;
        }

        wrist_var = GetVariable(GetRobotModel(model), &wrist_pitch_joint_name);
        if (wrist_var == NULL) {
            ROS_WARN("Failed to initialize RPY Solver. Missing joint variable %s", wrist_pitch_joint_name.c_str());
            return false;
        }

        end_effector_link = GetLink(robot_model, &end_effector_link_name);
        if (end_effector_link == NULL) {
            ROS_WARN("Failed to initialize RPY Solver. Missing link %s", end_effector_link_name.c_str());
            return false;
        }

        double wrist_min_limit;
        double wrist_max_limit;
        bool wrist_continuous;
        double wrist_vel_limit;
        double wrist_eff_limit;
        rpy_solver = smpl::make_unique<RPYSolver>(
                wrist_min_limit, wrist_max_limit);
    }

    model->rpy_solver = std::move(rpy_solver);
    model->forearm_roll_link = forearm_roll_link;
    model->wrist_pitch_joint = wrist_var;
    model->end_effector_link = end_effector_link;

    return true;
}

auto GetBaseLink(const PR2RobotModel* model) -> const std::string&
{
    return GetBaseLink(&model->kdl_model);
}

auto GetPlanningLink(const PR2RobotModel* model) -> const std::string&
{
    return GetPlanningLink(&model->kdl_model);
}

int GetJointCount(const PR2RobotModel* model)
{
    return GetJointCount(&model->kdl_model);
}

auto GetPlanningJoints(const PR2RobotModel* model)
    -> const std::vector<std::string>&
{
    return GetPlanningJoints(&model->kdl_model);
}

int GetJointVariableCount(const PR2RobotModel* model)
{
    return GetJointVariableCount(&model->kdl_model);
}

auto GetPlanningJointVariables(const PR2RobotModel* model)
    -> const std::vector<std::string>&
{
    return GetPlanningJointVariables(&model->kdl_model);
}

int GetRedundantVariableCount(const PR2RobotModel* model)
{
    return 1;
}

int GetRedundantVariableIndex(const PR2RobotModel* model, int vidx)
{
    assert(vidx == 0);
    return model->free_angle;
}

bool HasPosLimit(const PR2RobotModel* model, int vidx)
{
    return HasPosLimit(&model->kdl_model, vidx);
}

bool IsContinuous(const PR2RobotModel* model, int vidx)
{
    return IsContinuous(&model->kdl_model, vidx);
}

double GetMinPosLimit(const PR2RobotModel* model, int vidx)
{
    return GetMinPosLimit(&model->kdl_model, vidx);
}

double GetMaxPosLimit(const PR2RobotModel* model, int vidx)
{
    return GetMaxPosLimit(&model->kdl_model, vidx);
}

double GetVelLimit(const PR2RobotModel* model, int vidx)
{
    return GetVelLimit(&model->kdl_model, vidx);
}

double GetAccLimit(const PR2RobotModel* model, int vidx)
{
    return GetAccLimit(&model->kdl_model, vidx);
}

void SetReferenceState(PR2RobotModel* model, const double* positions)
{
    return SetReferenceState(&model->kdl_model, positions);
}

bool CheckJointLimits(
    PR2RobotModel* model,
    const smpl::RobotState& state,
    bool verbose)
{
    return CheckJointLimits(&model->kdl_model, state, verbose);
}

auto ComputeFK(
    PR2RobotModel* model,
    const smpl::RobotState& state)
    -> smpl::Affine3
{
    return ComputeFK(&model->kdl_model, state);
}

void PrintRobotModelInformation(const PR2RobotModel* model)
{
    return PrintRobotModelInformation(&model->kdl_model);
}

static
bool ComputeIK_RPY(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution)
{
    auto* robot_state = GetRobotState(model);

    for (auto i = 0; i < GetJointVariableCount(model); ++i) {
        auto& var_name = GetPlanningJointVariables(model)[i];
        auto* var = GetVariable(GetRobotModel(model), &var_name);
        SetVariablePosition(robot_state, var, start[i]);
    }

    // NOTE: It might be worth it to force an update to the kinematics frame
    // transform when the reference state is set, to avoid needing to update
    // it in this function. We shouldn't need to update it here since the group
    // state cannot modify this transform.
    auto* T_map_kinematics = GetUpdatedLinkTransform(
            robot_state,
            model->kdl_model.kinematics_link);
    auto T_kinematics_goal = KDL::Frame();
    tf::transformEigenToKDL(
            T_map_kinematics->inverse() * pose,
            T_kinematics_goal);

    auto* forearm_pose =
            GetUpdatedLinkTransform(robot_state, model->forearm_roll_link);
    auto* end_effector_pose =
            GetUpdatedLinkTransform(robot_state, model->end_effector_link);

    // transform to kdl to use their rpy conventions, which are matched by the
    // rpy solver
    auto fpose = KDL::Frame();
    auto epose = KDL::Frame();
    tf::transformEigenToKDL(*forearm_pose, fpose);
    tf::transformEigenToKDL(*end_effector_pose, epose);

    auto vfpose = std::vector<double>(6, 0.0);
    vfpose[0] = fpose.p.x();
    vfpose[1] = fpose.p.y();
    vfpose[2] = fpose.p.z();
    fpose.M.GetRPY(vfpose[3], vfpose[4], vfpose[5]);

    auto vepose = std::vector<double>(6, 0.0);
    vepose[0] = epose.p.x();
    vepose[1] = epose.p.y();
    vepose[2] = epose.p.z();
    epose.M.GetRPY(vepose[3], vepose[4], vepose[5]);

    auto rpy = std::vector<double>(3, 0.0);
    T_kinematics_goal.M.GetRPY(rpy[0], rpy[1], rpy[2]);
    solution.resize(start.size());
    return model->rpy_solver->computeRPYOnly(
            rpy, start, vfpose, vepose, 1, solution);
}

static
void CopyJntArrayToState(const KDL::JntArray* q_in, std::vector<double>* q_out)
{
    q_out->resize(q_in->rows());
    for (auto i = 0; i < q_out->size(); ++i) {
        (*q_out)[i] = (*q_in)(i);
    }
}

static
void CopyStateToJntArray(const std::vector<double>* q_in, KDL::JntArray* q_out)
{
    assert(q_in->size() == q_out->rows());
    for (auto i = 0; i < q_in->size(); ++i) {
        (*q_out)(i) = (*q_in)[i];
    }
}

bool ComputeIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    auto* robot_state = GetRobotState(model);
    switch (option) {
    case ik_option::RESTRICT_XYZ:
    {
        if (model->rpy_solver != NULL) {
            return ComputeIK_RPY(model, pose, start, solution);
        } else {
            return ComputeIK(&model->kdl_model, pose, start, solution, option);
        }
    }
    case ik_option::RESTRICT_RPY:
    {
        return ComputeIK(&model->kdl_model, pose, start, solution, option);
    }
    case ik_option::UNRESTRICTED:
    {
        CopyStateToJntArray(&start, &model->kdl_model.jnt_pos_in);
        NormalizeAngles(model, &model->kdl_model.jnt_pos_in);

        auto* robot_state = GetRobotState(model);
        auto* T_map_kinematics = GetUpdatedLinkTransform(
                robot_state, model->kdl_model.kinematics_link);
        auto T_kinematics_goal = KDL::Frame();
        tf::transformEigenToKDL(
                T_map_kinematics->inverse() * pose, T_kinematics_goal);

        auto timeout = 0.2;
        auto consistency_limit = 2.0 * M_PI;
        if (model->pr2_ik_solver->CartToJntSearch(
                model->kdl_model.jnt_pos_in,
                T_kinematics_goal,
                model->kdl_model.jnt_pos_out,
                timeout,
                consistency_limit) < 0)
        {
            return false;
        }

        CopyJntArrayToState(&model->kdl_model.jnt_pos_out, &solution);
        break;
    }
    }

    return false;
}

bool ComputeIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    // NOTE: only returns one solution
    auto solution = RobotState();
    if (ComputeIK(model, pose, start, solution)) {
        solutions.push_back(std::move(solution));
    }
    return solutions.size() > 0;
}

bool ComputeFastIK(
    PR2RobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution)
{
    auto* robot_state = GetRobotState(model);
    auto* T_map_kinematics = GetUpdatedLinkTransform(
            robot_state, model->kdl_model.kinematics_link);
    auto T_kinematics_goal = KDL::Frame();
    tf::transformEigenToKDL(
            T_map_kinematics->inverse() * pose, T_kinematics_goal);

    CopyStateToJntArray(&start, &model->kdl_model.jnt_pos_in);
    NormalizeAngles(model, &model->kdl_model.jnt_pos_in);

    if (model->pr2_ik_solver->CartToJnt(
            model->kdl_model.jnt_pos_in,
            T_kinematics_goal,
            model->kdl_model.jnt_pos_out) < 0)
    {
        return false;
    }

    CopyJntArrayToState(&model->kdl_model.jnt_pos_out, &solution);
    return true;
}

auto GetExtension(PR2RobotModel* model, size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<IForwardKinematics>()) return model;
    if (class_code == GetClassCode<IInverseKinematics>()) return model;
    if (class_code == GetClassCode<IRedundantManipulator>()) return model;
    return NULL;
}

///////////////////////
// Interface Methods //
///////////////////////

auto PR2RobotModel::minPosLimit(int vidx) const -> double
{
    return ::smpl::GetMinPosLimit(this, vidx);
}

auto PR2RobotModel::maxPosLimit(int vidx) const -> double
{
    return ::smpl::GetMaxPosLimit(this, vidx);
}

bool PR2RobotModel::hasPosLimit(int vidx) const
{
    return ::smpl::HasPosLimit(this, vidx);
}

bool PR2RobotModel::isContinuous(int vidx) const
{
    return ::smpl::IsContinuous(this, vidx);
}

auto PR2RobotModel::velLimit(int vidx) const -> double
{
    return ::smpl::GetVelLimit(this, vidx);
}

auto PR2RobotModel::accLimit(int vidx) const -> double
{
    return ::smpl::GetAccLimit(this, vidx);
}

bool PR2RobotModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    return ::smpl::CheckJointLimits(this, state, verbose);
}

auto PR2RobotModel::computeFK(const smpl::RobotState& state) -> smpl::Affine3
{
    return ::smpl::ComputeFK(this, state);
}

bool PR2RobotModel::computeIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    return ::smpl::ComputeIK(this, pose, start, solution, option);
}

bool PR2RobotModel::computeIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    return ::smpl::ComputeIK(this, pose, start, solutions, option);
}

const int PR2RobotModel::redundantVariableCount() const
{
    return ::smpl::GetRedundantVariableCount(this);
}

const int PR2RobotModel::redundantVariableIndex(int vidx) const
{
    return ::smpl::GetRedundantVariableIndex(this, vidx);
}

bool PR2RobotModel::computeFastIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution)
{
    return ::smpl::ComputeFastIK(this, pose, start, solution);
}

auto PR2RobotModel::GetExtension(size_t class_code) -> Extension*
{
    return ::smpl::GetExtension(this, class_code);
}

} // namespace smpl
