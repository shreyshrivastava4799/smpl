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

#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/stl/memory.h>
#include <smpl/time.h>
#include <urdf/model.h>

namespace smpl {

static
bool getCount(int& count, int max_count, int min_count)
{
    if (count > 0) {
        if (-count >= min_count) {
            count = -count;
            return true;
        } else if (count + 1 <= max_count) {
            count = count + 1;
            return true;
        } else {
            return false;
        }
    } else {
        if (1 - count <= max_count) {
            count = 1 - count;
            return true;
        } else if (count - 1 >= min_count) {
            count = count - 1;
            return true;
        } else {
            return false;
        }
    }
}

static
void NormalizeAngles(const KDLRobotModel* model, KDL::JntArray* q)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        if (model->urdf_model.vprops[i].continuous) {
            (*q)(i) = smpl::angles::normalize_angle((*q)(i));
        }
    }
}

static
auto GetSolverMinPosition(const KDLRobotModel* model, int vidx) -> double
{
    if (model->urdf_model.vprops[vidx].continuous) {
        return -M_PI;
    } else {
        return model->urdf_model.vprops[vidx].min_position;
    }
}

bool InitKDLRobotModel(
    KDLRobotModel* model,
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    ROS_DEBUG("Initialize KDL Robot Model");

    ROS_DEBUG("Initialize URDF from string");
    ::urdf::Model urdf;
    if (!urdf.initString(robot_description)) {
        ROS_ERROR("Failed to parse the URDF.");
        return false;
    }

    return InitKDLRobotModel(model, &urdf, base_link, tip_link, free_angle);
}

bool InitKDLRobotModel(
    KDLRobotModel* model,
    const ::urdf::ModelInterface* urdf,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    ROS_DEBUG("Initialize Robot Model");
    auto j_world = urdf::JointSpec();
    j_world.name = "map";
    j_world.origin = smpl::Affine3::Identity(); // IMPORTANT
    j_world.axis = smpl::Vector3::Zero();
    j_world.type = urdf::JointType::Floating;
    if (!urdf::InitRobotModel(&model->robot_model, urdf, &j_world)) {
        ROS_ERROR("Failed to initialize Robot Model");
        return false;
    }

    model->base_link = base_link;
    model->tip_link = tip_link;
    model->kinematics_link = GetLink(&model->robot_model, &base_link);
    if (model->kinematics_link == NULL) {
        return false; // this shouldn't happen if the chain initialized successfully
    }

    ROS_DEBUG("Initialize KDL tree");
    if (!kdl_parser::treeFromUrdfModel(*urdf, model->tree)) {
        ROS_ERROR("Failed to parse the kdl tree from robot description.");
        return false;
    }

    ROS_DEBUG("Initialize KDL chain (%s, %s)", base_link.c_str(), tip_link.c_str());
    if (!model->tree.getChain(base_link, tip_link, model->chain)) {
        ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", base_link.c_str(), tip_link.c_str());
        return false;
    }

    ROS_DEBUG("Gather joints in chain");
    auto planning_joints = std::vector<std::string>();
    for (auto i = (unsigned)0; i < model->chain.getNrOfSegments(); ++i) {
        auto& segment = model->chain.getSegment(i);
        auto& child_joint_name = segment.getJoint().getName();
        auto* joint = GetJoint(&model->robot_model, &child_joint_name);
        if (GetVariableCount(joint) > 1) {
            ROS_WARN("> 1 variable per joint");
            return false;
        }
        if (GetVariableCount(joint) == 0) {
            continue;
        }
        planning_joints.push_back(child_joint_name);
    }

    ROS_DEBUG("Initialize URDF Robot Model with planning joints = %s", to_string(planning_joints).c_str());
    if (!urdf::Init(&model->urdf_model, &model->robot_model, &planning_joints)) {
        ROS_ERROR("Failed to initialize URDF Robot Model");
        return false;
    }

    model->setPlanningJoints(planning_joints);

    ROS_DEBUG("Initialize planning link");
    // do this after we've initialized the URDFRobotModel...
    model->urdf_model.planning_link = GetLink(&model->robot_model, &tip_link);
    if (model->urdf_model.planning_link == NULL) {
        return false; // this shouldn't happen either
    }

    ROS_DEBUG("Initialize FK Position solver");
    model->fk_solver = make_unique<KDL::ChainFkSolverPos_recursive>(model->chain);

    ROS_DEBUG("Initialize IK Velocity solver");
    model->ik_vel_solver = make_unique<KDL::ChainIkSolverVel_pinv>(model->chain);

    ROS_DEBUG("Initialize IK Position solver");
    auto q_min = KDL::JntArray(model->jointVariableCount());
    auto q_max = KDL::JntArray(model->jointVariableCount());
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        if (model->urdf_model.vprops[i].continuous) {
            q_min(i) = -M_PI;
            q_max(i) = M_PI;
        } else {
            q_min(i) = model->urdf_model.vprops[i].min_position;
            q_max(i) = model->urdf_model.vprops[i].max_position;
        }
    }

    model->max_iterations = 200;
    model->kdl_eps = 0.001;
    model->ik_solver = make_unique<KDL::ChainIkSolverPos_NR_JL>(
            model->chain,
            q_min,
            q_max,
            *model->fk_solver,
            *model->ik_vel_solver,
            model->max_iterations,
            model->kdl_eps);

    ROS_DEBUG("Initialize IK search parameters");
    model->jnt_pos_in.resize(model->chain.getNrOfJoints());
    model->jnt_pos_out.resize(model->chain.getNrOfJoints());
    if (free_angle == -1) {
        free_angle = 2;
    }
    model->free_angle = free_angle;
    model->search_discretization = 0.02;
    model->timeout = 0.005;
}

auto GetRobotModel(const KDLRobotModel* model) -> const urdf::RobotModel*
{
    return &model->robot_model;
}

auto GetBaseLink(const KDLRobotModel* model) -> const std::string&
{
    return model->base_link;
}

auto GetPlanningLink(const KDLRobotModel* model) -> const std::string&
{
    return model->tip_link;
}

int GetJointCount(const KDLRobotModel* model)
{
    return model->urdf_model.getPlanningJoints().size();
}

auto GetPlanningJoints(const KDLRobotModel* model) -> const std::vector<std::string>&
{
    return model->getPlanningJoints();
}

int GetJointVariableCount(const KDLRobotModel* model)
{
    return GetJointCount(model);
}

auto GetPlanningJointVariables(const KDLRobotModel* model) -> const std::vector<std::string>&
{
    return model->getPlanningJoints();
}

int GetRedundantVariableCount(const KDLRobotModel* model)
{
    return 0;
}

int GetRedundantVariableIndex(const KDLRobotModel* model, int vidx)
{
    return 0.0;
}

bool HasPosLimit(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].bounded;
}

bool IsContinuous(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].continuous;
}

double GetMinPosLimit(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].min_position;
}

double GetMaxPosLimit(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].max_position;
}

double GetVelLimit(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].vel_limit;
}

double GetAccLimit(const KDLRobotModel* model, int vidx)
{
    return model->urdf_model.vprops[vidx].acc_limit;
}

void SetReferenceState(KDLRobotModel* model, const double* positions)
{
    SetReferenceState(&model->urdf_model, positions);
}

bool CheckJointLimits(
    KDLRobotModel* model,
    const smpl::RobotState& state,
    bool verbose)
{
    return model->urdf_model.checkJointLimits(state, verbose);
}

auto ComputeFK(KDLRobotModel* model, const smpl::RobotState& state)
    -> smpl::Affine3
{
    return model->urdf_model.computeFK(state);
}

void PrintRobotModelInformation(const KDLRobotModel* model)
{
    leatherman::printKDLChain(model->chain, "robot_model");
}

bool ComputeIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    if (option != ik_option::UNRESTRICTED) {
        return false;
    }

    // transform into kinematics and convert to kdl
    auto* T_map_kinematics = GetUpdatedLinkTransform(
            &model->urdf_model.robot_state, model->kinematics_link);
    auto frame_des = KDL::Frame();
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (auto i = 0; i < start.size(); i++) {
        model->jnt_pos_in(i) = start[i];
    }

    // must be normalized for CartToJnt
    NormalizeAngles(model, &model->jnt_pos_in);

    auto initial_guess = model->jnt_pos_in(model->free_angle);

    auto start_time = smpl::clock::now();
    auto loop_time = 0.0;
    auto count = 0;

    auto num_positive_increments =
            (int)((GetSolverMinPosition(model, model->free_angle) - initial_guess) /
                    model->search_discretization);
    auto num_negative_increments =
            (int)((initial_guess - GetSolverMinPosition(model, model->free_angle)) /
                    model->search_discretization);

    while (loop_time < model->timeout) {
        if (model->ik_solver->CartToJnt(
                model->jnt_pos_in, frame_des, model->jnt_pos_out) >= 0)
        {
            NormalizeAngles(model, &model->jnt_pos_out);
            solution.resize(start.size());
            for (auto i = 0; i < solution.size(); ++i) {
                solution[i] = model->jnt_pos_out(i);
            }

            return true;
        }

        if (!getCount(count, num_positive_increments, -num_negative_increments)) {
            return false;
        }
        model->jnt_pos_in(model->free_angle) = initial_guess + model->search_discretization * count;
        ROS_DEBUG("%d, %f", count, model->jnt_pos_in(model->free_angle));
        loop_time = to_seconds(smpl::clock::now() - start_time);
    }

    return false;
}

bool ComputeIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    // NOTE: only returns one solution
    auto solution = RobotState();
    if (ComputeIK(model, pose, start, solution)) {
        solutions.push_back(solution);
    }
    return solutions.size() > 0;
}

bool ComputeFastIK(
    KDLRobotModel* model,
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution)
{
    // transform into kinematics frame and convert to kdl
    auto* T_map_kinematics = GetUpdatedLinkTransform(
            &model->urdf_model.robot_state, model->kinematics_link);
    KDL::Frame frame_des;
    tf::transformEigenToKDL(T_map_kinematics->inverse() * pose, frame_des);

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        model->jnt_pos_in(i) = start[i];
    }

    // must be normalized for CartToJntSearch
    NormalizeAngles(model, &model->jnt_pos_in);

    if (model->ik_solver->CartToJnt(
            model->jnt_pos_in, frame_des, model->jnt_pos_out) < 0)
    {
        return false;
    }

    NormalizeAngles(model, &model->jnt_pos_out);

    solution.resize(start.size());
    for (size_t i = 0; i < solution.size(); ++i) {
        solution[i] = model->jnt_pos_out(i);
    }

    return true;
}

auto GetExtension(KDLRobotModel* model, size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<IForwardKinematics>()) return model;
    if (class_code == GetClassCode<IInverseKinematics>()) return model;
    if (class_code == GetClassCode<IRedundantManipulator>()) return model;
    return NULL;
}

auto KDLRobotModel::minPosLimit(int jidx) const -> double
{
    return ::smpl::GetMinPosLimit(this, jidx);
}

auto KDLRobotModel::maxPosLimit(int jidx) const -> double
{
    return ::smpl::GetMaxPosLimit(this, jidx);
}

bool KDLRobotModel::hasPosLimit(int jidx) const
{
    return ::smpl::HasPosLimit(this, jidx);
}

bool KDLRobotModel::isContinuous(int jidx) const
{
    return ::smpl::IsContinuous(this, jidx);
}

auto KDLRobotModel::velLimit(int jidx) const -> double
{
    return ::smpl::GetVelLimit(this, jidx);
}

auto KDLRobotModel::accLimit(int jidx) const -> double
{
    return ::smpl::GetAccLimit(this, jidx);
}

bool KDLRobotModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    return ::smpl::CheckJointLimits(this, state, verbose);
}

auto KDLRobotModel::computeFK(const smpl::RobotState& state) -> smpl::Affine3
{
    return ::smpl::ComputeFK(this, state);
}

bool KDLRobotModel::computeIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution,
    ik_option::IkOption option)
{
    return ::smpl::ComputeIK(this, pose, start, solution, option);
}

bool KDLRobotModel::computeIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    std::vector<RobotState>& solutions,
    ik_option::IkOption option)
{
    return ::smpl::ComputeIK(this, pose, start, solutions, option);
}

const int KDLRobotModel::redundantVariableCount() const
{
    return ::smpl::GetRedundantVariableCount(this);
}

const int KDLRobotModel::redundantVariableIndex(int vidx) const
{
    return ::smpl::GetRedundantVariableIndex(this, vidx);
}

bool KDLRobotModel::computeFastIK(
    const smpl::Affine3& pose,
    const RobotState& start,
    RobotState& solution)
{
    return ::smpl::ComputeFastIK(this, pose, start, solution);
}

auto KDLRobotModel::GetExtension(size_t class_code) -> Extension*
{
    return ::smpl::GetExtension(this, class_code);
}

} // namespace smpl
