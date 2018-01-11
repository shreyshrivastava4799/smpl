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

#include <sbpl_pr2_robot_model/ubr1_kdl_robot_model.h>
#include <ros/ros.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <kdl/tree.hpp>
#include <smpl/angles.h>
#include <eigen_conversions/eigen_kdl.h>

using namespace std;

namespace sbpl {
namespace motion {

UBR1KDLRobotModel::UBR1KDLRobotModel() :
    rpy_solver_(NULL)
{
    chain_root_name_ = "torso_lift_link";
    chain_tip_name_ = "gripper_link";
    forearm_roll_link_name_ = "forearm_roll_link";
    wrist_pitch_joint_name_ = "wrist_flex_joint";
    end_effector_link_name_ = "gripper_link";

    // initialize rpy solver
    double wrist_max_limit = -0.015, wrist_min_limit = -2.0;

//    bool wrist_continuous;
//    if (!getJointLimits(wrist_pitch_joint_name_, wrist_min_limit, wrist_max_limit, wrist_continuous)) {
//        ROS_ERROR("Failed to get wrist pitch joint limits...");
//    }
//    ROS_ERROR("Wrist limits:  {%0.3f, %0.3f}", wrist_min_limit, wrist_max_limit);

    rpy_solver_ = new RPYSolver(wrist_min_limit, wrist_max_limit);
}

UBR1KDLRobotModel::~UBR1KDLRobotModel()
{
    if (rpy_solver_) {
        delete rpy_solver_;
    }
}

bool UBR1KDLRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const RobotState& start,
    RobotState& solution,
    int option)
{
    KDL::Frame frame_des;
    tf::transformEigenToKDL(pose, frame_des);

    // transform into kinematics frame
    frame_des = T_planning_to_kinematics_ * frame_des;

    // seed configuration
    for (size_t i = 0; i < start.size(); i++) {
        // must be normalized for CartToJntSearch
        jnt_pos_in_(i) = angles::normalize_angle(start[i]);
    }

    solution.resize(start.size());

    // choose solver
    if (option == ik_option::RESTRICT_XYZ) {
        KDL::Frame fpose, epose;

        // get pose of forearm link
        if (!computeFK(start, forearm_roll_link_name_, fpose)) {
            ROS_ERROR("[rm] computeFK failed on forearm pose.");
            return false;
        }

        // get pose of end-effector link
        if (!computeFK(start, end_effector_link_name_, epose)) {
            ROS_ERROR("[rm] computeFK failed on end_eff pose.");
            return false;
        }

        std::vector<double> vfpose(6, 0.0);
        vfpose[0] = fpose.p.x();
        vfpose[1] = fpose.p.y();
        vfpose[2] = fpose.p.z();
        fpose.M.GetRPY(vfpose[3], vfpose[4], vfpose[5]);

        std::vector<double> vepose(6, 0.0);
        vepose[0] = epose.p.x();
        vepose[1] = epose.p.y();
        vepose[2] = epose.p.z();
        epose.M.GetRPY(vepose[3], vepose[4], vepose[5]);

        std::vector<double> rpy(3, 0.0);
        frame_des.M.GetRPY(rpy[0], rpy[1], rpy[2]);
        return rpy_solver_->computeRPYOnly(rpy, start, vfpose, vepose, 1, solution);
    }
    else {
        if (!computeIKSearch(pose, start, solution, 0.01)) {
            return false;
        }

        for (size_t i = 0; i < solution.size(); ++i) {
            solution[i] = jnt_pos_out_(i);
        }
    }

    return true;
}

} // namespace motion
} // namespace sbpl
