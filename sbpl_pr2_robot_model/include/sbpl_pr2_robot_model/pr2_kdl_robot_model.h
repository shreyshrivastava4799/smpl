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

#ifndef sbpl_manip_pr2_kdl_robot_model_h
#define sbpl_manip_pr2_kdl_robot_model_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <ros/console.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <urdf/model.h>

// project includes
#include <sbpl_pr2_robot_model/orientation_solver.h>

namespace sbpl {
namespace motion {

class PR2KDLRobotModel : public KDLRobotModel
{
public:

    PR2KDLRobotModel();

    ~PR2KDLRobotModel();

    /// \name Reimplemented Functions from KDLRobotModel
    ///@{
    bool init(
        const std::string& robot_description,
        const std::vector<std::string>& planning_joints,
        const std::string& chain_root_link,
        const std::string& chain_tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX) override;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) override;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const RobotState& start,
        RobotState& solution) override;
    ///@}

private:

    std::unique_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_ik_solver_;

    std::unique_ptr<RPYSolver> rpy_solver_;

    std::string forearm_roll_link_name_;
    std::string wrist_pitch_joint_name_;
    std::string end_effector_link_name_;
};

} // namespace motion
} // namespace sbpl

#endif
