////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Gokul Subramanian, Benjamin Cohen, Andrew Dornbush
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

/// \author Gokul Subramanian, MSE in Robotics & CIS 2011
/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef SBPL_PR2_ROBOT_MODEL_ORIENTATION_SOLVER_H
#define SBPL_PR2_ROBOT_MODEL_ORIENTATION_SOLVER_H

// standard includes
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

namespace smpl {

class RPYSolver
{
  public:

    RPYSolver(double wrist_pitch_min_limit, double wrist_pitch_max_limit);

    /// \param rpy
    /// \param start Joint variable vector of the start
    /// \param forearm_roll_link_pose { x, y, z, r, p, y } of the forearm
    /// \param endeff_link_pose { x, y, z, r, p, y } of the end effector
    /// \param solution_num 1 or 2 to choose the forearm roll orientation
    /// \param solution Joint variable vector of the solution
    bool computeRPYOnly(
        const std::vector<double>& rpy,
        const std::vector<double>& start,
        const std::vector<double>& forearm_roll_link_pose,
        const std::vector<double>& endeff_link_pose,
        int solution_num,
        std::vector<double>& solution) const;

  private:

    double wrist_pitch_min_limit_;
    double wrist_pitch_max_limit_;

    /// \brief Calculate the angles required for the pr2 wrist to achieve a
    ///     desired orientation
    ///
    /// Function calculates the forearm roll, wrist pitch and wrist roll for the
    /// PR2 robot hand which is required to attain a certain yaw, pitch and roll
    /// configuration in the world frame, starting at a certain yaw, pitch and
    /// roll configuration. Do not use for other robots unless the joints are of
    /// the same nature and the reference frame conventions are the same.
    ///
    /// \param[out] output { success, theta_1, theta_2, theta_3 }
    /// \param phi yaw of the forearm
    /// \param theta pitch of the forearm
    /// \param psi roll of the forearm
    /// \param yaw1 yaw of the end effector
    /// \param pitch1 pitch of the end effector
    /// \param roll1 roll of the end effector
    /// \param yaw2 desired yaw of the end effector
    /// \param pitch2 desired pitch of the end effector
    /// \param roll2 desired pitch of the end effector
    /// \param attempt 1 or 2 to choose the forearm roll orientation
    void orientationSolver(
        double* output,
        double phi, double theta, double psi,
        double yaw1, double pitch1, double roll1,
        double yaw2, double pitch2, double roll2,
        int attempt) const;
};

} // namespace smpl

#endif

