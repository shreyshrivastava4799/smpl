////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_MANIP_LATTICE_ACTION_SPACE_H
#define SMPL_MANIP_LATTICE_ACTION_SPACE_H

// standard includes
#include <string>
#include <vector>

// project includes
#include <smpl/graph/action_space.h>
#include <smpl/graph/motion_primitive.h>

namespace smpl {

class ManipLattice;
class Heuristic;
class IForwardKinematics;
class IInverseKinematics;
class IMetricGoalHeuristic;
class IMetricStartHeuristic;
class IGetPose;
class IGetRobotState;

// The following actions are available from each state:
//
// (1) Short, atomic motion primitives specified as an offset from the source
//     state; applied when near the start or the goal
// (2) Long, atomic motion primitives specified as an offset from the state;
//     applied when far from the start or the goal
// (3) An adaptively-generated motion that linearly interpolates to the goal
//     state, when seeking a joint-space goal
// (4) An adaptively-generated motion that linearly interpolates to an IK
//     solution that satisfies a 6-DOF pose constraint of the goal state.
// (5) An adaptively-generated motion that linearly interpolates to an IK
//     solution that satisfies a 3-DOF position constraint of the goal state.
// (6) An adaptively-generated motion that linearly interpolates to an IK
//     solution that satisfies a 3-DOF orientation constraint of the goal state.
//
// For (4), (5), and (6), the IK solver may return more than one solution
// and the action space may return a motion that interpolates to each one.
//
// For (4), (5), and (6), the IK solver will only be called if the source state
// is within some distance to the goal.

// Either use only (1) or (1) and (2)
//
// For ... , distances are defined in terms of task-space distance.
class ManipulationActionSpace : public ActionSpace
{
public:

    bool Init(ManipLattice* space);

    // Load short- and long- distance motion primitives from file
    bool Load(const std::string& action_filename);

    // Add a short- or long-distance motion primitive.
    void AddMotionPrimitive(
        const std::vector<double>& mprim,
        bool short_dist_mprim,
        bool add_converse = true);

    int GetNumShortMotions() const;
    int GetNumLongMotions() const;

    bool IsMotionTypeEnabled(MotionPrimitive::Type type) const;
    bool AreLongMotionsEnabled() const;
    bool IsInterpMotionEnabled() const;
    bool IsIKMotionXYZRPYEnabled() const;
    bool IsIKMotionXYZEnabled() const;
    bool IsIKMotionRPYEnabled() const;
    bool IsMultipleIKSolutionsEnabled() const;

    auto GetMotionTypeThreshold(MotionPrimitive::Type type) const -> double;
    auto GetLongMotionThreshold() const -> double;
    auto GetInterpMotionThreshold() const -> double;
    auto GetIKMotionXYZRPYThreshold() const -> double;
    auto GetIKMotionXYZThreshold() const -> double;
    auto GetIKMotionRPYThreshold() const -> double;

    void EnableLongMotions(bool enable);
    void EnableInterpMotion(bool enable);
    void EnableIKMotionXYZRPY(bool enable);
    void EnableIKMotionXYZ(bool enable);
    void EnableIKMotionRPY(bool enable);
    void EnableMultipleIKSolutions(bool enable);

    void SetLongMotionThreshold(double thresh);
    void SetInterpMotionThreshold(double thresh);
    void SetIKMotionXYZRPYThreshold(double thresh);
    void SetIKMotionXYZThreshold(double thresh);
    void SetIKMotionRPYThreshold(double thresh);

    // Remove all long and short motion primitives, and disable all adaptively-
    // generated motions.
    void Clear();

    /// \name ActionSpace Interface
    ///@{
    bool UpdateHeuristics(Heuristic** heuristics, int count) final;
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;

    auto Apply(int state_id, ActionArray store = ActionArray()) -> ActionArray final;
    ///@}

public:

    IForwardKinematics* m_fk_iface = NULL;
    IInverseKinematics* m_ik_iface = NULL;

    IMetricStartHeuristic*  m_start_heuristic = NULL;
    IMetricGoalHeuristic*   m_goal_heuristic = NULL;

    IGetPose*       m_get_goal_pose = NULL;
    IGetRobotState* m_get_goal_state = NULL;

    std::vector<MotionPrimitive> m_short_dist_mprims;
    std::vector<MotionPrimitive> m_long_dist_mprims;

    bool m_mprim_enabled[MotionPrimitive::NUMBER_OF_MPRIM_TYPES] = { };
    double m_mprim_thresh[MotionPrimitive::NUMBER_OF_MPRIM_TYPES] = { };

    bool m_use_multiple_ik_solutions        = false;
};

} // namespace smpl

#endif

