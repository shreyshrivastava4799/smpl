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

#ifndef SMPL_ROBOT_MODEL_H
#define SMPL_ROBOT_MODEL_H

// standard includes
#include <ostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// project includes
#include <smpl/extension.h>
#include <smpl/spatial.h>
#include <smpl/types.h>
#include <smpl/debug/marker.h>

namespace smpl {

/// \brief The root interface defining the basic requirements for a robot model
class RobotModel : public Extension
{
public:

    virtual ~RobotModel();

    /// \brief Return the lower position limit for a joint.
    virtual double MinPosLimit(int jidx) const = 0;

    /// \brief Return the upper position limit for a joint.
    virtual double MaxPosLimit(int jidx) const = 0;

    /// \brief Return whether a joint has position limits.
    virtual bool HasPosLimit(int jidx) const = 0;

    /// \brief Return whether the variable has topology SO(2).
    virtual bool IsContinuous(int jidx) const = 0;

    /// \brief Return the velocity limit for a joint with 0 = unlimited
    virtual double VelLimit(int jidx) const = 0;

    /// \brief Return the acceleration limit for a joint with 0 = unlimited
    virtual double AccLimit(int jidx) const = 0;

    /// \brief Check a state for joint limit violations.
    virtual bool CheckJointLimits(const RobotState& state, bool verbose = false) = 0;

    size_t JointCount() const { return planning_joints_.size(); }
    size_t JointVariableCount() const { return planning_joints_.size(); }

    void SetPlanningJoints(const std::vector<std::string>& joints);
    auto GetPlanningJoints() const -> const std::vector<std::string>&;

    virtual auto GetVisualization(const RobotState& state)
        -> std::vector<visual::Marker>;

protected:

    std::vector<std::string> planning_joints_;
};

/// \brief RobotModel extension for providing forward kinematics
class IForwardKinematics : public virtual RobotModel
{
public:

    virtual ~IForwardKinematics();

    /// \brief Compute forward kinematics of the planning link.
    ///
    /// The output pose, stored in \p pose, should be of the format
    /// { x, y, z, R, P, Y } of the planning link
    ///
    /// \return true if forward kinematics were computed; false otherwise
    virtual Affine3 ComputeFK(const RobotState& state) = 0;
};

namespace ik_option {

enum IkOption
{
    UNRESTRICTED = 0,
    RESTRICT_XYZ = 1,
    RESTRICT_RPY = 2
};

std::ostream& operator<<(std::ostream& o, IkOption option);
auto to_cstring(IkOption option) -> const char*;

} // namespace ik_option

/// \brief RobotModel extension for providing inverse kinematics
class IInverseKinematics : public virtual RobotModel
{
public:

    virtual ~IInverseKinematics();

    /// \brief Compute an inverse kinematics solution.
    virtual bool ComputeIK(
        const Affine3& pose,
        const RobotState& start,
        RobotState& solution,
        ik_option::IkOption option = ik_option::UNRESTRICTED) = 0;

    /// \brief Compute multiple inverse kinematic solutions.
    virtual bool ComputeIK(
        const Affine3& pose,
        const RobotState& start,
        std::vector<RobotState>& solutions,
        ik_option::IkOption option = ik_option::UNRESTRICTED) = 0;
};

class IRedundantManipulator : public virtual RobotModel
{
public:

    /// \brief Return the number of redundant joint variables.
    virtual const int RedundantVariableCount() const = 0;

    /// \brief Return the index (within planning joints) of the n'th redundant
    ///     variable.
    virtual const int RedundantVariableIndex(int rvidx) const = 0;

    /// \brief Compute an inverse kinematics solution while restricting all
    ///     redundant joint variables to the seed state.
    virtual bool ComputeFastIK(
        const Affine3& pose,
        const RobotState& start,
        RobotState& solution) = 0;
};

/// \brief Convenience class allowing a component to implement all root
///     interface methods via an existing extension
class RobotModelChild : public virtual RobotModel
{
public:

    RobotModelChild(RobotModel* parent) : m_parent(parent) { }

    RobotModel* Parent() const { return m_parent; }

    double MinPosLimit(int jidx) const { return m_parent->MinPosLimit(jidx); }
    double MaxPosLimit(int jidx) const { return m_parent->MaxPosLimit(jidx); }
    bool HasPosLimit(int jidx) const { return m_parent->HasPosLimit(jidx); }
    double VelLimit(int jidx) const { return m_parent->VelLimit(jidx); }
    double AccLimit(int jidx) const { return m_parent->AccLimit(jidx); }
    bool CheckJointLimits(const RobotState& state, bool verbose = false) {
        return m_parent->CheckJointLimits(state, verbose);
    }

private:

    RobotModel* m_parent;
};

} // namespace smpl

#endif
