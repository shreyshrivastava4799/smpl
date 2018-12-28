////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_WORKSPACE_LATTICE_H
#define SMPL_WORKSPACE_LATTICE_H

// standard includes
#include <chrono>
#include <vector>

// system includes
#include <sbpl/headers.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/robot_model.h>
#include <smpl/time.h>
#include <smpl/types.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/workspace_lattice_base.h>
#include <smpl/graph/workspace_lattice_types.h>

namespace smpl {

struct WorkspaceLatticeActionSpace;

/// \class Discrete state lattice representation representing a robot as the
///     pose of one of its links and all redundant joint variables
class WorkspaceLattice :
    public DiscreteSpace,
    public RobotPlanningSpace,
    public IExtractRobotState,
    public IProjectToPose,
    public ISearchable,
    private ILazySearchable
{
public:

    ~WorkspaceLattice();

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const WorkspaceLatticeBase::Params& params,
        WorkspaceLatticeActionSpace* actions);

    auto GetActionSpace() -> WorkspaceLatticeActionSpace*;
    auto GetActionSpace() const -> const WorkspaceLatticeActionSpace*;

    auto GetVisualizationFrameId() const -> const std::string&;
    void SetVisualizationFrameId(const std::string& frame_id);

    int ReserveHashEntry();
    int GetOrCreateState(const WorkspaceCoord& coord);
    auto GetState(int state_id) const -> WorkspaceLatticeState*;

    bool CheckAction(
        const RobotState& state,
        const WorkspaceAction& action,
        RobotState* final_rstate = NULL);

    int ComputeCost(
        const WorkspaceLatticeState& src,
        const WorkspaceLatticeState& dst);

    bool CheckLazyAction(
        const RobotState& state,
        const WorkspaceAction& action,
        RobotState* final_rstate = NULL);

    auto GetStateVisualization(const RobotState& state, const std::string& ns)
        -> std::vector<visual::Marker>;

    void PrintState(int state_id, bool verbose, FILE* fout = NULL);

    /// \name IExtractRobotState Interface
    ///@{
    auto ExtractState(int state_id) -> const RobotState& final;
    ///@}

    /// \name IProjectToPose Interface
    ///@{
    auto ProjectToPose(int state_id) -> Affine3 final;
    ///@}

    /// \name RobotPlanningSpace Interface
    ///@{
    int GetStateID(const RobotState& state) final;
    bool ExtractPath(
        const std::vector<int>& state_ids,
        std::vector<RobotState>& path) final;
    ///@}

    /// \name ISearchable Interface
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) final;
    ///@}

    /// \name ILazySearchable Interface
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) final;
    int GetTrueCost(int state_id, int succ_state_id) final;
    ///@}

    /// \name DiscreteSpace Interface
    ///@{
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

    WorkspaceLatticeBase base;

    // maps state -> id
    using StateKey = WorkspaceLatticeState;
    using StateHash = PointerValueHash<StateKey>;
    using StateEqual = PointerValueEqual<StateKey>;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps id -> state
    std::vector<WorkspaceLatticeState*> m_states;

    WorkspaceLatticeActionSpace* m_actions = NULL;

    std::string m_viz_frame_id;
};

} // namespace smpl

#endif

