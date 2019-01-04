////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_MANIP_LATTICE_H
#define SMPL_MANIP_LATTICE_H

// standard includes
#include <stddef.h>
#include <string>
#include <vector>

// project includes
#include <smpl/types.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/debug/marker.h>
#include <smpl/graph/manip_lattice_types.h>

namespace smpl {

class ActionSpace;
class CostFunction;
class CollisionChecker;
class RobotModel;
class LazyCostFunction;
class IForwardKinematics;

/// \class Discrete space constructed by expliciting discretizing each joint
class ManipLattice :
    public DiscreteSpace,
    public RobotPlanningSpace,
    public IExtractRobotState,
    public ISearchable,
    private ILazySearchable,
    private IProjectToPose
{
public:

    ~ManipLattice();

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions,
        CostFunction* cost_fun);

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions,
        CostFunction* cost_fun,
        LazyCostFunction* lazy_cost_fun);

    auto GetResolutions() const -> const std::vector<double>&;

    auto GetActionSpace() -> ActionSpace*;
    auto GetActionSpace() const -> const ActionSpace*;

    auto GetCostFunction() -> CostFunction*;
    auto GetCostFunction() const -> const CostFunction*;

    auto GetVisualizationFrameId() const -> const std::string&;
    void SetVisualizationFrameId(const std::string& frame_id);

    auto GetDiscreteCenter(const RobotState& state) const -> RobotState;
    auto GetDiscreteState(const RobotState& state) const -> RobotCoord;

    void CoordToState(const RobotCoord& coord, RobotState& state) const;
    void StateToCoord(const RobotState& state, RobotCoord& coord) const;

    int ReserveHashEntry();
    int GetOrCreateState(const RobotCoord& coord, const RobotState& state);
    int GetHashEntry(const RobotCoord& coord);
    int CreateHashEntry(const RobotCoord& coord, const RobotState& state);
    auto GetHashEntry(int state_id) const -> ManipLatticeState*;

    void ClearStates();

    bool IsActionWithinBounds(const RobotState& state, const Action& action);

    auto FindBestAction(int state_id, int succ_id) -> ManipLatticeAction;

    auto GetStateVisualization(const RobotState& vars, const std::string& ns)
        -> std::vector<visual::Marker>;

    void PrintState(int state_id, bool verbose, FILE* fout = NULL);

    /// \name IExtractRobotState Interface
    ///@{
    auto ExtractState(int state_id) -> const RobotState& final;
    ///@}

    /// \name ISearchable Interface
    ///@{
    void GetSuccs(int state_id, std::vector<int>* succs, std::vector<int>* costs) final;
    ///@}

    /// \name RobotPlanningSpace Interface
    ///@{
    int GetStateID(const RobotState& state) final;
    bool ExtractPath(const std::vector<int>& ids, std::vector<RobotState>& path) final;
    ///@}

    /// \name DiscreteSpace Interface
    ///@{
    bool UpdateHeuristics(Heuristic** heuristics, int count) final;
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;
    ///@}

    /// \name Extension Interface
    ///@{
    using Extension::GetExtension;
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

    ActionSpace* m_action_space = NULL;
    CostFunction* m_cost_fun = NULL;
    LazyCostFunction* m_lazy_cost_fun = NULL;

    IForwardKinematics* m_fk_iface = NULL;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool>   m_continuous;
    std::vector<bool>   m_bounded;

    std::vector<int>    m_coord_vals;
    std::vector<double> m_coord_deltas;

    // maps from coords to stateID
    using StateKey = ManipLatticeState;
    using StateHash = PointerValueHash<StateKey>;
    using StateEqual = PointerValueEqual<StateKey>;
    using StateToIDTable = hash_map<StateKey*, int, StateHash, StateEqual>;

    StateToIDTable m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    std::string m_viz_frame_id;

private:

    /// \name ILazySearchable Interface
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) final;
    int GetTrueCost(int state_id, int succ_state_id) final;
    ///@}

    /// \name IProjectToPose Interface
    ///@{
    auto ProjectToPose(int state_id) -> Affine3 final;
    ///@}
};

} // namespace smpl

#endif
