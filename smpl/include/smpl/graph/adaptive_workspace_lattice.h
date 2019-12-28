////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

#ifndef SMPL_ADAPTIVE_WORKSPACE_LATTICE_H
#define SMPL_ADAPTIVE_WORKSPACE_LATTICE_H

// standard includes
#include <functional>
#include <ostream>
#include <tuple>
#include <vector>

// project includes
#include <smpl/graph/adaptive_graph_extension.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/workspace_lattice_base.h>
#include <smpl/grid/grid.h>
#include <smpl/occupancy_grid.h>
#include <smpl/time.h>
#include <smpl/types.h>

namespace smpl {

/// Base class for adaptive states. Denotes whether a state is high dimensional.
struct AdaptiveState
{
    bool hid;
};

struct AdaptiveGridState : public AdaptiveState
{
    double x;
    double y;
    double z;
    int gx;
    int gy;
    int gz;
};

std::ostream& operator<<(std::ostream& o, const AdaptiveGridState& s);

inline
bool operator==(const AdaptiveGridState& a, const AdaptiveGridState& b)
{
    return std::tie(a.gx, a.gy, a.gz) == std::tie(b.gx, b.gy, b.gz);
}

struct AdaptiveWorkspaceState : public AdaptiveState
{
    RobotState state;
    WorkspaceCoord coord;
};

std::ostream& operator<<(std::ostream& o, const AdaptiveWorkspaceState& s);

inline
bool operator==(
    const AdaptiveWorkspaceState& a,
    const AdaptiveWorkspaceState& b)
{
    return a.coord == b.coord;
}

} // namespace smpl

// std::hash specializations for state types
namespace std {

template <>
struct hash<smpl::AdaptiveGridState>
{
    typedef smpl::AdaptiveGridState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

template <>
struct hash<smpl::AdaptiveWorkspaceState>
{
    typedef smpl::AdaptiveWorkspaceState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace smpl {

class AdaptiveWorkspaceLattice :
    public WorkspaceLatticeBase,
    public AdaptiveGraphExtension,
    public PointProjectionExtension
{
public:

    ~AdaptiveWorkspaceLattice();

    /// \name Reimplemented Public Functions from WorkspaceLatticeBase
    ///@{
    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const Params& params,
        const OccupancyGrid* grid);
    ///@}

    /// \name Required Public Functions from PointProjectionExtension
    ///@{
    bool projectToPoint(int state_id, Vector3& pos) override;
    ///@}

    /// \name Required Public Functions from AdaptiveGraphExtension
    ///@{
    bool addHighDimRegion(int state_id) override;
    bool setTunnel(const std::vector<int>& states) override;
    bool isExecutable(const std::vector<int>& states) const override;
    bool setTrackMode(const std::vector<int>& tunnel) override;
    bool setPlanMode() override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpcae
    ///@{
    int getStartStateID() const override;
    int getGoalStateID() const override;

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Reimplemneted Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetIslandSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs) override;

    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;

    void PrintState(int state_id, bool verbose, FILE* f = nullptr) override;
    ///@}

private:

    const OccupancyGrid* m_grid = nullptr;

    AdaptiveState* m_goal_state = nullptr;
    int m_goal_state_id = -1;

    AdaptiveState* m_start_state = nullptr;
    int m_start_state_id = -1;

    typedef AdaptiveWorkspaceState HiStateKey;
    typedef PointerValueHash<HiStateKey> HiStateHash;
    typedef PointerValueEqual<HiStateKey> HiStateEqual;
    typedef AdaptiveGridState LoStateKey;
    typedef PointerValueHash<LoStateKey> LoStateHash;
    typedef PointerValueEqual<LoStateKey> LoStateEqual;

    hash_map<HiStateKey*, int, HiStateHash, HiStateEqual> m_hi_to_id;
    hash_map<LoStateKey*, int, LoStateHash, LoStateEqual> m_lo_to_id;

    std::vector<AdaptiveState*> m_states;

    clock::time_point m_t_start;
    mutable bool m_near_goal = false;

    std::vector<Vector3> m_lo_prims;
    std::vector<MotionPrimitive> m_hi_prims;

    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    int m_region_radius = 1;
    int m_tunnel_radius = 3;

    bool m_plan_mode = true;

    struct AdaptiveGridCell
    {
        int grow_count;
        bool plan_hd; //planning_hd;
        bool trak_hd;

        AdaptiveGridCell() : grow_count(0), plan_hd(false), trak_hd(false) { }
    };
    Grid3<AdaptiveGridCell> m_dim_grid;

    bool initMotionPrimitives();

    bool setGoalPose(const GoalConstraint& goal);

    void GetSuccs(
        const AdaptiveGridState& state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetSuccs(
        const AdaptiveWorkspaceState& state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    int reserveHashEntry(bool hid);

    bool isHighDimensional(int gx, int gy, int gz) const;

    AdaptiveState* getHashEntry(int state_id) const;
    AdaptiveWorkspaceState* getHiHashEntry(int state_id) const;
    AdaptiveGridState* getLoHashEntry(int state_id) const;

    int getHiHashEntry(const WorkspaceCoord& coord);
    int getLoHashEntry(int x, int y, int z);

    int createHiState(const WorkspaceCoord& coord, const RobotState& state);
    int createLoState(int x, int y, int z, double wx, double wy, double wz);

    void getActions(
        const AdaptiveWorkspaceState& state,
        std::vector<Action>& actions);

    bool checkAction(
        const RobotState& state,
        const Action& action,
        RobotState* final_rstate = nullptr);

    bool isGoal(const WorkspaceState& state) const;
    bool isLoGoal(double x, double y, double z) const;

    auto getStateVisualization(const RobotState& state, const std::string& ns)
        -> std::vector<visual::Marker>;

    auto getAdaptiveGridVisualization(bool plan_mode) const -> visual::Marker;
};

} // namespace smpl

#endif
