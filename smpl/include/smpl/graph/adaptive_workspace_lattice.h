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
#include <cstddef>
#include <functional>
#include <iosfwd>
#include <vector>

// project includes
#include <smpl/graph/adaptive_graph_extension.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/motion_primitive.h>
#include <smpl/graph/workspace_lattice_base.h>
#include <smpl/grid/grid.h>
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

bool operator==(const AdaptiveGridState& a, const AdaptiveGridState& b);
auto operator<<(std::ostream& o, const AdaptiveGridState& s) -> std::ostream&;

struct AdaptiveWorkspaceState : public AdaptiveState
{
    RobotState state;
    WorkspaceCoord coord;
};

bool operator==(const AdaptiveWorkspaceState& a, const AdaptiveWorkspaceState& b);
auto operator<<(std::ostream& o, const AdaptiveWorkspaceState& s) -> std::ostream&;

} // namespace smpl

// std::hash specializations for state types
namespace std {

template <>
struct hash<smpl::AdaptiveGridState>
{
    using argument_type = smpl::AdaptiveGridState;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type;
};

template <>
struct hash<smpl::AdaptiveWorkspaceState>
{
    using argument_type = smpl::AdaptiveWorkspaceState;
    using result_type = std::size_t;
    auto operator()(const argument_type& s) const -> result_type;
};

} // namespace std

namespace smpl {

class OccupancyGrid;

// This class represents two lattice structures, a typical high-dimensional
// lattice structure and a low-dimensional lattice structure, with a projection
// function that maps between them. A search through this lattice structure may
// alternate between searching in the high-dimensional vs. the low-dimensional
// lattice.
class AdaptiveWorkspaceLattice :
    public DiscreteSpace,
    public RobotPlanningSpace,
    public IProjectToPoint,
    public IAdaptiveGraph,
    public ISearchable
{
public:

    ~AdaptiveWorkspaceLattice();

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const WorkspaceProjection::Params& params,
        const OccupancyGrid* grid);

    void PrintState(int state_id, bool verbose, FILE* f = NULL);

    /// \name RobotPlanningSpace Interface
    ///@{
    int GetStateID(const RobotState& state) final;
    bool ExtractPath(
        const std::vector<int>& state_ids,
        std::vector<RobotState>& path) final;
    ///@}

    /// \name IProjectToPoint Interface
    ///@{
    auto ProjectToPoint(int state_id) -> Vector3 final;
    ///@}

    /// \name IAdaptiveGraph Interface
    ///@{
    bool AddHighDimRegion(int state_id) final;
    bool SetTunnel(const std::vector<int>& states) final;
    bool IsExecutable(const std::vector<int>& states) const final;
    bool SetTrackMode(const std::vector<int>& tunnel) final;
    bool SetPlanMode() final;
    ///@}

    /// \name ISearchable
    ///@{
    void GetSuccs(int state_id, std::vector<int>* succs, std::vector<int>* costs) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

public:

    WorkspaceProjection m_proj;

    const OccupancyGrid* m_grid = NULL;

    using HiStateKey = AdaptiveWorkspaceState;
    using HiStateHash = PointerValueHash<HiStateKey>;
    using HiStateEqual = PointerValueEqual<HiStateKey>;
    using LoStateKey = AdaptiveGridState;
    using LoStateHash = PointerValueHash<LoStateKey>;
    using LoStateEqual = PointerValueEqual<LoStateKey>;

    hash_map<HiStateKey*, int, HiStateHash, HiStateEqual> m_hi_to_id;
    hash_map<LoStateKey*, int, LoStateHash, LoStateEqual> m_lo_to_id;

    std::vector<AdaptiveState*> m_states;

    std::vector<Vector3> m_lo_prims;
    std::vector<MotionPrimitive> m_hi_prims;

    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    int m_region_radius = 1;
    int m_tunnel_radius = 3;

    bool m_plan_mode = true;

    struct AdaptiveGridCell
    {
        int grow_count = 0;
        bool plan_hd = false;
        bool trak_hd = false;
    };
    Grid3<AdaptiveGridCell> m_dim_grid;
};

} // namespace smpl

#endif
