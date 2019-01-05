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

#ifndef SMPL_BFS_HEURISTIC_H
#define SMPL_BFS_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/debug/marker.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/bfs3d/bfs3d.h>

namespace smpl {

class IProjectToPoint;
class OccupancyGrid;

class BFSHeuristic :
    public Heuristic,
    public IGoalHeuristic,
    public IMetricGoalHeuristic,
    public IMetricStartHeuristic
{
public:

    bool Init(DiscreteSpace* space, const OccupancyGrid* grid);

    double GetInflationRadius() const;
    void SetInflationRadius(double radius);

    int GetCostPerCell() const;
    void SetCostPerCell(int cost);

    auto GetGrid() const -> const OccupancyGrid*;

    auto GetWallsVisualization() const -> visual::Marker;
    auto GetValuesVisualization() -> visual::Marker;

    void SyncGridAndBFS();

    /// \name IGoalHeuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) final;
    ///@}

    /// \name IMetricStartHeuristic Interface
    ///@{
    double GetMetricStartDistance(double x, double y, double z) final;
    ///@}

    /// \name IMetricGoalHeuristic Interface
    ///@{
    double GetMetricGoalDistance(double x, double y, double z) final;
    ///@}

    /// \name Heuristic Interface
    ///@{
    bool UpdateStart(int state_id) final;
    bool UpdateGoal(GoalConstraint* goal) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

private:

    static constexpr auto Infinity = ((1 << 16) - 1);

    const OccupancyGrid* m_grid = NULL;

    std::unique_ptr<BFS_3D> m_bfs;
    IProjectToPoint* m_project_to_point = NULL;

    double m_inflation_radius = 0.0;
    int m_cost_per_cell = 1;

    int m_start_state_id = -1;

    struct CellCoord
    {
        int x, y, z;
        CellCoord() = default;
        CellCoord(int x, int y, int z) : x(x), y(y), z(z) { }
    };
    std::vector<CellCoord> m_goal_cells;

    int GetBFSCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;
};

} // namespace smpl

#endif
