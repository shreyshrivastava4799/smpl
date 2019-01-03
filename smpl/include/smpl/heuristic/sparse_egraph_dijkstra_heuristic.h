////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Andrew Dornbush
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

#ifndef SMPL_SPARSE_EGRAPH_DIJKSTRA_HEURISTIC
#define SMPL_SPARSE_EGRAPH_DIJKSTRA_HEURISTIC

// standard includes
#include <cstdlib>
#include <limits>

// system includes
#include <Eigen/Core>

// project includes
#include <smpl/debug/marker.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>
#include <smpl/graph/experience_graph.h>
#include <smpl/grid/sparse_grid.h>

namespace smpl {

class IExperienceGraph;
class IGetPosition;
class IProjectToPoint;
class OccupancyGrid;

class SparseEGraphDijkstra3DHeuristic :
    public Heuristic,
    public IExperienceGraphHeuristic,
    public IGoalHeuristic,
    public IMetricStartHeuristic,
    public IMetricGoalHeuristic
{
public:

    bool Init(DiscreteSpace* space, const OccupancyGrid* grid);

    void SyncGridAndDijkstra();

    auto GetGrid() const -> const OccupancyGrid*;

    auto GetEGraphWeight() const ->double;
    void SetEGraphWeight(double w);

    double GetInflationRadius() const;
    void SetInflationRadius(double radius);

    auto GetWallsVisualization() -> visual::Marker;
    auto GetValuesVisualization() -> visual::Marker;

    /// \name IExperienceGraphHeuristic Interface
    ///@{
    void GetEquivalentStates(int state_id, std::vector<int>& ids) final;
    void GetShortcutSuccs(int state_id, std::vector<int>& ids) final;
    ///@}

    /// \name IGoalHeuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) final;
    ///@}

    /// \name IMetricStartHeuristic Interface
    ///@{
    auto GetMetricStartDistance(double x, double y, double z) -> double final;
    ///@}

    /// \name IMetricGoalHeuristic Interface
    ///@{
    auto GetMetricGoalDistance(double x, double y, double z) -> double final;
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

public:

    struct Cell : public heap_element
    {
        int dist;

        Cell() = default;
        explicit Cell(int d) : heap_element(), dist(d) { }

        bool operator==(Cell o) const { return o.dist == dist; }
    };

    struct CellCompare
    {
        bool operator()(const Cell& a, const Cell& b) const {
            return a.dist < b.dist;
        }
    };

    // map down-projected state cells to adjacent down-projected state cells
    struct Vector3iHash
    {
        using argument_type = Eigen::Vector3i;
        using result_type = std::size_t;

        auto operator()(const argument_type& s) const -> result_type;
    };

    struct HeuristicNode
    {
        std::vector<ExperienceGraph::node_id> up_nodes;
        std::vector<Eigen::Vector3i> edges;
    };

    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    static const int Infinity = Unknown;

    const OccupancyGrid* m_grid = NULL;

    SparseGrid<Cell> m_dist_grid;

    int m_start_state_id = -1;

    double m_eg_eps = 1.0;
    double m_inflation_radius = 0.0;

    intrusive_heap<Cell, CellCompare> m_open;

    // we can't use the address of the cell to determine the position within
    // the grid, as we can with dense grids (this isn't entirely true, but it
    // involves more effort than I'm willing to put in right now), and we don't
    // want to store the position of each cell within the cell, to avoid
    // excessive memory overhead and break compression of the octree. Instead,
    // we'll retain the positions of each cell, only for cells that currently
    // in the open list
    std::unordered_map<Cell*, Eigen::Vector3i> m_open_cell_to_pos;

    IProjectToPoint* m_pp = NULL;
    IExperienceGraph* m_eg = NULL;
    IGetPosition* m_get_goal_position = NULL;

    // map from experience graph nodes to their heuristic cell coordinates
    std::vector<Eigen::Vector3i> m_projected_nodes;

    // map from experience graph nodes to their component ids
    std::vector<int> m_component_ids;
    std::vector<std::vector<ExperienceGraph::node_id>> m_shortcut_nodes;

    hash_map<Eigen::Vector3i, HeuristicNode, Vector3iHash> m_heur_nodes;

    void ProjectExperienceGraph();
    int GetGoalHeuristic(const Eigen::Vector3i& dp);
};

} // namespace smpl

#endif
