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

#ifndef SMPL_EGRAPH_BFS_HEURISTIC_H
#define SMPL_EGRAPH_BFS_HEURISTIC_H

// standard includes
#include <limits>
#include <vector>

// project includes
#include <smpl/debug/marker.h>
#include <smpl/grid/grid.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/heuristic/egraph_heuristic.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/graph/experience_graph.h>

namespace smpl {

class IExperienceGraph;
class IProjectToPoint;
class IGetPosition;
class OccupancyGrid;

class DijkstraEGraphHeuristic3D :
    public Heuristic,
    public IExperienceGraphHeuristic,
    public IMetricStartHeuristic,
    public IMetricGoalHeuristic,
    public IGoalHeuristic
{
public:

    bool Init(DiscreteSpace* space, const OccupancyGrid* grid);

    void SyncGridAndDijkstra();

    auto GetGrid() const -> const OccupancyGrid*;

    auto GetEGraphWeight() const -> double;
    void SetEGraphWeight(double w);

    auto GetInflationRadius() const -> double;
    void SetInflationRadius(double radius);

    auto GetWallsVisualization() -> visual::Marker;
    auto GetValuesVisualization() -> visual::Marker;

    /// \name IExperienceGraphHeuristic Interface
    ///@{
    void GetEquivalentStates(int state_id, std::vector<int>& ids) final;
    void GetShortcutSuccs(int state_id, std::vector<int>& ids) final;
    ///@}

    /// \name IMetricStartHeuristic Interface
    ///@{
    double GetMetricStartDistance(double x, double y, double z) final;
    ///@}

    /// \name IMetricGoalHeuristic Interface
    ///@{
    double GetMetricGoalDistance(double x, double y, double z) final;
    ///@}

    /// \name IGoalHeuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) final;
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
        typedef Eigen::Vector3i argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type& s) const;
    };

    struct HeuristicNode
    {
        std::vector<ExperienceGraph::node_id> up_nodes;

        std::vector<Eigen::Vector3i> edges;
    };

    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    static const int Infinity = Unknown;

    int m_start_state_id = -1;

    const OccupancyGrid* m_grid = NULL;

    Grid3<Cell> m_dist_grid;

    double m_eg_eps = 1.0;
    double m_inflation_radius = 0.0;

    intrusive_heap<Cell, CellCompare> m_open;

    IProjectToPoint* m_pp = NULL;
    IExperienceGraph* m_eg = NULL;
    IGetPosition* m_get_goal_position = NULL;

    // map from experience graph nodes to their heuristic cell coordinates
    std::vector<Eigen::Vector3i> m_projected_nodes;

    // map from experience graph nodes to their component ids
    std::vector<int> m_component_ids;
    std::vector<std::vector<ExperienceGraph::node_id>> m_shortcut_nodes;

    hash_map<Eigen::Vector3i, HeuristicNode, Vector3iHash> m_heur_nodes;
};

} // namespace smpl

#endif
