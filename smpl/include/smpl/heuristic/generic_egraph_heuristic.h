////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_GENERIC_EGRAPH_HEURISTIC_H
#define SMPL_GENERIC_EGRAPH_HEURISTIC_H

// standard includes
#include <limits>
#include <vector>

// project includes
#include <smpl/graph/experience_graph.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

namespace smpl {

class IExperienceGraph;
class IPairwiseHeuristic;
class IGoalHeuristic;

class GenericEGraphHeuristic :
    public Heuristic,
    public IExperienceGraphHeuristic,
    public IGoalHeuristic,
    private IMetricStartHeuristic,
    private IMetricGoalHeuristic
{
public:

    bool Init(DiscreteSpace* space, Heuristic* h);

    auto GetEGraphWeight() const -> double;
    void SetEGraphWeight(double w);

    /// \name IExperienceGraphHeuristic Interface
    ///@{
    void GetEquivalentStates(int state_id, std::vector<int>& ids) final;
    void GetShortcutSuccs(int state_id, std::vector<int>& ids) final;
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

    struct HeuristicNode : public heap_element
    {
        int dist;

        HeuristicNode() = default;
        HeuristicNode(int d) : heap_element(), dist(d) { }
    };

    struct NodeCompare
    {
        bool operator()(const HeuristicNode& a, const HeuristicNode& b) const
        {
            return a.dist < b.dist;
        }
    };

    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    static const int Infinity = Unknown;

    Heuristic* m_orig_h = NULL;
    IGoalHeuristic* m_goal_h = NULL;
    IPairwiseHeuristic* m_pairwise_h = NULL;
    IMetricStartHeuristic* m_metric_start_h = NULL;
    IMetricGoalHeuristic* m_metric_goal_h = NULL;

    IExperienceGraph* m_eg = NULL;

    double m_eg_eps = 1.0;

    std::vector<int> m_component_ids;
    std::vector<std::vector<ExperienceGraph::node_id>> m_shortcut_nodes;

    std::vector<HeuristicNode> m_h_nodes;
    intrusive_heap<HeuristicNode, NodeCompare> m_open;

    /// \name IMetricStartHeuristic Interface
    ///@{
    auto GetMetricStartDistance(double x, double y, double z) -> double final;
    ///@}

    /// \name IMetricGoalHeuristic Interface
    ///@{
    auto GetMetricGoalDistance(double x, double y, double z) -> double final;
    ///@}
};

} // namespace smpl

#endif
