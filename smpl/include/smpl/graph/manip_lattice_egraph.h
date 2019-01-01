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

#ifndef SMPL_MANIP_LATTICE_EGRAPH_H
#define SMPL_MANIP_LATTICE_EGRAPH_H

// project includes
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/experience_graph.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl {

class ActionSpace;
class CollisionChecker;
class CostFunction;
class RobotModel;

// A graph composed of two sub-graphs, a typical joint-space lattice graph and
// an experience graph represented as a dense set of states and edges.
//
// The nodes in the graph are one of two types:
//
// (1) A state in the original joint-space lattice
// (2) A state in the experience graph
//
// The following types of actions are available from each state:
//
// (1) The original actions as defined by the joint-space lattice
// (2) Edges between states in the experience graph
// (3) Precomputed shortcut edges between states in the experience graph that
//     represent paths through the experience graph that may quickly lead to
//     the goal.
// (4) "Snap motions" or adaptively-generated motions between
//     original graph states and experience graph states, with similar
//     heuristic values, to avoid local minima imposed by the experience-graph
//     heuristic
// (5) Bridge edges that connect original and experience graph states within
//     the same discretization.
class ManipLatticeEGraph :
    public DiscreteSpace,
    public RobotPlanningSpace,
    public IProjectToPose,
    public IExtractRobotState,
    public ISearchable,
    public IExperienceGraph
{
public:

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions,
        CostFunction* cost_fun);

    void PrintState(int state_id, bool verbose, FILE* f = NULL);

    /// \name RobotPlanningSpace Interface
    ///@{
    int GetStateID(const RobotState& state) final;
    bool ExtractPath(const std::vector<int>& ids, std::vector<RobotState>& path) final;
    ///@}

    /// \name ISearchable
    ///@{
    void GetSuccs(int state_id, std::vector<int>* succs, std::vector<int>* costs) final;
    ///@}

    /// \name IProjectToPose Interface
    ///@{
    auto ProjectToPose(int state_id) -> Affine3 final;
    ///@}

    /// \name IExtractState Interface
    ///@{
    auto ExtractState(int state_id) -> const RobotState& final;
    ///@}

    /// \name IExperienceGraph Interface
    ///@{
    bool LoadExperienceGraph(const std::string& path) final;

    void GetExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) final;

    bool Shortcut(int first_id, int second_id, int& cost) final;
    bool Snap(int first_id, int second_id, int& cost) final;

    auto GetExperienceGraph() const -> const ExperienceGraph* final;
    auto GetExperienceGraph() -> ExperienceGraph* final;

    int GetStateID(ExperienceGraph::node_id n) const final;
    ///@}

    /// \name DiscreteSpace Interface
    ///@{
    bool UpdateGoal(GoalConstraint* goal) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

public:

    struct RobotCoordHash
    {
        using argument_type = std::vector<int>;
        using result_type = std::size_t;

        auto operator()(const argument_type& s) const -> result_type;
    };

    // The ManipLattice maintains both experience graph states and states
    // generated by calls to its GetSuccs function. The experience graph states
    // are preloaded into its state table and are unreachable by its GetSuccs
    // method.
    ManipLattice m_lattice;
    IProjectToPose* m_project_to_pose = NULL;

    ExperienceGraph m_egraph;

    // map from discrete state to the set of experience graph states within
    // the same discretization
    using CoordToExperienceGraphNodeMap = hash_map<
            RobotCoord,
            std::vector<ExperienceGraph::node_id>,
            RobotCoordHash>;
    CoordToExperienceGraphNodeMap m_coord_to_nodes;

    // map from state id to the experience graph node id
    hash_map<int, ExperienceGraph::node_id> m_state_to_node;

    // map from experience graph node ids to state ids
    std::vector<int> m_egraph_state_ids;
};

} // namespace smpl

#endif
