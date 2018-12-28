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

#include <smpl/graph/experience_graph.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/manip_lattice.h>

namespace smpl {

class ManipLatticeEgraph :
    public RobotPlanningSpace,
    public PoseProjectionExtension,
    public ExtractRobotStateExtension,
    public ExperienceGraphExtension
{
public:

    /// \name RobotPlanningSpace Interface
    ///@{
    int getStartStateID() const final;
    int getGoalStateID() const final;
    bool extractPath(const std::vector<int>& ids, std::vector<RobotState>& path) final;
    void GetSuccs(int state_id, std::vector<int>* succs, std::vector<int>* costs) final;
    void GetPreds(int state_id, std::vector<int>* preds, std::vector<int>* costs) final;
    void PrintState(int state_id, bool verbose, FILE* f = NULL) final;
    ///@}

    /// \name PoseProjectionExtension Interface
    ///@{
    bool projectToPose(int state_id, Affine3& pose) final;
    ///@}

    /// \name ExtractRobotStateExtension Interface
    ///@{
    auto extractState(int state_id) -> const RobotState& final;
    ///@}

    /// \name ExperienceGraphExtension Interface
    ///@{
    bool loadExperienceGraph(const std::string& path) final;

    void getExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) final;

    bool shortcut(int first_id, int second_id, int& cost) final;
    bool snap(int first_id, int second_id, int& cost) final;

    auto getExperienceGraph() const -> const ExperienceGraph* final;
    auto getExperienceGraph() -> ExperienceGraph* final;

    int getStateID(ExperienceGraph::node_id n) const final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> Extension* override;
    ///@}

    struct RobotCoordHash
    {
        typedef std::vector<int> argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type& s) const;
    };

    ManipLattice lattice;

    typedef hash_map<
            RobotCoord,
            std::vector<ExperienceGraph::node_id>,
            RobotCoordHash>
    CoordToExperienceGraphNodeMap;

    CoordToExperienceGraphNodeMap m_coord_to_nodes;
    hash_map<int, ExperienceGraph::node_id> m_state_to_node;

    ExperienceGraph m_egraph;

    // map from experience graph node ids to state ids
    std::vector<int> m_egraph_state_ids;
};

} // namespace smpl

#endif
