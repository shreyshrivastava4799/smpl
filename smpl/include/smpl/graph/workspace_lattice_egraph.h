#ifndef SMPL_WORKSPACE_LATTICE_EGRAPH_H
#define SMPL_WORKSPACE_LATTICE_EGRAPH_H

#include <smpl/types.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/experience_graph.h>

namespace smpl {

struct WorkspaceLatticeEGraph :
    public WorkspaceLattice,
    public ExperienceGraphExtension
{
    ExperienceGraph egraph;

    // map: [x, y, z, y, p, r, f0, ..., fn] -> [n1, ..., nn]
    using CoordToEGraphNodesMap = hash_map<
            WorkspaceCoord,
            std::vector<ExperienceGraph::node_id>,
            VectorHash<int>>;

    // map from discrete states to e-graph states lying within the same discrete
    // bin. This is queried to determine the set of edges E_bridge during
    // planning.
    CoordToEGraphNodesMap coord_to_egraph_nodes;

    // map from e-graph node id to state id
    std::vector<int> egraph_node_to_state;

    // map from state id back to e-graph node
    hash_map<int, ExperienceGraph::node_id> state_to_egraph_node;

    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        bool unique);

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;

    bool loadExperienceGraph(const std::string& path) override;

    /// \name Required ExperienceGraphExtension Interface
    ///@{
    void getExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) override;

    bool shortcut(int src_id, int dst_id, int& cost) override;

    bool snap(
        int first_id,
        int second_id,
        int& cost) override;

    auto getExperienceGraph() const -> const ExperienceGraph* override;
    auto getExperienceGraph() -> ExperienceGraph* override;

    int getStateID(ExperienceGraph::node_id n) const override;
    ///@}

    auto getExtension(size_t class_code) -> Extension* override;
};

bool FindShortestExperienceGraphPath(
    const ExperienceGraph& egraph,
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path);

} // namespace smpl

#endif

