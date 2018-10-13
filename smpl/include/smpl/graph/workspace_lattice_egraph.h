#ifndef SMPL_WORKSPACE_LATTICE_EGRAPH_H
#define SMPL_WORKSPACE_LATTICE_EGRAPH_H

#include <smpl/types.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/experience_graph.h>

namespace smpl {

class WorkspaceLatticeEGraph :
    public WorkspaceLattice,
    public ExperienceGraphExtension
{
public:

    ExperienceGraph m_egraph;

    // map: [x, y, z, y, p, r, f0, ..., fn] -> [n1, ..., nn]
    using CoordToEGraphNodesMap = hash_map<
            WorkspaceCoord,
            std::vector<ExperienceGraph::node_id>,
            VectorHash<int>>;

    // map from discrete states to e-graph states lying within the same discrete
    // bin. This is queried to determine the set of edges E_bridge during
    // planning.
    CoordToEGraphNodesMap m_coord_to_egraph_nodes;

    // map from e-graph node id to state id
    std::vector<int> m_egraph_node_to_state;

    // map from state id back to e-graph node
    hash_map<int, ExperienceGraph::node_id> m_state_to_egraph_node;

    void getUniqueSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getEGraphStateSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getEGraphStateAdjacentSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getEGraphStateBridgeSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getOrigStateSuccs(
        smpl::WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getOrigStateBridgeSuccs(
        WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getOrigStateOrigSuccs(
        WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void insertExperienceGraphPath(const std::vector<smpl::RobotState>& path);
    void clearExperienceGraph();

    /// \name ExperienceGraphExtension Interface
    ///@{
    bool loadExperienceGraph(const std::string& path) override;

    void getExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) override;

    bool shortcut(int src_id, int dst_id, int& cost) override;

    bool snap(int first_id, int second_id, int& cost) override;

    auto getExperienceGraph() const -> const ExperienceGraph* override;
    auto getExperienceGraph() -> ExperienceGraph* override;

    int getStateID(ExperienceGraph::node_id n) const override;
    ///@}

    /// \name RobotPlanningSpaceInterface
    ///@{
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name DiscreteSpaceInformation Interface
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> Extension* override;
    ///@}
};

bool ParseExperienceGraphFile(
    const std::string& filepath,
    RobotModel* robot_model,
    std::vector<RobotState>& egraph_states);

bool FindShortestExperienceGraphPath(
    const ExperienceGraph& egraph,
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path);

} // namespace smpl

#endif

