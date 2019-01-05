#ifndef SMPL_WORKSPACE_LATTICE_EGRAPH_H
#define SMPL_WORKSPACE_LATTICE_EGRAPH_H

// project includes
#include <smpl/types.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/experience_graph.h>

namespace smpl {

class WorkspaceLatticeEGraph :
    public DiscreteSpace,
    public RobotPlanningSpace,
    public IExtractRobotState,
    public IProjectToPose,
    public ISearchable,
    public IExperienceGraph
{
public:

    bool Init(
        RobotModel* robot,
        CollisionChecker* checker,
        const WorkspaceProjectionParams& params,
        WorkspaceLatticeActionSpace* actions);

    void GetEGraphStateSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetEGraphStateAdjacentSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetEGraphStateBridgeSuccs(
        WorkspaceLatticeState* state,
        ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetOrigStateSuccs(
        smpl::WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetOrigStateBridgeSuccs(
        WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void GetOrigStateOrigSuccs(
        WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void InsertExperienceGraphPath(const std::vector<smpl::RobotState>& path);
    void ClearExperienceGraph();

    auto GetVisualizationFrameId() const -> const std::string&;
    void SetVisualizationFrameId(const std::string& frame_id);

    /// \name IExperienceGraph Interface
    ///@{
    bool LoadExperienceGraph(const std::string& path) final;

    void GetExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) final;

    bool Shortcut(int src_id, int dst_id, int& cost) final;

    bool Snap(int first_id, int second_id, int& cost) final;

    auto GetExperienceGraph() const -> const ExperienceGraph* final;
    auto GetExperienceGraph() -> ExperienceGraph* final;

    int GetStateID(ExperienceGraph::node_id n) const final;
    ///@}

    /// \name RobotPlanningSpace Interface
    ///@{
    int GetStateID(const RobotState& state) final;

    bool ExtractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) final;
    ///@}

    /// \name IExtractRobotState Interface
    ///@{
    auto ExtractState(int state_id) -> const RobotState& final;
    ///@}

    /// \name IProjectToPose Interface
    ///@{
    auto ProjectToPose(int state_id) -> Affine3 final;
    ///@}

    /// \name DiscreteSpace Interface
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final;
    ///@}

public:

    WorkspaceLattice m_lattice;
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

