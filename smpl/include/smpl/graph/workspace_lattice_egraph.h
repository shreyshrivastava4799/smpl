#include <smpl/types.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/experience_graph.h>

namespace smpl = sbpl::motion;

namespace sbpl {
namespace motion {

struct WorkspaceLatticeEGraph :
    public WorkspaceLattice,
    public ExperienceGraphExtension
{
    ExperienceGraph egraph;

    hash_map<WorkspaceCoord, std::vector<ExperienceGraph::node_id>, VectorHash<int>> coord_to_node;

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

} // namespace smpl
} // namespace sbpl

