#include <smpl/graph/workspace_lattice_egraph.h>

// standard includes
#include <fstream>

// system includes
#include <boost/filesystem.hpp>

// project includes
#include <smpl/csv_parser.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

namespace sbpl {
namespace motion {

bool WorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    SMPL_WARN("Path extraction unimplemented");
    return false;
}

bool ParseExperienceGraphFile(
    const std::string& filepath,
    RobotModel* robot_model,
    std::vector<RobotState>& egraph_states)
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        SMPL_WARN("Failed to open file '%s' for writing", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parse experience graph at '%s'", filepath.c_str());

    CSVParser parser;
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_WARN("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parsed experience graph file");
    SMPL_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_INFO("  %zu records", parser.recordCount());
    SMPL_INFO("  %zu fields", parser.fieldCount());

    auto jvar_count = robot_model->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        SMPL_WARN("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        RobotState state(jvar_count);
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    SMPL_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

// Load the experience graph from a database of paths:
// 1. Convert the raw path to an ExperienceGraph, which captures the
// connectivity of the demonstration (some states are retained as unique nodes
// and others make up the local paths between nodes)
// 2. Reserve special states in the graph for each unique node in the
// ExperienceGraph.
// 3. Construct a mapping between unique experience graph nodes and states in
// the graph.
//
// ExperienceGraph to uniquely label nodes
bool WorkspaceLatticeEGraph::loadExperienceGraph(const std::string& path)
{
    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!ParseExperienceGraphFile(filepath, robot(), egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) continue;

        SMPL_INFO("Create hash entries for experience graph states");

        auto& prev_pt = egraph_states.front();
        WorkspaceCoord prev_disc_pt(this->dofCount());
        this->stateRobotToCoord(prev_pt, prev_disc_pt);

        auto prev_node_id = this->egraph.insert_node(prev_pt);
        this->coord_to_egraph_nodes[prev_disc_pt].push_back(prev_node_id);

        auto state_id = this->reserveHashEntry();
        auto* state = getState(state_id);
        state->coord = prev_disc_pt;
        state->state = prev_pt;

        // map egraph node <-> egraph state
        this->egraph_node_to_state.resize(prev_node_id + 1, -1);
        this->egraph_node_to_state[prev_node_id] = state_id;
        this->state_to_egraph_node[state_id] = prev_node_id;

        // Walk through the demonstration and create a unique e-graph state
        // every time the discrete state changes. Intermediately encountered
        // states that span between two discrete states become the edges in
        // the e-graph.
        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            auto& pt = egraph_states[i];

            WorkspaceCoord disc_pt(this->dofCount());
            this->stateRobotToCoord(pt, disc_pt);

            if (disc_pt != prev_disc_pt) {
                auto node_id = this->egraph.insert_node(pt);
                this->coord_to_egraph_nodes[disc_pt].push_back(node_id);

                auto state_id = this->reserveHashEntry();
                auto* state = this->getState(state_id);
                state->coord = disc_pt;
                state->state = pt;

                this->egraph_node_to_state.resize(node_id + 1, -1);
                this->egraph_node_to_state[node_id] = state_id;
                this->state_to_egraph_node[state_id] = node_id;
                this->egraph.insert_edge(prev_node_id, node_id, edge_data);

                prev_disc_pt = disc_pt;
                prev_node_id = node_id;
                edge_data.clear();
            } else {
                edge_data.push_back(pt);
            }
        }
    }

    SMPL_INFO("Experience garph contains %zu nodes and %zu edges", this->egraph.num_nodes(), this->egraph.num_edges());
    return true;
}

void WorkspaceLatticeEGraph::getExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = this->state_to_egraph_node.find(state_id);
    if (it != end(this->state_to_egraph_node)) {
        nodes.push_back(it->second);
    }
}

bool WorkspaceLatticeEGraph::shortcut(int src_id, int dst_id, int& cost)
{
    auto* src_state = this->getState(src_id);
    auto* dst_state = this->getState(dst_id);
    assert(src_state != NULL && dst_state != NULL);

    SMPL_INFO_STREAM("Shortcut " << src_state->coord << " -> " << dst_state->state);
    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(src_state->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(dst_state->state, "shortcut_to"));

    SMPL_INFO("  shortcut %d -> %d!", src_id, dst_id);
    cost = 1000;
    return true;
}

bool WorkspaceLatticeEGraph::snap(int src_id, int dst_id, int& cost)
{
    auto* src_state = this->getState(src_id);
    auto* dst_state = this->getState(dst_id);
    assert(src_state != NULL && dst_state != NULL);

    SMPL_INFO_STREAM("Snap " << src_state->coord << " -> " << dst_state->state);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(src_state->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(dst_state->state, "snap_to"));

    if (!this->collisionChecker()->isStateToStateValid(
            src_state->state, dst_state->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_INFO("  Snap %d -> %d!", src_id, dst_id);
    cost = 1000;
    return true;
}

auto WorkspaceLatticeEGraph::getExperienceGraph() const -> const ExperienceGraph*
{
    return &this->egraph;
}

auto WorkspaceLatticeEGraph::getExperienceGraph() -> ExperienceGraph*
{
    return &this->egraph;
}

int WorkspaceLatticeEGraph::getStateID(ExperienceGraph::node_id n) const
{
    if (n >= this->egraph_node_to_state.size()) {
        return -1;
    } else {
        return this->egraph_node_to_state[n];
    }
}

Extension* WorkspaceLatticeEGraph::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphExtension>()) {
        return this;
    } else {
        return WorkspaceLattice::getExtension(class_code);
    }
}

} // namespace motion
} // namespace sbpl

