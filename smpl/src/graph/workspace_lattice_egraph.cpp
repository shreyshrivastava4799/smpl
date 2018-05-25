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
#include <smpl/intrusive_heap.h>

namespace sbpl {
namespace motion {

static
bool FindShortestExperienceGraphPath(
    const ExperienceGraph& egraph,
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path)
{
    struct ExperienceGraphSearchNode : heap_element
    {
        int                         g = std::numeric_limits<int>::max();
        bool                        closed = false;
        ExperienceGraphSearchNode*  bp = NULL;
    };

    struct NodeCompare
    {
        bool operator()(
            const ExperienceGraphSearchNode& a,
            const ExperienceGraphSearchNode& b)
        {
            return a.g < b.g;
        }
    };

    typedef intrusive_heap<ExperienceGraphSearchNode, NodeCompare> heap_type;

    std::vector<ExperienceGraphSearchNode> search_nodes(egraph.num_nodes());

    heap_type open;

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    int exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        ExperienceGraphSearchNode* min = open.min();
        open.pop();
        min->closed = true;

        if (min == &search_nodes[goal_node]) {
            SMPL_DEBUG("Found shortest experience graph path");
            ExperienceGraphSearchNode* ps = nullptr;
            for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                s; s = s->bp)
            {
                if (s != ps) {
                    path.push_back(std::distance(search_nodes.data(), s));
                    ps = s;
                } else {
                    SMPL_ERROR("Cycle detected!");
                }
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
        auto adj = egraph.adjacent_nodes(n);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            ExperienceGraphSearchNode& succ = search_nodes[*ait];
            if (succ.closed) {
                continue;
            }
            int new_cost = min->g + 1;
            if (new_cost < succ.g) {
                succ.g = new_cost;
                succ.bp = min;
                if (open.contains(&succ)) {
                    open.decrease(&succ);
                } else {
                    open.push(&succ);
                }
            }
        }
    }

    SMPL_INFO("Expanded %d nodes looking for shortcut", exp_count);
    return false;
}

static
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
    if (parser.fieldCount() < jvar_count) {
        SMPL_WARN("Parsed experience graph contains insufficient number of joint variables (%zu < %zu)", parser.fieldCount(), jvar_count);
        return false;
    }
    if (parser.fieldCount() > jvar_count) {
        SMPL_WARN("Parsed experience graph contains superflous many joint variables (%zu > %zu)", parser.fieldCount(), jvar_count);
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

bool WorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(this->params()->graph_log, "State ID Path: " << ids);

    if (ids.empty()) return true;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (ids.size() == 1) {
        auto state_id = ids[0];

        if (state_id == this->getGoalStateID()) {
            auto* entry = this->getState(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(this->params()->graph_log, "Failed to get state entry for state %d", this->getStartStateID());
                return false;
            }
            path.push_back(entry->state);
        } else {
            auto* entry = this->getState(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(this->params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (ids[0] == this->getGoalStateID()) {
        SMPL_ERROR_NAMED(this->params()->graph_log, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<RobotState> opath;

    // grab the first point
    {
        auto* entry = this->getState(ids[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(this->params()->graph_log, "Failed to get state entry for state %d", ids[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(this->params()->graph_log, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == this->getGoalStateID()) {
            SMPL_ERROR_NAMED(this->params()->graph_log, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        // find the successor state corresponding to the cheapest valid action

        auto* prev_entry = this->getState(prev_id);
        auto& prev_state = prev_entry->state;

        std::vector<Action> actions;
        this->m_actions.apply(*prev_entry, actions);
//        SMPL_ERROR_NAMED(this->params()->graph_log, "Failed to get actions while extracting the path");

        SMPL_DEBUG_NAMED(this->params()->graph_log, "Check for transition via normal successors");
        WorkspaceLatticeState* best_state = NULL;
        WorkspaceCoord succ_coord(this->robot()->jointVariableCount());
        auto best_cost = std::numeric_limits<int>::max();

        if (curr_id == this->getGoalStateID()) {
            SMPL_DEBUG_NAMED(this->params()->graph_log, "Search for transition to goal state");
        }

        for (auto& action : actions) {
            // check the validity of this transition
            if (!this->checkAction(prev_state, action)) continue;

            if (curr_id == this->getGoalStateID()) {

                // skip non-goal states
                if (!this->isGoal(action.back())) continue;

                this->stateWorkspaceToCoord(action.back(), succ_coord);
                int succ_state_id = this->createState(succ_coord);
                auto* succ_entry = this->getState(succ_state_id);
                assert(succ_entry);

                auto edge_cost = this->computeCost(*prev_entry, *succ_entry);

                SMPL_DEBUG_NAMED(this->params()->graph_log, "Found a goal state at %d with cost %d", succ_state_id, edge_cost);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                }
            } else {
                this->stateWorkspaceToCoord(action.back(), succ_coord);
                auto succ_state_id = this->createState(succ_coord);
                auto* succ_entry = this->getState(succ_state_id);
                assert(succ_entry);
                if (succ_state_id != curr_id) continue;

                auto edge_cost = computeCost(*prev_entry, *succ_entry);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                }
            }
        }

        if (best_state != NULL) {
            SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "Extract successor state " << best_state->state);
            opath.push_back(best_state->state);
            continue;
        }

        SMPL_DEBUG_NAMED(params()->graph_log, "Check for shortcut successor");

        auto found = false;
        // check for shortcut transition
        auto pnit = std::find(begin(this->egraph_node_to_state), end(this->egraph_node_to_state), prev_id);
        auto cnit = std::find(begin(this->egraph_node_to_state), end(this->egraph_node_to_state), curr_id);
        if (pnit != end(this->egraph_node_to_state) &&
            cnit != end(this->egraph_node_to_state))
        {
            // position in node array is synonymous with e-graph node id
            auto prev_node = std::distance(begin(this->egraph_node_to_state), pnit);
            auto curr_node = std::distance(begin(this->egraph_node_to_state), cnit);

            SMPL_INFO("Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, prev_node, curr_node);

            std::vector<ExperienceGraph::node_id> node_path;
            found = FindShortestExperienceGraphPath(this->egraph, prev_node, curr_node, node_path);
            if (found) {
                for (ExperienceGraph::node_id n : node_path) {
                    auto state_id = this->egraph_node_to_state[n];
                    auto* entry = this->getState(state_id);
                    assert(entry);
                    opath.push_back(entry->state);
                }
            }
        }

        if (found) continue;

        // check for snap transition
        SMPL_DEBUG_NAMED(params()->graph_log, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            SMPL_INFO("Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            auto* entry = this->getState(curr_id);
            assert(entry);
            opath.push_back(entry->state);
            continue;
        }

        SMPL_ERROR_NAMED(params()->graph_log, "Failed to find valid goal successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);

    SMPL_INFO("Final path:");
    for (auto& point : path) {
        SMPL_INFO_STREAM("  " << point);
    }

    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
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
        {
            this->coord_to_egraph_nodes[prev_disc_pt].push_back(prev_node_id);

            auto state_id = this->reserveHashEntry();
            auto* state = getState(state_id);
            state->coord = prev_disc_pt;
            state->state = prev_pt;

            // map egraph node <-> egraph state
            this->egraph_node_to_state.resize(prev_node_id + 1, -1);
            this->egraph_node_to_state[prev_node_id] = state_id;
            this->state_to_egraph_node[state_id] = prev_node_id;
        }

        // Walk through the demonstration and create a unique e-graph state
        // every time the discrete state changes. Intermediately encountered
        // states that span between two discrete states become the edges in
        // the e-graph.
        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            auto& robot_state = egraph_states[i];

            WorkspaceCoord disc_pt(this->dofCount());
            this->stateRobotToCoord(robot_state, disc_pt);

            if (disc_pt != prev_disc_pt) {
                auto node_id = this->egraph.insert_node(robot_state);
                this->coord_to_egraph_nodes[disc_pt].push_back(node_id);

                auto state_id = this->reserveHashEntry();
                auto* state = this->getState(state_id);
                state->coord = disc_pt;
                state->state = robot_state;

                this->egraph_node_to_state.resize(node_id + 1, -1);
                this->egraph_node_to_state[node_id] = state_id;
                this->state_to_egraph_node[state_id] = node_id;
                this->egraph.insert_edge(prev_node_id, node_id, edge_data);

                prev_disc_pt = disc_pt;
                prev_node_id = node_id;
                edge_data.clear();
            } else {
                edge_data.push_back(robot_state);
            }
        }
    }

    SMPL_INFO("Experience graph contains %zu nodes and %zu edges", this->egraph.num_nodes(), this->egraph.num_edges());
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

    SMPL_INFO_STREAM("Shortcut " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(src_state->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(dst_state->state, "shortcut_to"));

    SMPL_INFO("  shortcut %d -> %d!", src_id, dst_id);
    cost = 10;
    return true;
}

bool WorkspaceLatticeEGraph::snap(int src_id, int dst_id, int& cost)
{
    auto* src_state = this->getState(src_id);
    auto* dst_state = this->getState(dst_id);
    assert(src_state != NULL && dst_state != NULL);

    SMPL_INFO_STREAM("Snap " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(src_state->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(dst_state->state, "snap_to"));

    if (!this->collisionChecker()->isStateToStateValid(
            src_state->state, dst_state->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_DEBUG("  Snap %d -> %d!", src_id, dst_id);
    cost = 10;
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

