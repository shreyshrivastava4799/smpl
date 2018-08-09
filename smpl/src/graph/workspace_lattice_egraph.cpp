#include <smpl/graph/workspace_lattice_egraph.h>

// standard includes
#include <fstream>

// system includes
#include <boost/filesystem.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/csv_parser.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/heap/intrusive_heap.h>

namespace smpl {

static
bool GetSnapMotion(
    WorkspaceLatticeEGraph* graph,
    int src_id,
    int dst_id,
    std::vector<RobotState>& path)
{
    auto* src_state = graph->getState(src_id);
    auto* dst_state = graph->getState(dst_id);

    WorkspaceState start_state;
    WorkspaceState finish_state;
    graph->stateCoordToWorkspace(src_state->coord, start_state);
    graph->stateCoordToWorkspace(dst_state->coord, finish_state);
    int num_waypoints = 10;
    for (int i = 0; i < num_waypoints; ++i) {
        WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(graph->dofCount());

        // x, y, z, R, P, Y, FA1 same as start
        interm_workspace_state[0] = finish_state[0];
        interm_workspace_state[1] = finish_state[1];
        interm_workspace_state[2] = finish_state[2];
        interm_workspace_state[3] = finish_state[3];
        interm_workspace_state[4] = finish_state[4];
        interm_workspace_state[5] = finish_state[5];
        interm_workspace_state[6] = finish_state[6];

        auto interp = [](double a, double b, double t) {
            return (1.0 - t) * a + t * b;
        };

        auto t = (double)i / (double)(num_waypoints - 1);
        // interpolate torso, theta, x, y
        interm_workspace_state[7] = interp(start_state[7], finish_state[7], t);
        interm_workspace_state[8] = interp(start_state[8], finish_state[8], t);
        interm_workspace_state[9] = interp(start_state[9], finish_state[9], t);
        interm_workspace_state[10] = interp(start_state[10], finish_state[10], t);

        // hinge kept the same
        interm_workspace_state[11] = finish_state[11];

        auto stateWorkspaceToRobotPermissive = [&](
            const WorkspaceState& state,
            RobotState& ostate)
        {
            RobotState seed = src_state->state; //(graph->robot()->jointVariableCount(), 0);
            for (size_t fai = 0; fai < graph->freeAngleCount(); ++fai) {
                seed[graph->m_fangle_indices[fai]] = state[6 + fai];
            }

            Eigen::Affine3d pose =
                    Eigen::Translation3d(state[0], state[1], state[2]) *
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX());

//            ostate = seed;
//            return graph->m_ik_iface->computeIK(pose, dst_state->state, ostate);
            return graph->m_ik_iface->computeIK(pose, seed, ostate);
        };

        RobotState robot_state;
        // TODO: this should be permissive and allow moving the redundant angles
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
            SMPL_WARN("Failed to find ik solution for interpolated state");
            return false;
        }

        path.push_back(std::move(robot_state));
    }

    return true;
}

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
            SMPL_DEBUG_NAMED(G_LOG, "Found shortest experience graph path");
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

    SMPL_DEBUG_NAMED(G_LOG, "Expanded %d nodes looking for shortcut", exp_count);
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

    SMPL_DEBUG_NAMED(G_LOG, "Parse experience graph at '%s'", filepath.c_str());

    CSVParser parser;
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_WARN("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "Parsed experience graph file");
    SMPL_DEBUG_NAMED(G_LOG, "  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_DEBUG_NAMED(G_LOG, "  %zu records", parser.recordCount());
    SMPL_DEBUG_NAMED(G_LOG, "  %zu fields", parser.fieldCount());

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

    SMPL_DEBUG_NAMED(G_LOG, "Read %zu states from experience graph file", egraph_states.size());
    return true;
}

void WorkspaceLatticeEGraph::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    return GetSuccs(state_id, succs, costs, false);
}

auto GetPoseCoord(const WorkspaceCoord& coord) -> std::vector<int>
{
    std::vector<int> pose(4);
    pose[0] = coord[0];
    pose[1] = coord[1];
    pose[2] = coord[2];
    pose[3] = coord[5];
    return pose;
}

void WorkspaceLatticeEGraph::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    assert(state_id >= 0 && state_id < m_states.size());

    succs->clear();
    costs->clear();

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    auto* parent_entry = getState(state_id);
    assert(parent_entry != NULL);
    assert(parent_entry->coord.size() == this->dofCount());

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    bool is_egraph_node = false;
    ExperienceGraph::node_id egraph_node;
    {
        auto it = this->state_to_egraph_node.find(state_id);
        if (it != end(this->state_to_egraph_node)) {
            is_egraph_node = true;
            egraph_node = it->second;
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  egraph state: %s", is_egraph_node ? "true" : "false");

    if (is_egraph_node) { // expanding an egraph node
        auto& egraph_state = this->egraph.state(egraph_node);

        // E_bridge from V_demo
        {
            // egraph robot state -> workspace state
            WorkspaceState workspace_state;
            this->stateRobotToWorkspace(egraph_state, workspace_state);

            // discrete successor state
            WorkspaceCoord workspace_coord;
            this->stateWorkspaceToCoord(workspace_state, workspace_coord);

            // successor robot state
            RobotState robot_state;
            if (this->stateWorkspaceToRobot(workspace_state, egraph_state, robot_state)) {
                // inherit the exact z value from the e-graph state
                robot_state.back() = egraph_state.back();

                // TODO: check action

                auto succ_id = createState(workspace_coord);
                auto* succ_state = getState(succ_id);
                succ_state->state = robot_state;

                SMPL_INFO_NAMED(G_EXPANSIONS_LOG, "  bridge edge to %d", succ_id);

                if (!unique && this->isGoal(workspace_state, robot_state)) {
                    succs->push_back(this->getGoalStateID());
                } else {
                    succs->push_back(succ_id);
                }

                costs->push_back(this->computeCost(*parent_entry, *succ_state));
            } else {
                SMPL_INFO_NAMED(G_EXPANSIONS_LOG, "  bridge edge failed");
            }
        }

        // E_demo from V_demo
        auto adj = this->egraph.adjacent_nodes(egraph_node);
        SMPL_INFO_NAMED(G_EXPANSIONS_LOG, "  %td adjacent experience graph edges", std::distance(adj.first, adj.second));
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto& adj_egraph_state = this->egraph.state(*ait);
            WorkspaceState workspace_state;
            this->stateRobotToWorkspace(adj_egraph_state, workspace_state);
            if (!unique && this->isGoal(workspace_state, adj_egraph_state)) {
                succs->push_back(this->m_goal_state_id);
            } else {
                succs->push_back(this->egraph_node_to_state[*ait]);
            }
            costs->push_back(10);
        }

        return;
    }

    std::vector<WorkspaceAction> actions;
    m_actions->apply(*parent_entry, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // e-graph nodes within psi tolerance, for E_z
    std::vector<ExperienceGraph::node_id> parent_nearby_nodes;
    {
        auto pose_coord = GetPoseCoord(parent_entry->coord);
        SMPL_DEBUG_STREAM("parent pose coord = " << pose_coord);
        auto it = this->psi_to_egraph_nodes.find(pose_coord);
        if (it != end(this->psi_to_egraph_nodes)) {
            for (auto node : it->second) {
                auto z = this->egraph.state(node).back();
                if (z == parent_entry->state.back()) {
                    parent_nearby_nodes.push_back(node);
                }
            }
        }
    }

    // E_bridge from V_orig
    auto it = coord_to_egraph_nodes.find(parent_entry->coord);
    if (it != end(coord_to_egraph_nodes)) {
        SMPL_INFO_NAMED(G_EXPANSIONS_LOG, "  bridge to %zu e-graph nodes", it->second.size());
        for (auto node : it->second) {
            auto succ_id = this->egraph_node_to_state[node];
            auto& egraph_state = this->egraph.state(node);
            WorkspaceState workspace_state;
            this->stateRobotToWorkspace(egraph_state, workspace_state);
            if (!unique && this->isGoal(workspace_state, egraph_state)) {
                succs->push_back(this->getGoalStateID());
            } else {
                succs->push_back(succ_id);
            }
            costs->push_back(30);
        }
    }

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        RobotState final_robot_state;
        if (!checkAction(parent_entry->state, action, &final_robot_state)) {
            continue;
        }

        auto& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // E_orig from V_orig
        {
            // check if hash entry already exists, if not then create one
            auto succ_id = createState(succ_coord);
            auto* succ_state = getState(succ_id);
            succ_state->state = final_robot_state;

            // put successor on successor list with the proper cost
            if (!unique && this->isGoal(final_state, final_robot_state)) {
                succs->push_back(m_goal_state_id);
            } else {
                succs->push_back(succ_id);
            }

            auto edge_cost = computeCost(*parent_entry, *succ_state);
            costs->push_back(edge_cost);

            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "      succ: %d", succ_id);
            SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_state->coord);
            SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_state->state);
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG,        "        cost: %5d", edge_cost);
        }

        // E_z from V_orig
        if (!parent_nearby_nodes.empty()) {
            auto pose_coord = GetPoseCoord(succ_coord);
            auto it = this->psi_to_egraph_nodes.find(pose_coord);
            if (it != end(this->psi_to_egraph_nodes)) {
                // for each experience graph node within error...
                for (auto node : it->second) {
                    // if node in the adjacent list of any nearby node...
                    for (auto nn : parent_nearby_nodes) {
                        if (this->egraph.edge(nn, node)) {
                            auto z = this->egraph.state(node).back();
                            auto this_final_state = final_state;
                            auto this_final_robot_state = final_robot_state;
                            this_final_state.back() = z;
                            this_final_robot_state.back() = z;
                            WorkspaceCoord succ_coord;
                            this->stateWorkspaceToCoord(this_final_state, succ_coord);
                            auto succ_id = createState(succ_coord);
                            auto* succ_state = getState(succ_id);
                            succ_state->state = this_final_robot_state;
                            SMPL_DEBUG_NAMED(G_LOG, "Return Z-EDGE z = %f", z);
                            if (!unique && this->isGoal(this_final_state, this_final_robot_state)) {
                                succs->push_back(this->getGoalStateID());
                            } else {
                                succs->push_back(succ_id);
                            }
                            auto edge_cost = computeCost(*parent_entry, *succ_state);
                            costs->push_back(edge_cost);
                        }
                    }
                }
            }
        }
    }
}

bool WorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << ids);

    if (ids.empty()) return true;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (ids.size() == 1) {
        auto state_id = ids[0];

        if (state_id == this->getGoalStateID()) {
            auto* entry = this->getState(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", this->getStartStateID());
                return false;
            }
            path.push_back(entry->state);
        } else {
            auto* entry = this->getState(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (ids[0] == this->getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<RobotState> opath;

    // grab the first point
    {
        auto* entry = this->getState(ids[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", ids[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == this->getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        // find the successor state corresponding to the cheapest valid action

        // TODO: return an iterator here to avoid collision checking all
        // successors
        std::vector<int> succs, costs;
        GetSuccs(prev_id, &succs, &costs, true);

        WorkspaceLatticeState* best_state = NULL;
        auto best_cost = std::numeric_limits<int>::max();
        for (size_t i = 0; i < succs.size(); ++i) {
            if (curr_id == this->getGoalStateID()) {
                auto* state = this->getState(succs[i]);
                WorkspaceState workspace_state;
                this->stateRobotToWorkspace(state->state, workspace_state);
                if (costs[i] < best_cost && this->isGoal(workspace_state, state->state)) {
                    best_state = state;
                    best_cost = costs[i];
                }
            } else {
                if (succs[i] == curr_id && costs[i] < best_cost) {
                    best_state = this->getState(succs[i]);
                    best_cost = costs[i];
                }
            }
        }

        if (best_state != NULL) {
            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Extract successor state " << best_state->state);
            opath.push_back(best_state->state);
            continue;
        }

        SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut successor");

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

            SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, prev_node, curr_node);

            std::vector<ExperienceGraph::node_id> node_path;
            found = FindShortestExperienceGraphPath(this->egraph, prev_node, curr_node, node_path);
            if (found) {
                for (auto n : node_path) {
                    auto state_id = this->egraph_node_to_state[n];
                    auto* entry = this->getState(state_id);
                    assert(entry);
                    opath.push_back(entry->state);
                }
            }
        }

        if (found) continue;

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            SMPL_DEBUG_NAMED(G_LOG, "Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            if (!GetSnapMotion(this, prev_id, curr_id, opath)) {

            }

//            auto* entry = this->getState(curr_id);
//            assert(entry);
//            opath.push_back(entry->state);
            continue;
        }



        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid successor during path extraction");
//        return false;
        {
            auto* giveup = this->getState(curr_id);
            opath.push_back(giveup->state);
        }
    }

    // we made it!
    path = std::move(opath);

    SMPL_DEBUG_NAMED(G_LOG, "Final path:");
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

        SMPL_DEBUG_NAMED(G_LOG, "Create hash entries for experience graph states");

        // create the first state
        // 1. insert continuous state into the ExperienceGraph
        // 2. map from discrete coordinate to ExperienceGraph node id
        // 3. create entry in the state table for this state
        // 4. assign the discrete coordinates to the state
        // 5. map from ExperienceGraph node id to state id
        // 6. map from state id to ExperienceGraph node id
        auto& prev_pt = egraph_states.front();
        WorkspaceCoord prev_disc_pt(this->dofCount());
        this->stateRobotToCoord(prev_pt, prev_disc_pt);

        auto prev_node_id = this->egraph.insert_node(prev_pt);
        {
            SMPL_DEBUG_NAMED(G_LOG, "xyz = %d, %d, %d, %d, %d, %d",
                    prev_disc_pt[0],
                    prev_disc_pt[1],
                    prev_disc_pt[2],
                    prev_disc_pt[3],
                    prev_disc_pt[4],
                    prev_disc_pt[5]);
            this->psi_to_egraph_nodes[GetPoseCoord(prev_disc_pt)].push_back(prev_node_id);
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
            SMPL_DEBUG_NAMED(G_LOG, "xyz = %d, %d, %d, %d, %d, %d",
                    disc_pt[0],
                    disc_pt[1],
                    disc_pt[2],
                    disc_pt[3],
                    disc_pt[4],
                    disc_pt[5]);
                this->psi_to_egraph_nodes[GetPoseCoord(disc_pt)].push_back(node_id);
                this->coord_to_egraph_nodes[disc_pt].push_back(prev_node_id);

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

    SMPL_DEBUG_NAMED(G_LOG, "Experience graph contains %zu nodes and %zu edges", this->egraph.num_nodes(), this->egraph.num_edges());
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

    SMPL_DEBUG_NAMED(G_LOG, "  shortcut %d -> %d!", src_id, dst_id);
    cost = 10;
    return true;
}

bool WorkspaceLatticeEGraph::snap(int src_id, int dst_id, int& cost)
{
    auto* src_state = this->getState(src_id);
    auto* dst_state = this->getState(dst_id);
    assert(src_state != NULL && dst_state != NULL);

    SMPL_DEBUG_STREAM("Snap " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(src_state->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(dst_state->state, "snap_to"));

    // interpolate between the src and the destination, maintaining the
    // end effector position

    auto dx = dst_state->state[0] - src_state->state[0];
    auto dy = dst_state->state[1] - src_state->state[1];
    if (std::fabs(dx) > 1e-6 || std::fabs(dy) > 1e-6) {
        auto heading = atan2(dy, dx);
        auto alt_heading = heading + M_PI;
        if (angles::shortest_angle_dist(heading, src_state->state[2]) >
                angles::to_radians(10.0) &&
            angles::shortest_angle_dist(alt_heading, src_state->state[2]) >
                angles::to_radians(10.0))
        {
            SMPL_DEBUG_NAMED(G_LOG, "SKIP SNAP MOTION");
//            return false;
        }
    }

#if 1
    WorkspaceState start_state;
    WorkspaceState finish_state;
    this->stateCoordToWorkspace(src_state->coord, start_state);
    this->stateCoordToWorkspace(dst_state->coord, finish_state);
    int num_waypoints = 10;
    for (int i = 0; i < num_waypoints; ++i) {
        WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(this->dofCount());

        // x, y, z, R, P, Y, FA1 same as start
        interm_workspace_state[0] = finish_state[0];
        interm_workspace_state[1] = finish_state[1];
        interm_workspace_state[2] = finish_state[2];
        interm_workspace_state[3] = finish_state[3];
        interm_workspace_state[4] = finish_state[4];
        interm_workspace_state[5] = finish_state[5];
        interm_workspace_state[6] = finish_state[6];

        auto interp = [](double a, double b, double t) {
            return (1.0 - t) * a + t * b;
        };

        auto t = (double)i / (double)(num_waypoints - 1);
        // interpolate torso, theta, x, y
        interm_workspace_state[7] = interp(start_state[7], finish_state[7], t);
        interm_workspace_state[8] = interp(start_state[8], finish_state[8], t);
        interm_workspace_state[9] = interp(start_state[9], finish_state[9], t);
        interm_workspace_state[10] = interp(start_state[10], finish_state[10], t);

        // hinge kept the same
        interm_workspace_state[11] = finish_state[11];

        auto stateWorkspaceToRobotPermissive = [&](
            const WorkspaceState& state,
            RobotState& ostate)
        {
            RobotState seed = src_state->state; //(robot()->jointVariableCount(), 0);
            for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
                seed[m_fangle_indices[fai]] = state[6 + fai];
            }

            Eigen::Affine3d pose =
                    Eigen::Translation3d(state[0], state[1], state[2]) *
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX());

//            return m_ik_iface->computeIK(pose, dst_state->state, ostate);
            return m_ik_iface->computeIK(pose, seed, ostate);
        };

        RobotState robot_state;
        // TODO: this should be permissive and allow moving the redundant angles
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
            SMPL_WARN("Failed to find ik solution for interpolated state");
            return false;
        }
    }
#endif

    if (!this->collisionChecker()->isStateToStateValid(
            src_state->state, dst_state->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "  Snap %d -> %d!", src_id, dst_id);
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

} // namespace smpl

