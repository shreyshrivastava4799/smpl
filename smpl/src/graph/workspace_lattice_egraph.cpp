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
#include <smpl/planning_params.h>

namespace smpl {

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

    auto parser = CSVParser();
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_WARN("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "Parsed experience graph file");
    SMPL_DEBUG_NAMED(G_LOG, "  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_DEBUG_NAMED(G_LOG, "  %zu records", parser.recordCount());
    SMPL_DEBUG_NAMED(G_LOG, "  %zu fields", parser.fieldCount());

    auto jvar_count = robot_model->GetPlanningJoints().size();
    if (parser.fieldCount() < jvar_count) {
        SMPL_WARN("Parsed experience graph contains insufficient number of joint variables (%zu < %zu)", parser.fieldCount(), jvar_count);
        return false;
    }
    if (parser.fieldCount() > jvar_count) {
        SMPL_WARN("Parsed experience graph contains superflous many joint variables (%zu > %zu)", parser.fieldCount(), jvar_count);
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (auto i = 0; i < parser.recordCount(); ++i) {
        auto state = RobotState(jvar_count);
        for (auto j = 0; j < parser.fieldCount(); ++j) {
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

bool WorkspaceLatticeEGraph::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const WorkspaceProjectionParams& params,
    WorkspaceLatticeActionSpace* actions)
{
    if (!DiscreteSpace::Init(robot, checker)) {
        return false;
    }

    if (!m_lattice.Init(robot, checker, params, actions)) {
        return false;
    }

    return true;
}

void WorkspaceLatticeEGraph::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    auto* state = m_lattice.GetState(state_id);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, m_lattice.GetStateVisualization(state->state, vis_name));

    auto is_egraph_node = false;
    ExperienceGraph::node_id egraph_node;
    {
        auto it = m_state_to_egraph_node.find(state_id);
        if (it != end(m_state_to_egraph_node)) {
            is_egraph_node = true;
            egraph_node = it->second;
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  egraph state: %s", is_egraph_node ? "true" : "false");

    if (is_egraph_node) { // expanding an egraph node
        return GetEGraphStateSuccs(state, egraph_node, succs, costs);
    } else {
        return GetOrigStateSuccs(state, succs, costs);
    }
}

void WorkspaceLatticeEGraph::GetEGraphStateSuccs(
    WorkspaceLatticeState* state,
    ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    GetEGraphStateBridgeSuccs(state, egraph_node, succs, costs);
    GetEGraphStateAdjacentSuccs(state, egraph_node, succs, costs);
}

void WorkspaceLatticeEGraph::GetEGraphStateAdjacentSuccs(
    WorkspaceLatticeState* state,
    ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto& egraph_state = m_egraph.state(egraph_node);

    // E_demo from V_demo
    auto adj = m_egraph.adjacent_nodes(egraph_node);
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  %td adjacent experience graph edges", std::distance(adj.first, adj.second));
    for (auto ait = adj.first; ait != adj.second; ++ait) {
        auto& adj_egraph_state = m_egraph.state(*ait);
        auto workspace_state = WorkspaceState();
        StateRobotToWorkspace(&m_lattice.m_proj, adj_egraph_state, workspace_state);
        succs->push_back(m_egraph_node_to_state[*ait]);
        costs->push_back(10);
    }
}

void WorkspaceLatticeEGraph::GetEGraphStateBridgeSuccs(
    WorkspaceLatticeState* state,
    ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto& egraph_state = m_egraph.state(egraph_node);

    // E_bridge from V_demo
    {
        // egraph robot state -> workspace state
        auto workspace_state = WorkspaceState();
        StateRobotToWorkspace(&m_lattice.m_proj, egraph_state, workspace_state);

        // discrete successor state
        auto workspace_coord = WorkspaceCoord();
        StateWorkspaceToCoord(&m_lattice.m_proj, workspace_state, workspace_coord);

        // successor robot state
        auto robot_state = RobotState();
        if (StateWorkspaceToRobot(&m_lattice.m_proj, workspace_state, egraph_state, robot_state)) {
            // inherit the exact z value from the e-graph state

            // TODO: check action

            auto succ_id = m_lattice.GetOrCreateState(workspace_coord);
            auto* succ_state = m_lattice.GetState(succ_id);
            succ_state->state = robot_state;

            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  bridge edge to %d", succ_id);

            succs->push_back(succ_id);

            costs->push_back(m_lattice.ComputeCost(*state, *succ_state));
        } else {
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  bridge edge failed");
        }
    }
}

void WorkspaceLatticeEGraph::GetOrigStateSuccs(
    smpl::WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    GetOrigStateBridgeSuccs(state, succs, costs);
    GetOrigStateOrigSuccs(state, succs, costs);
}

void WorkspaceLatticeEGraph::GetOrigStateBridgeSuccs(
    WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    // E_bridge from V_orig
    auto it = m_coord_to_egraph_nodes.find(state->coord);
    if (it != end(m_coord_to_egraph_nodes)) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  bridge to %zu e-graph nodes", it->second.size());
        for (auto node : it->second) {
            auto succ_id = m_egraph_node_to_state[node];
            auto& egraph_state = m_egraph.state(node);
            smpl::WorkspaceState workspace_state;
            StateRobotToWorkspace(&m_lattice.m_proj, egraph_state, workspace_state);
            succs->push_back(succ_id);
            costs->push_back(30);
        }
    }
}

void WorkspaceLatticeEGraph::GetOrigStateOrigSuccs(
    WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto actions = std::vector<smpl::WorkspaceAction>();
    m_lattice.m_actions->Apply(*state, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      waypoints: %zu", action.size());

        auto final_robot_state = RobotState();
        if (!m_lattice.CheckAction(state->state, action, &final_robot_state)) {
            continue;
        }

        auto& final_workspace_state = action.back();
        auto succ_coord = smpl::WorkspaceCoord();
        StateWorkspaceToCoord(&m_lattice.m_proj, final_workspace_state, succ_coord);

        // check if hash entry already exists, if not then create one
        auto succ_id = m_lattice.GetOrCreateState(succ_coord);
        auto* succ_state = m_lattice.GetState(succ_id);
        succ_state->state = final_robot_state;

        // put successor on successor list with the proper cost
        succs->push_back(succ_id);

        auto edge_cost = m_lattice.ComputeCost(*state, *succ_state);
        costs->push_back(edge_cost);

        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG,        "      succ: %d", succ_id);
        SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "        coord: " << succ_state->coord);
        SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "        state: " << succ_state->state);
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG,        "        cost: %5d", edge_cost);
    }
}

void WorkspaceLatticeEGraph::InsertExperienceGraphPath(
    const std::vector<smpl::RobotState>& path)
{
    if (path.empty()) {
        SMPL_WARN("No experience graph states contained in file");
        return;
    }

    SMPL_DEBUG_NAMED(G_LOG, "Create hash entries for experience graph states");

    // Create an experience graph state for the first state

    auto& first_egraph_state = path.front();

    auto prev_node_id = m_egraph.insert_node(first_egraph_state);

    {
        // map discrete egraph state -> egraph node
        auto tmp = WorkspaceState();
        StateRobotToWorkspace(&m_lattice.m_proj, first_egraph_state, tmp);

        auto disc_egraph_state = WorkspaceCoord(m_lattice.m_proj.dof_count);
        StateWorkspaceToCoord(&m_lattice.m_proj, tmp, disc_egraph_state);
        m_coord_to_egraph_nodes[disc_egraph_state].push_back(prev_node_id);

        // reserve a graph state for this state
        auto state_id = m_lattice.ReserveHashEntry();
        auto* state = m_lattice.GetState(state_id);
        state->coord = disc_egraph_state;
        state->state = first_egraph_state;

        // map egraph node -> graph state
        m_egraph_node_to_state.resize(prev_node_id + 1, -1);
        m_egraph_node_to_state[prev_node_id] = state_id;

        // map graph state -> egraph node
        m_state_to_egraph_node[state_id] = prev_node_id;

        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  egraph state = " << first_egraph_state);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  egraph node = " << prev_node_id);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  disc state = " << disc_egraph_state);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state id = " << state_id);
    }

    // Walk through the demonstration and create a unique e-graph state
    // for each state. Intermediately encountered
    // states that span between two discrete states become the edges in
    // the e-graph.
    for (auto i = 1; i < path.size(); ++i) {
        auto& egraph_state = path[i];

        auto node_id = m_egraph.insert_node(egraph_state);

        auto tmp = WorkspaceState();
        StateRobotToWorkspace(&m_lattice.m_proj, egraph_state, tmp);

        // map discrete egraph state -> egraph node
        auto disc_egraph_state = WorkspaceCoord(m_lattice.m_proj.dof_count);
        StateWorkspaceToCoord(&m_lattice.m_proj, tmp, disc_egraph_state);
        m_coord_to_egraph_nodes[disc_egraph_state].push_back(prev_node_id);

        // reserve a graph state for this state
        auto state_id = m_lattice.ReserveHashEntry();
        auto* state = m_lattice.GetState(state_id);
        state->coord = disc_egraph_state;
        state->state = egraph_state;

        // map egraph node -> graph state
        m_egraph_node_to_state.resize(node_id + 1, -1);
        m_egraph_node_to_state[node_id] = state_id;

        // map graph state -> egraph node
        m_state_to_egraph_node[state_id] = node_id;

        // add edge from previous node
        m_egraph.insert_edge(prev_node_id, node_id, { });

        prev_node_id = node_id;

        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  egraph state = " << egraph_state);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  egraph node = " << prev_node_id);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  disc state = " << disc_egraph_state);
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  state id = " << state_id);
    }
}

void WorkspaceLatticeEGraph::ClearExperienceGraph()
{
    // delete all reserved states corresponding to e-graph states
    for (auto state_id : m_egraph_node_to_state) {
        auto& state = m_lattice.m_states[state_id];
        delete state;
        state = NULL;
    }

    // remove all e-graph states from the state table

    auto first = (decltype(m_lattice.m_states)::size_type)0;
    auto last = m_lattice.m_states.size();

    // find the index of the first non-null state
    while (first != last) {
        if (m_lattice.m_states[first] == NULL) break;
        ++first;
    }

    // shift all non-null states
    if (first != last) {
        for (auto i = first; ++i != last;) {
            if (m_lattice.m_states[i] != NULL) {
                // remap state to id
                m_lattice.m_state_to_id[m_lattice.m_states[i]] = (int)first;
                // shift the state
                m_lattice.m_states[first] = m_lattice.m_states[i];
                ++first;
            }
        }
    }

    m_lattice.m_states.resize(first);

    m_egraph.clear();
    m_coord_to_egraph_nodes.clear();
    m_egraph_node_to_state.clear();
    m_state_to_egraph_node.clear();
}

auto WorkspaceLatticeEGraph::GetVisualizationFrameId() const -> const std::string&
{
    return m_lattice.GetVisualizationFrameId();
}

void WorkspaceLatticeEGraph::SetVisualizationFrameId(const std::string& frame_id)
{
    return m_lattice.SetVisualizationFrameId(frame_id);
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
bool WorkspaceLatticeEGraph::LoadExperienceGraph(const std::string& path)
{
    auto p = boost::filesystem::path(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        auto egraph_path = std::vector<RobotState>();
        if (!ParseExperienceGraphFile(filepath, GetRobotModel(), egraph_path)) {
            continue;
        }

        InsertExperienceGraphPath(egraph_path);
    }

    SMPL_DEBUG_NAMED(G_LOG, "Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void WorkspaceLatticeEGraph::GetExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = m_state_to_egraph_node.find(state_id);
    if (it != end(m_state_to_egraph_node)) {
        nodes.push_back(it->second);
    }
}

bool WorkspaceLatticeEGraph::Shortcut(int src_id, int dst_id, int& cost)
{
    auto* src_state = m_lattice.GetState(src_id);
    auto* dst_state = m_lattice.GetState(dst_id);

    SMPL_INFO_STREAM("Shortcut " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(src_state->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(dst_state->state, "shortcut_to"));

    SMPL_DEBUG_NAMED(G_LOG, "  shortcut %d -> %d!", src_id, dst_id);
    cost = 10;
    return true;
}

bool WorkspaceLatticeEGraph::Snap(int src_id, int dst_id, int& cost)
{
    auto* src_state = m_lattice.GetState(src_id);
    auto* dst_state = m_lattice.GetState(dst_id);
    assert(src_state != NULL && dst_state != NULL);

    SMPL_DEBUG_STREAM("Snap " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(src_state->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(dst_state->state, "snap_to"));

    if (!GetCollisionChecker()->IsStateToStateValid(
            src_state->state, dst_state->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "  Snap %d -> %d!", src_id, dst_id);
    cost = 1000;
    return true;
}

auto WorkspaceLatticeEGraph::GetExperienceGraph() const -> const ExperienceGraph*
{
    return &m_egraph;
}

auto WorkspaceLatticeEGraph::GetExperienceGraph() -> ExperienceGraph*
{
    return &m_egraph;
}

int WorkspaceLatticeEGraph::GetStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_node_to_state.size()) {
        return -1;
    } else {
        return m_egraph_node_to_state[n];
    }
}

int WorkspaceLatticeEGraph::GetStateID(const RobotState& state)
{
    return m_lattice.GetStateID(state);
}

bool WorkspaceLatticeEGraph::ExtractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << ids);

    if (ids.empty()) return true;

    auto opath = std::vector<RobotState>();

    // grab the first point
    {
        auto* entry = m_lattice.GetState(ids[0]);
        if (entry == NULL) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", ids[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (auto i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        // find the successor state corresponding to the cheapest valid action

        // TODO: return an iterator here to avoid collision checking all
        // successors
        std::vector<int> succs, costs;
        GetSuccs(prev_id, &succs, &costs);

        WorkspaceLatticeState* best_state = NULL;
        auto best_cost = std::numeric_limits<int>::max();
        for (auto i = 0; i < succs.size(); ++i) {
            if (succs[i] == curr_id && costs[i] < best_cost) {
                best_state = m_lattice.GetState(succs[i]);
                best_cost = costs[i];
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
        auto pnit = std::find(begin(m_egraph_node_to_state), end(m_egraph_node_to_state), prev_id);
        auto cnit = std::find(begin(m_egraph_node_to_state), end(m_egraph_node_to_state), curr_id);
        if (pnit != end(m_egraph_node_to_state) &&
            cnit != end(m_egraph_node_to_state))
        {
            // position in node array is synonymous with e-graph node id
            auto prev_node = std::distance(begin(m_egraph_node_to_state), pnit);
            auto curr_node = std::distance(begin(m_egraph_node_to_state), cnit);

            SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, prev_node, curr_node);

            auto node_path = std::vector<ExperienceGraph::node_id>();
            found = FindShortestExperienceGraphPath(m_egraph, prev_node, curr_node, node_path);
            if (found) {
                for (auto n : node_path) {
                    auto state_id = m_egraph_node_to_state[n];
                    auto* entry = m_lattice.GetState(state_id);
                    opath.push_back(entry->state);
                }
            }
        }

        if (found) continue;

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (Snap(prev_id, curr_id, cost)) {
            SMPL_DEBUG_NAMED(G_LOG, "Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            auto* curr_state = m_lattice.GetState(curr_id);
            opath.push_back(curr_state->state);
            continue;
        }

        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);

    SMPL_DEBUG_NAMED(G_LOG, "Final path:");
    for (auto& point : path) {
        SMPL_INFO_STREAM("  " << point);
    }

    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(path.back(), vis_name));
    return true;
}

auto WorkspaceLatticeEGraph::ExtractState(int state_id) -> const RobotState&
{
    return m_lattice.ExtractState(state_id);
}

auto WorkspaceLatticeEGraph::ProjectToPose(int state_id) -> Affine3
{
    return m_lattice.ProjectToPose(state_id);
}

auto WorkspaceLatticeEGraph::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<DiscreteSpace>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<IExtractRobotState>() ||
        class_code == GetClassCode<IProjectToPose>() ||
        class_code == GetClassCode<ISearchable>() ||
        class_code == GetClassCode<IExperienceGraph>())
    {
        return this;
    }

    return NULL;
}

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

    using heap_type = intrusive_heap<ExperienceGraphSearchNode, NodeCompare>;

    auto search_nodes = std::vector<ExperienceGraphSearchNode>(egraph.num_nodes());

    auto open = heap_type();

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    auto exp_count = 0;
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

} // namespace smpl

