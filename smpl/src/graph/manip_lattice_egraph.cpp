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

#include <smpl/graph/manip_lattice_egraph.h>

// standard includes
#include <algorithm>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

// system includes
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/csv_parser.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/action_space.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>

namespace smpl {

auto ManipLatticeEGraph::RobotCoordHash::operator()(const argument_type& s) const ->
    result_type
{
    auto seed = (size_t)0;
    boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
    return seed;
}

static
bool FindShortestExperienceGraphPath(
    ManipLatticeEGraph* lattice,
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path)
{
    struct ExperienceGraphSearchNode : heap_element
    {
        int g;
        bool closed;
        ExperienceGraphSearchNode* bp;
        ExperienceGraphSearchNode() :
            g(std::numeric_limits<int>::max()),
            closed(false),
            bp(NULL)
        { }
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

    auto search_nodes = std::vector<ExperienceGraphSearchNode>(lattice->m_egraph.num_nodes());

    auto open = heap_type();

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    auto exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        auto* min = open.min();
        open.pop();
        min->closed = true;

        if (min == &search_nodes[goal_node]) {
            SMPL_ERROR("Found shortest experience graph path");
            ExperienceGraphSearchNode* ps = NULL;
            for (auto* s = &search_nodes[goal_node]; s; s = s->bp) {
                if (s != ps) {
                    path.push_back(std::distance(search_nodes.data(), s));
                    ps = s;
                } else {
                    SMPL_ERROR("Cycle detected!");
                }
            }
            std::reverse(begin(path), end(path));
            return true;
        }

        auto n = (ExperienceGraph::node_id)std::distance(search_nodes.data(), min);
        auto adj = lattice->m_egraph.adjacent_nodes(n);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto& succ = search_nodes[*ait];
            if (succ.closed) {
                continue;
            }
            auto new_cost = min->g + 1;
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
    const ManipLatticeEGraph* lattice,
    const std::string& filepath,
    std::vector<RobotState>& egraph_states)
{
    auto fin = std::ifstream(filepath);
    if (!fin.is_open()) {
        return false;
    }

    auto parser = CSVParser();
    auto with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parsed experience graph file");
    SMPL_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_INFO("  %zu records", parser.recordCount());
    SMPL_INFO("  %zu fields", parser.fieldCount());

    auto jvar_count = lattice->GetRobotModel()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        SMPL_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
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

    SMPL_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

bool ManipLatticeEGraph::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const std::vector<double>& resolutions,
    ActionSpace* actions,
    CostFunction* cost_fun)
{
    if (!m_lattice.Init(robot, checker, resolutions, actions, cost_fun)) {
        return false;
    }

    return DiscreteSpace::Init(robot, checker);
}

void ManipLatticeEGraph::PrintState(int state_id, bool verbose, FILE* f)
{
    return m_lattice.PrintState(state_id, verbose, f);
}

int ManipLatticeEGraph::GetStateID(const RobotState& state)
{
    return m_lattice.GetStateID(state);
}

bool ManipLatticeEGraph::ExtractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << idpath);
    if (idpath.empty()) {
        return true;
    }

    assert(idpath.size() > 1);

    auto opath = std::vector<RobotState>();

    // grab the first point
    {
//        assert(IsValidStateID(this, idpath[0]));
        auto* first_state = m_lattice.GetHashEntry(idpath[0]);
        if (!first_state) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(first_state->state);
    }

    // grab the rest of the points
    for (auto i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        auto* prev_state = m_lattice.GetHashEntry(prev_id);

        auto best_action = m_lattice.FindBestAction(prev_id, curr_id);
        if (best_action.action_id >= 0) {
            for (auto& wp : best_action.motion) {
                opath.push_back(wp);
            }
            continue;
        }

        // check for shortcut transition
        auto pnit = std::find(begin(m_egraph_state_ids), end(m_egraph_state_ids), prev_id);
        auto cnit = std::find(begin(m_egraph_state_ids), end(m_egraph_state_ids), curr_id);
        if (pnit != end(m_egraph_state_ids) && cnit != end(m_egraph_state_ids)) {
            auto pn = (ExperienceGraph::node_id)std::distance(m_egraph_state_ids.begin(), pnit);
            auto cn = (ExperienceGraph::node_id)std::distance(m_egraph_state_ids.begin(), cnit);

            SMPL_INFO("Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, pn, cn);

            auto node_path = std::vector<ExperienceGraph::node_id>();
            if (FindShortestExperienceGraphPath(this, pn, cn, node_path)) {
                for (auto n : node_path) {
                    auto state_id = m_egraph_state_ids[n];
                    auto* entry = m_lattice.GetHashEntry(state_id);
                    assert(entry != NULL);
                    opath.push_back(entry->state);
                }
                continue;
            }
        }

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (Snap(prev_id, curr_id, cost)) {
            SMPL_ERROR("Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            auto* entry = m_lattice.GetHashEntry(curr_id);
            assert(entry);
            opath.push_back(entry->state);
            continue;
        }

        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid goal successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(path.back(), vis_name));
    return true;
}

void ManipLatticeEGraph::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    return m_lattice.GetSuccs(state_id, succs, costs);
}

auto ManipLatticeEGraph::ProjectToPose(int state_id) -> Affine3
{
    return m_project_to_pose->ProjectToPose(state_id);
}

auto ManipLatticeEGraph::ExtractState(int state_id) -> const RobotState&
{
    return m_lattice.ExtractState(state_id);
}

bool ManipLatticeEGraph::LoadExperienceGraph(const std::string& path)
{
    SMPL_INFO("Load Experience Graph at %s", path.c_str());

    auto p = boost::filesystem::path(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        auto egraph_states = std::vector<RobotState>();
        if (!ParseExperienceGraphFile(this, filepath, egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) {
            continue;
        }

        SMPL_INFO("Create hash entries for experience graph states");

        auto& pp = egraph_states.front();  // previous robot state
        auto pdp = RobotCoord(GetRobotModel()->jointVariableCount()); // previous robot coord
        m_lattice.StateToCoord(egraph_states.front(), pdp);

        auto pid = m_egraph.insert_node(pp);
        m_coord_to_nodes[pdp].push_back(pid);

        auto entry_id = m_lattice.ReserveHashEntry();
        auto* entry = m_lattice.GetHashEntry(entry_id);
        entry->coord = pdp;
        entry->state = pp;

        // map state id <-> experience graph state
        m_egraph_state_ids.resize(pid + 1, -1);
        m_egraph_state_ids[pid] = entry_id;
        m_state_to_node[entry_id] = pid;

        auto edge_data = std::vector<RobotState>();
        for (auto i = 1; i < egraph_states.size(); ++i) {
            auto& p = egraph_states[i];
            auto dp = RobotCoord(GetRobotModel()->jointVariableCount());
            m_lattice.StateToCoord(p, dp);
            if (dp != pdp) {
                // found a new discrete state along the path

                auto id = m_egraph.insert_node(p);
                m_coord_to_nodes[dp].push_back(id);

                int entry_id = m_lattice.ReserveHashEntry();
                auto* entry = m_lattice.GetHashEntry(entry_id);
                entry->coord = dp;
                entry->state = p;

                m_egraph_state_ids.resize(id + 1, -1);
                m_egraph_state_ids[id] = entry_id;
                m_state_to_node[entry_id] = id;
                m_egraph.insert_edge(pid, id, edge_data);

                pdp = dp;
                pid = id;
                edge_data.clear();
            } else {
                // gather intermediate robot states
                edge_data.push_back(p);
            }
        }
    }

    SMPL_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void ManipLatticeEGraph::GetExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = m_state_to_node.find(state_id);
    if (it != m_state_to_node.end()) {
        nodes.push_back(it->second);
    }
}

bool ManipLatticeEGraph::Shortcut(int first_id, int second_id, int& cost)
{
    auto* first_state = m_lattice.GetHashEntry(first_id);
    auto* second_state = m_lattice.GetHashEntry(second_id);
    assert(first_state != NULL && second_state != NULL);

    SMPL_INFO_STREAM("Shortcut " << first_state->state << " -> " << second_state->state);
    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(first_state->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(second_state->state, "shortcut_to"));

    SMPL_INFO("  Shortcut %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

bool ManipLatticeEGraph::Snap(
    int first_id,
    int second_id,
    int& cost)
{
    auto* first_entry = m_lattice.GetHashEntry(first_id);
    auto* second_entry = m_lattice.GetHashEntry(second_id);
    assert(first_entry != NULL && second_entry != NULL);

    SMPL_INFO_STREAM("Snap " << first_entry->state << " -> " << second_entry->state);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(first_entry->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, m_lattice.GetStateVisualization(second_entry->state, "snap_to"));

    if (!GetCollisionChecker()->isStateToStateValid(first_entry->state, second_entry->state)) {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_INFO("  Snap %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

auto ManipLatticeEGraph::GetExperienceGraph() const -> const ExperienceGraph*
{
    return &m_egraph;
}

auto ManipLatticeEGraph::GetExperienceGraph() -> ExperienceGraph*
{
    return &m_egraph;
}

int ManipLatticeEGraph::GetStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_state_ids.size()) {
        return -1;
    } else {
        return m_egraph_state_ids[n];
    }
}

bool ManipLatticeEGraph::UpdateGoal(GoalConstraint* goal)
{
    return m_lattice.UpdateGoal(goal);
}

auto ManipLatticeEGraph::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<IExperienceGraph>() ||
        class_code == GetClassCode<IExtractRobotState>())
    {
        return this;
    }

    if (class_code == GetClassCode<IProjectToPose>()) {
        if (m_project_to_pose == NULL) {
            auto* project_to_pose = m_lattice.GetExtension<IProjectToPose>();
            if (project_to_pose != NULL) {
                m_project_to_pose = project_to_pose;
                return this;
            }
        } else {
            return this;
        }
    }

    return NULL;
}

} // namespace smpl
