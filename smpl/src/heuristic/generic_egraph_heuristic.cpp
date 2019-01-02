////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#include <smpl/heuristic/generic_egraph_heuristic.h>

// project includes
#include <smpl/console/console.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/experience_graph_extension.h>

namespace smpl {

static const char* LOG = "heuristic.generic_egraph";

bool GenericEGraphHeuristic::Init(DiscreteSpace* space, Heuristic* h)
{
    if (h == NULL) {
        return false;
    }

    auto egraph = space->GetExtension<IExperienceGraph>();
    if (egraph == NULL) {
        return false;
    }

    auto pairwise = h->GetExtension<IPairwiseHeuristic>();
    if (pairwise == NULL) {
        return false;
    }

    auto goal_heuristic = h->GetExtension<IGoalHeuristic>();
    if (goal_heuristic == NULL) {
        return false;
    }

    auto metric_start = h->GetExtension<IMetricStartHeuristic>();
    auto metric_goal = h->GetExtension<IMetricGoalHeuristic>();

    if (!Heuristic::Init(space)) {
        return false;
    }

    m_orig_h = h;
    m_pairwise_h = pairwise;
    m_metric_start_h = metric_start;
    m_metric_goal_h = metric_goal;
    m_eg = egraph;
    return true;
}

auto GenericEGraphHeuristic::GetEGraphWeight() const -> double
{
    return m_eg_eps;
}

void GenericEGraphHeuristic::SetEGraphWeight(double w)
{
    m_eg_eps = w;
}

void GenericEGraphHeuristic::GetEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    auto* eg = m_eg->GetExperienceGraph();
    auto nodes = eg->nodes();
    auto equiv_thresh = 100;
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto egraph_state_id = m_eg->GetStateID(*nit);
        auto h = m_pairwise_h->GetPairwiseHeuristic(state_id, egraph_state_id);
        if (h <= equiv_thresh) {
            ids.push_back(egraph_state_id);
        }
    }
}

void GenericEGraphHeuristic::GetShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    auto egraph_nodes = std::vector<ExperienceGraph::node_id>();
    m_eg->GetExperienceGraphNodes(state_id, egraph_nodes);

    for (auto n : egraph_nodes) {
        auto comp_id = m_component_ids[n];
        for (auto nn : m_shortcut_nodes[comp_id]) {
            auto egraph_state_id = m_eg->GetStateID(nn);
            if (state_id != egraph_state_id) {
                shortcut_ids.push_back(egraph_state_id);
            }
        }
    }
}

int GenericEGraphHeuristic::GetGoalHeuristic(int state_id)
{
    auto* eg = m_eg->GetExperienceGraph();
    assert(eg != NULL);

    auto best_h = (int)(m_eg_eps * m_goal_h->GetGoalHeuristic(state_id));
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto egraph_state_id = m_eg->GetStateID(*nit);
        auto h = m_pairwise_h->GetPairwiseHeuristic(state_id, egraph_state_id);
        auto dist = m_h_nodes[*nit + 1].dist;
        auto new_h = dist + (int)(m_eg_eps * h);
        if (new_h < best_h) {
            best_h = new_h;
        }
    }

    return best_h;
}

bool GenericEGraphHeuristic::UpdateStart(int state_id)
{
    return m_orig_h->UpdateStart(state_id);
}

bool GenericEGraphHeuristic::UpdateGoal(GoalConstraint* goal)
{
    if (!m_orig_h->UpdateGoal(goal)) {
        return false;
    }

    auto* eg = m_eg->GetExperienceGraph();
    assert(eg != NULL);

    //////////////////////////////////////////////////////////
    // Compute Connected Components of the Experience Graph //
    //////////////////////////////////////////////////////////

    auto comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (m_component_ids[*nit] != -1) {
            continue;
        }

        auto frontier = std::vector<ExperienceGraph::node_id>();
        frontier.push_back(*nit);
        while (!frontier.empty()) {
            auto n = frontier.back();
            frontier.pop_back();

            m_component_ids[n] = comp_count;

            auto adj = eg->adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                if (m_component_ids[*ait] == -1) {
                    frontier.push_back(*ait);
                }
            }
        }

        ++comp_count;
    }

    SMPL_INFO_NAMED(LOG, "Experience graph contains %d connected components", comp_count);

    ////////////////////////////
    // Compute Shortcut Nodes //
    ////////////////////////////

    m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
    auto shortcut_heuristics = std::vector<int>(comp_count);
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto n = *nit;
        auto comp_id = m_component_ids[n];
        auto state_id = m_eg->GetStateID(n);

        auto h = m_goal_h->GetGoalHeuristic(state_id);

        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(n);
            shortcut_heuristics[comp_id] = h;
        } else {
            int best_h = shortcut_heuristics[comp_id];
            if (h < best_h) {
                m_shortcut_nodes[comp_id].clear();
                m_shortcut_nodes[comp_id].push_back(n);
                shortcut_heuristics[comp_id] = h;
            } else if (h == best_h) {
                m_shortcut_nodes[comp_id].push_back(n);
            }
        }
    }

    ////////////////////////////////////////////////////////////
    // Compute Heuristic Distances for Experience Graph Nodes //
    ////////////////////////////////////////////////////////////

    m_h_nodes.assign(eg->num_nodes() + 1, HeuristicNode(Unknown));
    m_open.clear();
    m_h_nodes[0].dist = 0;
    m_open.push(&m_h_nodes[0]);
    while (!m_open.empty()) {
        auto* s = m_open.min();
        m_open.pop();

        auto nidx = std::distance(m_h_nodes.data(), s);
        if (nidx == 0) {
            // neighbors: inflated edges to all experience graph states
            // unconditionally relaxed (goal node is the first node removed)
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                auto nid = *nit;
                auto* n = &m_h_nodes[nid + 1];
                auto state_id = m_eg->GetStateID(nid);
                auto h = m_goal_h->GetGoalHeuristic(state_id);
                n->dist = (int)(m_eg_eps * (double)h);
                m_open.push(n);
            }
        } else {
            // neighbors: inflated edges to all non-adjacent experience graph
            // states original cost edges to all adjacent experience graph
            // states
            auto sid = nidx - 1;
            auto s_state_id = m_eg->GetStateID(sid);
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                auto nid = *nit;
                auto* n = &m_h_nodes[nid + 1];
                if (eg->edge(sid, nid)) {
                    auto edge_cost = 10;
                    auto new_cost = s->dist + edge_cost;
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                } else {
                    auto n_state_id = m_eg->GetStateID(nid);
                    auto h = m_pairwise_h->GetPairwiseHeuristic(s_state_id, n_state_id);
                    auto new_cost = s->dist + (int)(m_eg_eps * h);
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                }
            }
        }
    }

    return true;
}

auto GenericEGraphHeuristic::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>() ||
        class_code == GetClassCode<IExperienceGraphHeuristic>())
    {
        return this;
    }

    if (class_code == GetClassCode<IMetricStartHeuristic>()) {
        if (m_metric_start_h != NULL) return this;
    }
    if (class_code == GetClassCode<IMetricGoalHeuristic>()) {
        if (m_metric_goal_h != NULL) return this;
    }

    return NULL;
}

auto GenericEGraphHeuristic::GetMetricStartDistance(
    double x, double y, double z) -> double
{
    return m_metric_start_h->GetMetricStartDistance(x, y, z);
}

auto GenericEGraphHeuristic::GetMetricGoalDistance(
    double x, double y, double z) -> double
{
    return m_metric_goal_h->GetMetricGoalDistance(x, y, z);
}

} // namespace smpl
