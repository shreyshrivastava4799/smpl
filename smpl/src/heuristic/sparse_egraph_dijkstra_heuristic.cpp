////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Andrew Dornbush
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

#include <smpl/heuristic/sparse_egraph_dijkstra_heuristic.h>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/colors.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/occupancy_grid.h>
#include <smpl/spatial.h>
#include <smpl/stl/algorithm.h>

namespace smpl {

static const char* LOG = "heuristic.egraph_bfs";

static
auto DiscretizePoint(const OccupancyGrid* grid, const Vector3& p)
    -> Eigen::Vector3i
{
    auto dp = Eigen::Vector3i();
    grid->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    return dp;
}

auto SparseEGraphDijkstra3DHeuristic::Vector3iHash::operator()(
    const argument_type& s) const
    -> result_type
{
    auto seed = (std::size_t)0;
    boost::hash_combine(seed, std::hash<int>()(s.x()));
    boost::hash_combine(seed, std::hash<int>()(s.y()));
    boost::hash_combine(seed, std::hash<int>()(s.z()));
    return seed;
}

bool SparseEGraphDijkstra3DHeuristic::Init(
    DiscreteSpace* space,
    const OccupancyGrid* grid)
{
    if (grid == NULL) {
        return false;
    }

    auto* project_to_point = space->GetExtension<IProjectToPoint>();
    if (project_to_point == NULL) {
        return false;
    }
    auto* egraph = space->GetExtension<IExperienceGraph>();
    if (egraph == NULL) {
        return false;
    }

    if (!Heuristic::Init(space)) {
        return false;
    }

    m_grid = grid;

    m_pp = project_to_point;
    m_eg = egraph;

    auto num_cells_x = m_grid->numCellsX() + 2;
    auto num_cells_y = m_grid->numCellsY() + 2;
    auto num_cells_z = m_grid->numCellsZ() + 2;

    m_dist_grid.resize(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

    SMPL_INFO("Create dijkstra distance grid of size %zu x %zu x %zu", num_cells_x, num_cells_y, num_cells_z);

    SyncGridAndDijkstra();

    auto add_wall = [&](int x, int y, int z) {
        m_dist_grid(x, y, z).dist = Wall;
    };

    // pad distance grid borders with walls
    for (auto y = 0; y < num_cells_y; ++y) {
        for (auto z = 0; z < num_cells_z; ++z) {
            add_wall(0, y, z);
            add_wall(num_cells_x - 1, y, z);
        }
    }
    for (auto x = 1; x < num_cells_x - 1; ++x) {
        for (auto z = 0; z < num_cells_z; ++z) {
            add_wall(x, 0, z);
            add_wall(x, num_cells_y - 1, z);
        }
    }
    for (auto x = 1; x < num_cells_x - 1; ++x) {
        for (auto y = 1; y < num_cells_y - 1; ++y) {
            add_wall(x, y, 0);
            add_wall(x, y, num_cells_z - 1);
        }
    }

    return true;
}

void SparseEGraphDijkstra3DHeuristic::SyncGridAndDijkstra()
{
    auto xc = m_grid->numCellsX();
    auto yc = m_grid->numCellsY();
    auto zc = m_grid->numCellsZ();

    auto cell_count = xc * yc * zc;

    auto wall_count = 0;
    for (auto x = 0; x < m_grid->numCellsX(); ++x) {
    for (auto y = 0; y < m_grid->numCellsY(); ++y) {
    for (auto z = 0; z < m_grid->numCellsZ(); ++z) {
        if (m_grid->getDistance(x, y, z) <= m_inflation_radius) {
            m_dist_grid(x + 1, y + 1, z + 1).dist = Wall;
            ++wall_count;
        }
    }
    }
    }

    SMPL_INFO_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

auto SparseEGraphDijkstra3DHeuristic::GetGrid() const -> const OccupancyGrid*
{
    return m_grid;
}

auto SparseEGraphDijkstra3DHeuristic::GetEGraphWeight() const -> double
{
    return m_eg_eps;
}

void SparseEGraphDijkstra3DHeuristic::SetEGraphWeight(double w)
{
    m_eg_eps = w;
}

auto SparseEGraphDijkstra3DHeuristic::GetInflationRadius() const ->double
{
    return m_inflation_radius;
}

void SparseEGraphDijkstra3DHeuristic::SetInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

auto SparseEGraphDijkstra3DHeuristic::GetWallsVisualization() -> visual::Marker
{
    auto centers = std::vector<Vector3>();
    for (auto x = 0; x < m_grid->numCellsX(); x++) {
    for (auto y = 0; y < m_grid->numCellsY(); y++) {
    for (auto z = 0; z < m_grid->numCellsZ(); z++) {
        if (m_dist_grid.get(x + 1, y + 1, z + 1).dist == Wall) {
            Vector3 p;
            m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    auto cubes_marker = visual::MakeCubesMarker(
            centers,
            m_grid->resolution(),
            visual::MakeColorHexARGB(0xFF6495ED),
            m_grid->getReferenceFrame(),
            "bfs_walls",
            0);

    return cubes_marker;
}

auto SparseEGraphDijkstra3DHeuristic::GetValuesVisualization() -> visual::Marker
{
    SMPL_INFO("Retrieve values visualization");

    if (m_start_state_id < 0) {
        return visual::MakeEmptyMarker();
    }

    auto start_heur = GetGoalHeuristic(m_start_state_id);

    auto max_cost = (int)(1.1 * start_heur);

    auto points = std::vector<Vector3>();
    auto colors = std::vector<visual::Color>();
    for (auto x = 0; x < m_grid->numCellsX(); ++x) {
    for (auto y = 0; y < m_grid->numCellsY(); ++y) {
    for (auto z = 0; z < m_grid->numCellsZ(); ++z) {
        auto dp = Eigen::Vector3i(x, y, z);
        dp += Eigen::Vector3i::Ones();

        auto d = GetGoalHeuristic(dp);
        auto cost_pct = (float)d / (float)max_cost;

        if (cost_pct > 1.0) {
            continue;
        }

        auto color = visual::MakeColorHSV(300.0f - 300.0f * cost_pct);

        color.r = clamp(color.r, 0.0f, 1.0f);
        color.g = clamp(color.g, 0.0f, 1.0f);
        color.b = clamp(color.b, 0.0f, 1.0f);

        Vector3 p;
        m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        colors.push_back(color);
    }
    }
    }

    auto marker = MakeCubesMarker(
            std::move(points),
            0.5 * m_grid->resolution(),
            std::move(colors),
            m_grid->getReferenceFrame(),
            "h_values");

    SMPL_INFO("Retrieved values visualization with %zu points", boost::get<visual::CubeList>(marker.shape).points.size());
    return marker;
}

void SparseEGraphDijkstra3DHeuristic::GetEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    auto p = m_pp->ProjectToPoint(state_id);
    auto dp = DiscretizePoint(m_grid, p);
    if (!m_grid->isInBounds(dp.x(), dp.y(), dp.z())) {
        return;
    }
    dp += Eigen::Vector3i::Ones();

    auto hit = m_heur_nodes.find(dp);
    if (hit == m_heur_nodes.end()) {
        return;
    }

    for (auto n : hit->second.up_nodes) {
        int id = m_eg->GetStateID(n);
        if (id != state_id) {
            ids.push_back(id);
        }
    }
}

void SparseEGraphDijkstra3DHeuristic::GetShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    auto egraph_nodes = std::vector<ExperienceGraph::node_id>();
    m_eg->GetExperienceGraphNodes(state_id, egraph_nodes);

    for (auto n : egraph_nodes) {
        auto comp_id = m_component_ids[n];
        for (auto nn : m_shortcut_nodes[comp_id]) {
            int id = m_eg->GetStateID(nn);
            if (id != state_id) {
                shortcut_ids.push_back(id);
            }
        }
    }
}

int SparseEGraphDijkstra3DHeuristic::GetGoalHeuristic(int state_id)
{
    // project and discretize state
    auto p = m_pp->ProjectToPoint(state_id);
    auto dp = DiscretizePoint(m_grid, p);

    if (!m_grid->isInBounds(dp.x(), dp.y(), dp.z())) {
        return Infinity;
    }

    dp += Eigen::Vector3i::Ones();
    return GetGoalHeuristic(dp);
}

auto SparseEGraphDijkstra3DHeuristic::GetMetricStartDistance(
    double x, double y, double z) -> double
{
    auto p = m_pp->ProjectToPoint(m_start_state_id);
    auto s = DiscretizePoint(m_grid, p);
    auto g = DiscretizePoint(m_grid, Vector3(x, y, z));
    // compute the manhattan distance to the start cell
    auto dx = Eigen::Vector3i(s - g);
    return m_grid->resolution() * (abs(dx.x()) + abs(dx.y()) + abs(dx.z()));
}

auto SparseEGraphDijkstra3DHeuristic::GetMetricGoalDistance(
    double x, double y, double z) -> double
{
    auto gp = m_get_goal_position->GetPosition();
    auto dgp = DiscretizePoint(m_grid, gp);
    auto dp = DiscretizePoint(m_grid, Vector3(x, y, z));
    auto d = Eigen::Vector3i(dgp - dp);
    return m_grid->resolution() * (abs(d.x()) + abs(d.y() + abs(d.z())));
}

bool SparseEGraphDijkstra3DHeuristic::UpdateStart(int state_id)
{
    m_start_state_id = state_id;
    return true;
}

bool SparseEGraphDijkstra3DHeuristic::UpdateGoal(GoalConstraint* goal)
{
    SMPL_INFO_NAMED(LOG, "Update EGraphBfsHeuristic goal");

    auto* get_goal_position = goal->GetExtension<IGetPosition>();
    if (get_goal_position == NULL) {
        return false;
    }

    m_get_goal_position = get_goal_position;

    m_open.clear();

    // reset all distances
    for (auto x = 1; x < m_dist_grid.size_x() - 1; ++x) {
    for (auto y = 1; y < m_dist_grid.size_y() - 1; ++y) {
    for (auto z = 1; z < m_dist_grid.size_z() - 1; ++z) {
        if (m_dist_grid.get(x, y, z).dist != Wall) {
            m_dist_grid.set(x, y, z, Cell(Unknown));
        }
    }
    }
    }

    ProjectExperienceGraph();

    auto gp = get_goal_position->GetPosition();

    auto dgp = DiscretizePoint(m_grid, gp);

    // precompute shortcuts
    assert(m_component_ids.size() == m_eg->GetExperienceGraph()->num_nodes());
    auto* eg = m_eg->GetExperienceGraph();
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        int comp_id = m_component_ids[*nit];
        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(*nit);
            continue;
        }

        // get the distance of this node to the goal
        auto p = m_pp->ProjectToPoint(m_eg->GetStateID(*nit));

        auto dist = (gp - p).squaredNorm();

        auto lp = m_pp->ProjectToPoint(m_eg->GetStateID(m_shortcut_nodes[comp_id].front()));

        auto curr_dist = (gp - lp).squaredNorm();

        if (dist < curr_dist) {
            m_shortcut_nodes[comp_id].clear();
            m_shortcut_nodes[comp_id].push_back(*nit);
        } else if (dist == curr_dist) {
            m_shortcut_nodes[comp_id].push_back(*nit);
        }
    }

    m_open.clear();

    if (!m_grid->isInBounds(dgp.x(), dgp.y(), dgp.z())) {
        SMPL_WARN("Cell (%d, %d, %d) is outside heuristic bounds", dgp.x(), dgp.y(), dgp.z());
        return true;
    }

    dgp += Eigen::Vector3i::Ones();

    auto* c = &m_dist_grid(dgp.x(), dgp.y(), dgp.z());
    c->dist = 0;
    m_open.push(c);
    m_open_cell_to_pos[c] = dgp;

    SMPL_INFO_NAMED(LOG, "Updated EGraphBfsHeuristic goal");
    return true;
}

auto SparseEGraphDijkstra3DHeuristic::GetExtension(size_t class_code)
    -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IExperienceGraphHeuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>() ||
        class_code == GetClassCode<IMetricStartHeuristic>() ||
        class_code == GetClassCode<IMetricGoalHeuristic>())
    {
        return this;
    }
    return NULL;
}

// Project experience graph states down to their 3D projections. Note that this
// should be called once the goal is set in the environment as the projection to
// 3D may be based off of the goal condition (for instance, the desired planning
// frame is determined according to the planning link and a fixed offset)
void SparseEGraphDijkstra3DHeuristic::ProjectExperienceGraph()
{
    m_heur_nodes.clear();

    // project experience graph into 3d space (projections of adjacent nodes in
    // the experience graph impose additional edges in 3d, cost equal to the
    // cheapest transition):
    //
    // (1) lookup transitions on-demand when a grid cell is expanded, loop
    // through all experience graph states and their neighbors (method used by
    // origin experience graph code)
    //
    // (2) embed an adjacency list in the dense grid structure as a
    // precomputation
    //
    // (3) maintain an external adjacency list mapping cells with projections
    // from experience graph states to adjacent cells (method used here)
    SMPL_INFO("Project experience graph into three-dimensional grid");
    auto* eg = m_eg->GetExperienceGraph();
    if (eg == NULL) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    auto viz_points = std::vector<Vector3>();

    m_projected_nodes.resize(eg->num_nodes());

    auto proj_node_count = 0;
    auto proj_edge_count = 0;
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        // project experience graph state to point and discretize
        auto first_id = m_eg->GetStateID(*nit);
        SMPL_DEBUG_STREAM_NAMED(LOG, "Project experience graph state " << first_id << " " << eg->state(*nit) << " into 3D");
        auto p = m_pp->ProjectToPoint(first_id);
        SMPL_DEBUG_NAMED(LOG, "Discretize point (%0.3f, %0.3f, %0.3f)", p.x(), p.y(), p.z());
        auto dp = DiscretizePoint(m_grid, p);

        Vector3 viz_pt;
        m_grid->gridToWorld(dp.x(), dp.y(), dp.z(), viz_pt.x(), viz_pt.y(), viz_pt.z());
        viz_points.push_back(viz_pt);

        dp += Eigen::Vector3i::Ones();

        m_projected_nodes[*nit] = dp;

        // insert node into down-projected experience graph
        auto empty = HeuristicNode();
        auto ent = m_heur_nodes.insert(std::make_pair(dp, std::move(empty)));
        auto eit = ent.first;
        if (ent.second) {
            SMPL_DEBUG_NAMED(LOG, "Inserted down-projected cell (%d, %d, %d) into experience graph heuristic", dp.x(), dp.y(), dp.z());
            ++proj_node_count;
        } else {
            SMPL_DEBUG_NAMED(LOG, "Duplicate down-projected cell (%d, %d, %d)", dp.x(), dp.y(), dp.z());
        }

        auto& hnode = eit->second;

        hnode.up_nodes.push_back(*nit);

        auto adj = eg->adjacent_nodes(*nit);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            // project adjacent experience graph state and discretize
            auto second_id = m_eg->GetStateID(*ait);
            SMPL_DEBUG_NAMED(LOG, "  Project experience graph edge to state %d", second_id);
            auto q = m_pp->ProjectToPoint(second_id);
            auto dq = DiscretizePoint(m_grid, q);
            if (!m_grid->isInBounds(dq.x(), dq.y(), dq.z())) {
                continue;
            }
            dq += Eigen::Vector3i::Ones();

            // insert adjacent edge
            if (std::find(hnode.edges.begin(), hnode.edges.end(), dq) ==
                hnode.edges.end())
            {
                ++proj_edge_count;
                SMPL_DEBUG_NAMED(LOG, "  Insert edge to state %d", second_id);
                hnode.edges.push_back(dq);
            } else {
                SMPL_DEBUG_NAMED(LOG, "  Duplicate edge to state %d", second_id);
            }
        }
    }

    SMPL_INFO("Projected experience graph contains %zu nodes and %zu edges", proj_node_count, proj_edge_count);

    auto comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
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

    // pre-allocate shortcuts array here, fill in updateGoal()
    m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
    SMPL_INFO("Experience graph contains %d components", comp_count);

    auto* vis_name = "egraph_projection";
    SV_SHOW_INFO_NAMED(
            vis_name,
            visual::MakeCubesMarker(
                    std::move(viz_points),
                    m_grid->resolution(),
                    visual::MakeColorHexARGB(0xFFFF8C00),
                    m_grid->getReferenceFrame(),
                    vis_name));
}

int SparseEGraphDijkstra3DHeuristic::GetGoalHeuristic(const Eigen::Vector3i& dp)
{
    if (m_dist_grid.get(dp.x(), dp.y(), dp.z()).dist == Wall) {
        return Infinity;
    }

    auto* cell = &m_dist_grid(dp.x(), dp.y(), dp.z());

    static auto last_expand_count = 0;
    auto expand_count = 0;
    static auto repeat_count = 1;
    while (cell->dist == Unknown && !m_open.empty()) {
        ++expand_count;
        auto* curr_cell = m_open.min();
        m_open.pop();

        auto pit = m_open_cell_to_pos.find(curr_cell);
        assert(pit != end(m_open_cell_to_pos));

        auto pos = pit->second;
//        m_open_cell_to_pos.erase(pit);

        SMPL_DEBUG_NAMED(LOG, "Expand cell (%zu, %zu, %zu)", pos.x(), pos.y(), pos.z());

        // update experience graph neighbors
        auto it = m_heur_nodes.find(pos);
        if (it != end(m_heur_nodes)) {
            auto& hnode = it->second;
            SMPL_DEBUG_NAMED(LOG, "  %zu adjacent egraph cells", hnode.edges.size());
            for (auto& adj : hnode.edges) {
                auto* ncell = &m_dist_grid(adj.x(), adj.y(), adj.z());

                auto dp = adj - pos;
                auto cost = (int)(1000.0 * std::sqrt((double)dp.squaredNorm()));
                auto new_cost = curr_cell->dist + cost;
                if (new_cost < ncell->dist) {
                    ncell->dist = new_cost;
                    if (m_open.contains(ncell)) {
                        SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        m_open.decrease(ncell);
                    } else {
                        SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        m_open.push(ncell);
                        m_open_cell_to_pos[ncell] = adj;
                    }
                }
            }
        }

        // update 26-connected neighbors
        for (auto dx = -1; dx <= 1; ++dx) {
        for (auto dy = -1; dy <= 1; ++dy) {
        for (auto dz = -1; dz <= 1; ++dz) {
            if ((dx == 0) & (dy == 0) & (dz == 0)) {
                continue;
            }

            auto spos = Eigen::Vector3i(pos + Eigen::Vector3i(dx, dy, dz));

            auto* ncell = &m_dist_grid(spos.x(), spos.y(), spos.z());

            // bounds and obstacle check
            if (ncell->dist == Wall) {
                continue;
            }

            auto cost = (int)(m_eg_eps * 1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
            auto new_cost = curr_cell->dist + cost;

            if (new_cost < ncell->dist) {
                ncell->dist = new_cost;
                if (m_open.contains(ncell)) {
                    SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with normal edge (-> %d)", spos.x(), spos.y(), spos.z(), new_cost);
                    m_open.decrease(ncell);
                } else {
                    SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with normal edge (-> %d)", spos.x(), spos.y(), spos.z(), new_cost);
                    m_open.push(ncell);
                    m_open_cell_to_pos[ncell] = spos;
                }
            }
        }
        }
        }
    }

    if (last_expand_count != expand_count) {
        SMPL_INFO("Computed heuristic in %d expansions (after %d lookups)", expand_count, repeat_count);
        last_expand_count = expand_count;
        repeat_count = 1;
    } else {
        ++repeat_count;
    }

    if (cell->dist > Infinity) {
        return Infinity;
    }
    return cell->dist;
}

} // namespace smpl
