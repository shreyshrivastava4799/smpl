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

/// \author Andrew Dornbush

#include <smpl/heuristic/egraph_bfs_heuristic.h>

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
#include <smpl/stl/algorithm.h>

namespace smpl {

static const char* LOG = "heuristic.dijkstra_egraph";
static const char* SLOG = "heuristic.dijkstra_egraph.shortcuts";

static
auto DiscretizePoint(
    DijkstraEGraphHeuristic3D* heur,
    const Vector3& p)
    -> Eigen::Vector3i
{
    auto dp = Eigen::Vector3i();
    heur->m_grid->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    return dp;
}

static
auto RealizePoint(DijkstraEGraphHeuristic3D* heur, const Eigen::Vector3i& dp)
    -> Vector3
{
    auto p = Vector3();
    heur->m_grid->gridToWorld(dp.x(), dp.y(), dp.z(), p.x(), p.y(), p.z());
    return p;
}

// Project experience graph states down to their 3D projections. Note that this
// should be called once the goal is set in the environment as the projection to
// 3D may be based off of the goal condition (for instance, the desired planning
// frame is determined according to the planning link and a fixed offset)
static
void ProjectExperienceGraph(DijkstraEGraphHeuristic3D* heur)
{
    heur->m_heur_nodes.clear();

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
    auto* eg = heur->m_eg->GetExperienceGraph();
    if (eg == NULL) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    auto viz_points = std::vector<Vector3>();

    heur->m_projected_nodes.resize(eg->num_nodes());

    auto proj_node_count = 0;
    auto proj_edge_count = 0;
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        // project experience graph state to point and discretize
        auto first_id = heur->m_eg->GetStateID(*nit);
        SMPL_DEBUG_STREAM_NAMED(LOG, "Project experience graph state " << first_id << " " << eg->state(*nit) << " into 3D");
        auto p = heur->m_pp->ProjectToPoint(first_id);
        SMPL_DEBUG_NAMED(LOG, "Discretize point (%0.3f, %0.3f, %0.3f)", p.x(), p.y(), p.z());
        auto dp = DiscretizePoint(heur, p);

        auto viz_pt = RealizePoint(heur, dp);
        viz_points.push_back(viz_pt);

        dp += Eigen::Vector3i::Ones();

        heur->m_projected_nodes[*nit] = dp;

        // insert node into down-projected experience graph
        auto empty = DijkstraEGraphHeuristic3D::HeuristicNode();
        auto ent = heur->m_heur_nodes.insert(std::make_pair(dp, std::move(empty)));
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
            auto second_id = heur->m_eg->GetStateID(*ait);
            SMPL_DEBUG_NAMED(LOG, "  Project experience graph edge to state %d", second_id);
            auto q = heur->m_pp->ProjectToPoint(second_id);
            auto dq = DiscretizePoint(heur, q);
            if (!heur->m_grid->isInBounds(dq.x(), dq.y(), dq.z())) {
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

    SMPL_INFO("Projected experience graph contains %d nodes and %d edges", proj_node_count, proj_edge_count >> 1);

    auto comp_count = 0;
    heur->m_component_ids.assign(eg->num_nodes(), -1);
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (heur->m_component_ids[*nit] != -1) {
            continue;
        }

        auto frontier = std::vector<ExperienceGraph::node_id>();
        frontier.push_back(*nit);
        while (!frontier.empty()) {
            auto n = frontier.back();
            frontier.pop_back();

            heur->m_component_ids[n] = comp_count;

            auto adj = eg->adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                if (heur->m_component_ids[*ait] == -1) {
                    frontier.push_back(*ait);
                }
            }
        }

        ++comp_count;
    }

    // pre-allocate shortcuts array here, fill in updateGoal()
    heur->m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
    SMPL_INFO("Experience graph contains %d components", comp_count);

    auto* vis_name = "egraph_projection";
    SV_SHOW_INFO_NAMED(
            vis_name,
            visual::MakeCubesMarker(
                    std::move(viz_points),
                    heur->m_grid->resolution(),
                    visual::MakeColorHexARGB(0xFFFF8C00),
                    heur->m_grid->getReferenceFrame(),
                    vis_name));
}

static
int GetCellHeuristic(DijkstraEGraphHeuristic3D* heur, const Eigen::Vector3i& dp)
{
    constexpr auto Wall = DijkstraEGraphHeuristic3D::Wall;
    constexpr auto Unknown = DijkstraEGraphHeuristic3D::Unknown;
    constexpr auto Infinity = DijkstraEGraphHeuristic3D::Infinity;

    auto* cell = &heur->m_dist_grid(dp.x(), dp.y(), dp.z());

    if (cell->dist == Wall) {
        return Infinity;
    }

    static auto last_expand_count = 0;
    int expand_count = 0;
    static int repeat_count = 1;
    while (cell->dist == Unknown && !heur->m_open.empty()) {
        ++expand_count;
        auto* curr_cell = heur->m_open.min();
        heur->m_open.pop();

        auto cidx = (int)std::distance(heur->m_dist_grid.data(), curr_cell);
        int cx, cy, cz;
        heur->m_dist_grid.index_to_coord(cidx, cx, cy, cz);
        SMPL_DEBUG_NAMED(LOG, "Expand cell (%zu, %zu, %zu)", cx, cy, cz);

        // relax experience graph adjacency edges
        auto it = heur->m_heur_nodes.find(Eigen::Vector3i(cx, cy, cz));
        if (it != end(heur->m_heur_nodes)) {
            auto& hnode = it->second;
            SMPL_DEBUG_NAMED(LOG, "  %zu adjacent egraph cells", hnode.edges.size());
            for (auto& adj : hnode.edges) {
                auto dx = adj.x() - cx;
                auto dy = adj.y() - cy;
                auto dz = adj.z() - cz;
                auto* ncell = &heur->m_dist_grid(adj.x(), adj.y(), adj.z());

                auto cost = (int)(1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
                auto new_cost = curr_cell->dist + cost;
                if (new_cost < ncell->dist) {
                    ncell->dist = new_cost;
                    if (heur->m_open.contains(ncell)) {
                        SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        heur->m_open.decrease(ncell);
                    } else {
                        SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        heur->m_open.push(ncell);
                    }
                }
            }
        }

        // relax neighboring edges
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if ((dx == 0) & (dy == 0) & (dz == 0)) {
                continue;
            }

            auto sx = cx + dx;
            auto sy = cy + dy;
            auto sz = cz + dz;

            auto* ncell = &heur->m_dist_grid(sx, sy, sz);

            // bounds and obstacle check
            if (ncell->dist == Wall) {
                continue;
            }

            auto cost = (int)(heur->m_eg_eps * 1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
            auto new_cost = curr_cell->dist + cost;

            if (new_cost < ncell->dist) {
                ncell->dist = new_cost;
                if (heur->m_open.contains(ncell)) {
                    SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with normal edge (-> %d)", (int)sx, (int)sy, (int)sz, new_cost);
                    heur->m_open.decrease(ncell);
                } else {
                    SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with normal edge (-> %d)", (int)sx, (int)sy, (int)sz, new_cost);
                    heur->m_open.push(ncell);
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

auto DijkstraEGraphHeuristic3D::Vector3iHash::operator()(const argument_type& s) const
    -> result_type
{
    auto seed = (std::size_t)0;
    boost::hash_combine(seed, std::hash<int>()(s.x()));
    boost::hash_combine(seed, std::hash<int>()(s.y()));
    boost::hash_combine(seed, std::hash<int>()(s.z()));
    return seed;
}

bool DijkstraEGraphHeuristic3D::Init(
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

    m_dist_grid.assign(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

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

void DijkstraEGraphHeuristic3D::SyncGridAndDijkstra()
{
    auto xc = m_grid->numCellsX();
    auto yc = m_grid->numCellsY();
    auto zc = m_grid->numCellsZ();

    auto cell_count = xc * yc * zc;

    auto wall_count = 0;
    for (auto x = 0; x < m_grid->numCellsX(); ++x) {
    for (auto y = 0; y < m_grid->numCellsY(); ++y) {
    for (auto z = 0; z < m_grid->numCellsZ(); ++z) {
        auto radius = m_inflation_radius;
        if (m_grid->getDistance(x, y, z) <= radius) {
            m_dist_grid(x + 1, y + 1, z + 1).dist = Wall;
            ++wall_count;
        }
    }
    }
    }

    SMPL_INFO_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

auto DijkstraEGraphHeuristic3D::GetGrid() const -> const OccupancyGrid*
{
    return m_grid;
}

auto DijkstraEGraphHeuristic3D::GetEGraphWeight() const -> double
{
    return m_eg_eps;
}

void DijkstraEGraphHeuristic3D::SetEGraphWeight(double w)
{
    m_eg_eps = w;
}

auto DijkstraEGraphHeuristic3D::GetInflationRadius() const -> double
{
    return m_inflation_radius;
}

void DijkstraEGraphHeuristic3D::SetInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

auto DijkstraEGraphHeuristic3D::GetWallsVisualization() -> visual::Marker
{
    auto centers = std::vector<Vector3>();
    for (auto x = 0; x < m_grid->numCellsX(); x++) {
    for (auto y = 0; y < m_grid->numCellsY(); y++) {
    for (auto z = 0; z < m_grid->numCellsZ(); z++) {
        if (m_dist_grid(x + 1, y + 1, z + 1).dist == Wall) {
            auto p = Vector3();
            m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    auto cubes_marker = visual::MakeCubesMarker(
            std::move(centers),
            m_grid->resolution(),
            visual::MakeColorHexARGB(0xFF6495ED),
            m_grid->getReferenceFrame(),
            "bfs_walls",
            0);

    return cubes_marker;
}

auto DijkstraEGraphHeuristic3D::GetValuesVisualization() -> visual::Marker
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

        auto d = GetCellHeuristic(this, dp);
        auto cost_pct = (float)d / (float)max_cost;

        if (cost_pct > 1.0f) {
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

void DijkstraEGraphHeuristic3D::GetEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    auto p = m_pp->ProjectToPoint(state_id);
    auto dp = DiscretizePoint(this, p);

    if (!m_grid->isInBounds(dp.x(), dp.y(), dp.z())) {
        return;
    }
    dp += Eigen::Vector3i::Ones();

    auto hit = m_heur_nodes.find(dp);
    if (hit == m_heur_nodes.end()) {
        return;
    }

    for (auto eg_node : hit->second.up_nodes) {
        int id = m_eg->GetStateID(eg_node);
        if (id != state_id) {
            ids.push_back(id);
        }
    }
}

void DijkstraEGraphHeuristic3D::GetShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    SMPL_INFO_NAMED(SLOG, "Get shortcut successors for state %d", state_id);

    auto egraph_nodes = std::vector<ExperienceGraph::node_id>();
    m_eg->GetExperienceGraphNodes(state_id, egraph_nodes);

    SMPL_INFO_STREAM_NAMED(SLOG, "  e-graph nodes: " << egraph_nodes);

    for (auto node : egraph_nodes) {
        auto comp_id = m_component_ids[node];
        for (auto shortcut_node : m_shortcut_nodes[comp_id]) {
            auto id = m_eg->GetStateID(shortcut_node);
            if (id != state_id) {
                SMPL_INFO_NAMED(SLOG, "  %d", id);
                shortcut_ids.push_back(id);
            }
        }
    }
}

auto DijkstraEGraphHeuristic3D::GetMetricStartDistance(
    double x, double y, double z) -> double
{
    auto p = m_pp->ProjectToPoint(m_start_state_id);
    auto s = DiscretizePoint(this, p);
    auto g = DiscretizePoint(this, Vector3(x, y, z));
    auto d = Eigen::Vector3i(s - g);
    return m_grid->resolution() * (abs(d.x()) + abs(d.y()) + abs(d.z()));
}

auto DijkstraEGraphHeuristic3D::GetMetricGoalDistance(
    double x, double y, double z) -> double
{
    auto gp = m_get_goal_position->GetPosition();
    auto dgp = DiscretizePoint(this, gp);
    auto dp = DiscretizePoint(this, Vector3(x, y, z));
    auto d = Eigen::Vector3i(dgp - dp);
    return m_grid->resolution() * (abs(d.x()) + abs(d.y() + abs(d.z())));
}

auto DijkstraEGraphHeuristic3D::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IExperienceGraphHeuristic>() ||
        class_code == GetClassCode<IMetricStartHeuristic>() ||
        class_code == GetClassCode<IMetricGoalHeuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>())
    {
        return this;
    }
    return NULL;
}

bool DijkstraEGraphHeuristic3D::UpdateStart(int state_id)
{
    m_start_state_id = state_id;
    return true;
}

bool DijkstraEGraphHeuristic3D::UpdateGoal(GoalConstraint* goal)
{
    SMPL_INFO_NAMED(LOG, "Update EGraphBfsHeuristic goal");

    auto get_goal_position = goal->GetExtension<IGetPosition>();
    if (get_goal_position == NULL) {
        return false;
    }
    m_get_goal_position = get_goal_position;

    // reset all distances
    for (auto x = 1; x < m_dist_grid.xsize() - 1; ++x) {
    for (auto y = 1; y < m_dist_grid.ysize() - 1; ++y) {
    for (auto z = 1; z < m_dist_grid.zsize() - 1; ++z) {
        auto& c = m_dist_grid(x, y, z);
        if (c.dist != Wall) {
            c.dist = Unknown;
        }
    }
    }
    }

    ProjectExperienceGraph(this);

    auto gp = m_get_goal_position->GetPosition();

    auto dgp = DiscretizePoint(this, gp);

    // precompute shortcuts
    assert(m_component_ids.size() == m_eg->GetExperienceGraph()->num_nodes());
    auto* eg = m_eg->GetExperienceGraph();
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto comp_id = m_component_ids[*nit];
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

    SMPL_INFO_NAMED(LOG, "Updated EGraphBfsHeuristic goal");
    return true;
}

int DijkstraEGraphHeuristic3D::GetGoalHeuristic(int state_id)
{
    // project and discretize state
    auto p = m_pp->ProjectToPoint(state_id);
    Eigen::Vector3i dp;
    m_grid->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    if (!m_grid->isInBounds(dp.x(), dp.y(), dp.z())) {
        return Infinity;
    }

    dp += Eigen::Vector3i::Ones();
    return GetCellHeuristic(this, dp);
}

} // namespace smpl
