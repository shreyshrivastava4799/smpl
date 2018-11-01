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

#include <boost/functional/hash.hpp>

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace smpl {

static const char* LOG = "heuristic.dijkstra_egraph";
static const char* SLOG = "heuristic.dijkstra_egraph.shortcuts";

auto DijkstraEgraphHeuristic3D::Vector3iHash::operator()(const argument_type& s) const
    -> result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(s.x()));
    boost::hash_combine(seed, std::hash<int>()(s.y()));
    boost::hash_combine(seed, std::hash<int>()(s.z()));
    return seed;
}

bool DijkstraEgraphHeuristic3D::init(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid)
{
    if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    m_eg = space->getExtension<ExperienceGraphExtension>();

    if (!m_pp) {
        SMPL_WARN_NAMED(LOG, "EgraphBfsHeuristic recommends PointProjectionExtension");
    }
    if (!m_eg) {
        SMPL_WARN_NAMED(LOG, "EgraphBfsHeuristic recommends ExperienceGraphExtension");
    }

    size_t num_cells_x = m_grid->numCellsX() + 2;
    size_t num_cells_y = m_grid->numCellsY() + 2;
    size_t num_cells_z = m_grid->numCellsZ() + 2;

    m_dist_grid.assign(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

    SMPL_INFO("Create dijkstra distance grid of size %zu x %zu x %zu", num_cells_x, num_cells_y, num_cells_z);

    syncGridAndDijkstra();

    auto add_wall = [&](int x, int y, int z) {
        m_dist_grid(x, y, z).dist = Wall;
    };

    // pad distance grid borders with walls
    for (int y = 0; y < num_cells_y; ++y) {
        for (int z = 0; z < num_cells_z; ++z) {
            add_wall(0, y, z);
            add_wall(num_cells_x - 1, y, z);
        }
    }
    for (int x = 1; x < num_cells_x - 1; ++x) {
        for (int z = 0; z < num_cells_z; ++z) {
            add_wall(x, 0, z);
            add_wall(x, num_cells_y - 1, z);
        }
    }
    for (int x = 1; x < num_cells_x - 1; ++x) {
        for (int y = 1; y < num_cells_y - 1; ++y) {
            add_wall(x, y, 0);
            add_wall(x, y, num_cells_z - 1);
        }
    }

    return true;
}

void DijkstraEgraphHeuristic3D::setWeightEGraph(double w)
{
    m_eg_eps = w;
    SMPL_INFO_NAMED(LOG, "egraph_epsilon: %0.3f", m_eg_eps);
}

void DijkstraEgraphHeuristic3D::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void DijkstraEgraphHeuristic3D::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    Vector3 p;
    m_pp->projectToPoint(state_id, p);
    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    if (!grid()->isInBounds(dp.x(), dp.y(), dp.z())) {
        return;
    }
    dp += Eigen::Vector3i::Ones();

    auto hit = m_heur_nodes.find(dp);
    if (hit == m_heur_nodes.end()) {
        return;
    }

    for (ExperienceGraph::node_id n : hit->second.up_nodes) {
        int id = m_eg->getStateID(n);
        if (id != state_id) {
            ids.push_back(id);
        }
    }
}

void DijkstraEgraphHeuristic3D::getShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    SMPL_INFO_NAMED(SLOG, "Get shortcut successors for state %d", state_id);

    std::vector<ExperienceGraph::node_id> egraph_nodes;
    m_eg->getExperienceGraphNodes(state_id, egraph_nodes);

    SMPL_INFO_STREAM_NAMED(SLOG, "  e-graph nodes: " << egraph_nodes);

    for (auto node : egraph_nodes) {
        auto comp_id = m_component_ids[node];
        for (auto shortcut_node : m_shortcut_nodes[comp_id]) {
            auto id = m_eg->getStateID(shortcut_node);
            if (id != state_id) {
                SMPL_INFO_NAMED(SLOG, "  %d", id);
                shortcut_ids.push_back(id);
            }
        }
    }
}

auto DijkstraEgraphHeuristic3D::getWallsVisualization() -> visual::Marker
{
    std::vector<Vector3> centers;
    for (int x = 0; x < grid()->numCellsX(); x++) {
    for (int y = 0; y < grid()->numCellsY(); y++) {
    for (int z = 0; z < grid()->numCellsZ(); z++) {
        if (m_dist_grid(x + 1, y + 1, z + 1).dist == Wall) {
            Vector3 p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    auto cubes_marker = visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls",
            0);

    return cubes_marker;
}

auto DijkstraEgraphHeuristic3D::getValuesVisualization() -> visual::Marker
{
    SMPL_INFO("Retrieve values visualization");

    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());

    int max_cost = (int)(1.1 * start_heur);

    std::vector<Vector3> points;
    std::vector<visual::Color> colors;
    for (int x = 0; x < grid()->numCellsX(); ++x) {
    for (int y = 0; y < grid()->numCellsY(); ++y) {
    for (int z = 0; z < grid()->numCellsZ(); ++z) {
        Eigen::Vector3i dp(x, y, z);
        dp += Eigen::Vector3i::Ones();

        int d = getGoalHeuristic(dp);
        double cost_pct = (double)d / (double)max_cost;

        if (cost_pct > 1.0) {
            continue;
        }

        visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

        auto clamp = [](double d, double lo, double hi) {
            if (d < lo) {
                return lo;
            } else if (d > hi) {
                return hi;
            } else {
                return d;
            }
        };

        color.r = clamp(color.r, 0.0f, 1.0f);
        color.g = clamp(color.g, 0.0f, 1.0f);
        color.b = clamp(color.b, 0.0f, 1.0f);

        Vector3 p;
        grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
        points.push_back(p);

        colors.push_back(color);
    }
    }
    }

    auto marker = MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "h_values");

    SMPL_INFO("Retrieved values visualization with %zu points", boost::get<visual::CubeList>(marker.shape).points.size());
    return marker;
}

double DijkstraEgraphHeuristic3D::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Vector3 p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double DijkstraEgraphHeuristic3D::getMetricGoalDistance(double x, double y, double z)
{
    Vector3 gp(planningSpace()->goal().pose.translation());
    Eigen::Vector3i dgp;
    grid()->worldToGrid(gp.x(), gp.y(), gp.z(), dgp.x(), dgp.y(), dgp.z());

    Eigen::Vector3i dp;
    grid()->worldToGrid(x, y, z, dp.x(), dp.y(), dp.z());

    const Eigen::Vector3i d = dgp - dp;
    return grid()->resolution() * (abs(d.x()) + abs(d.y() + abs(d.z())));
}

Extension* DijkstraEgraphHeuristic3D::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}

void DijkstraEgraphHeuristic3D::updateGoal(const GoalConstraint& goal)
{
    SMPL_INFO_NAMED(LOG, "Update EGraphBfsHeuristic goal");

    // reset all distances
    for (size_t x = 1; x < m_dist_grid.xsize() - 1; ++x) {
    for (size_t y = 1; y < m_dist_grid.ysize() - 1; ++y) {
    for (size_t z = 1; z < m_dist_grid.zsize() - 1; ++z) {
        auto& c = m_dist_grid(x, y, z);
        if (c.dist != Wall) {
            c.dist = Unknown;
        }
    } } }

    projectExperienceGraph();

    Vector3 gp(goal.pose.translation());

    Eigen::Vector3i dgp;
    grid()->worldToGrid(gp.x(), gp.y(), gp.z(), dgp.x(), dgp.y(), dgp.z());

    // precompute shortcuts
    assert(m_component_ids.size() == m_eg->getExperienceGraph()->num_nodes());
    auto* eg = m_eg->getExperienceGraph();
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto comp_id = m_component_ids[*nit];
        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(*nit);
            continue;
        }

        // get the distance of this node to the goal
        Vector3 p;
        m_pp->projectToPoint(m_eg->getStateID(*nit), p);

        auto dist = (gp - p).squaredNorm();

        Vector3 lp;
        m_pp->projectToPoint(m_eg->getStateID(m_shortcut_nodes[comp_id].front()), lp);

        auto curr_dist = (gp - lp).squaredNorm();

        if (dist < curr_dist) {
            m_shortcut_nodes[comp_id].clear();
            m_shortcut_nodes[comp_id].push_back(*nit);
        } else if (dist == curr_dist) {
            m_shortcut_nodes[comp_id].push_back(*nit);
        }
    }

    if (!grid()->isInBounds(dgp.x(), dgp.y(), dgp.z())) {
        SMPL_WARN("Cell (%d, %d, %d) is outside heuristic bounds", dgp.x(), dgp.y(), dgp.z());
        return;
    }

    dgp += Eigen::Vector3i::Ones();

    m_open.clear();
    auto* c = &m_dist_grid(dgp.x(), dgp.y(), dgp.z());
    c->dist = 0;
    m_open.push(c);

    SMPL_INFO_NAMED(LOG, "Updated EGraphBfsHeuristic goal");
}

int DijkstraEgraphHeuristic3D::GetGoalHeuristic(int state_id)
{
    // project and discretize state
    Vector3 p;
    m_pp->projectToPoint(state_id, p);
    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    if (!grid()->isInBounds(dp.x(), dp.y(), dp.z())) {
        return Infinity;
    }

    dp += Eigen::Vector3i::Ones();
    return getGoalHeuristic(dp);
}

int DijkstraEgraphHeuristic3D::GetStartHeuristic(int state_id)
{
    return 0;
}

int DijkstraEgraphHeuristic3D::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

// Project experience graph states down to their 3D projections. Note that this
// should be called once the goal is set in the environment as the projection to
// 3D may be based off of the goal condition (for instance, the desired planning
// frame is determined according to the planning link and a fixed offset)
void DijkstraEgraphHeuristic3D::projectExperienceGraph()
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
    auto* eg = m_eg->getExperienceGraph();
    if (!eg) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    std::vector<Vector3> viz_points;

    m_projected_nodes.resize(eg->num_nodes());

    int proj_node_count = 0;
    int proj_edge_count = 0;
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        // project experience graph state to point and discretize
        int first_id = m_eg->getStateID(*nit);
        SMPL_DEBUG_STREAM_NAMED(LOG, "Project experience graph state " << first_id << " " << eg->state(*nit) << " into 3D");
        Vector3 p;
        m_pp->projectToPoint(first_id, p);
        SMPL_DEBUG_NAMED(LOG, "Discretize point (%0.3f, %0.3f, %0.3f)", p.x(), p.y(), p.z());
        Eigen::Vector3i dp;
        grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

        Vector3 viz_pt;
        grid()->gridToWorld(dp.x(), dp.y(), dp.z(), viz_pt.x(), viz_pt.y(), viz_pt.z());
        viz_points.push_back(viz_pt);

        dp += Eigen::Vector3i::Ones();

        m_projected_nodes[*nit] = dp;

        // insert node into down-projected experience graph
        HeuristicNode empty;
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
            auto second_id = m_eg->getStateID(*ait);
            SMPL_DEBUG_NAMED(LOG, "  Project experience graph edge to state %d", second_id);
            Vector3 q;
            m_pp->projectToPoint(second_id, q);
            Eigen::Vector3i dq;
            grid()->worldToGrid(q.x(), q.y(), q.z(), dq.x(), dq.y(), dq.z());
            if (!grid()->isInBounds(dq.x(), dq.y(), dq.z())) {
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

    SMPL_INFO("Projected experience graph contains %d nodes and %d edges", proj_node_count, proj_edge_count);

    auto comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (m_component_ids[*nit] != -1) {
            continue;
        }

        std::vector<ExperienceGraph::node_id> frontier;
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

    visual::Color color;
    color.r = (float)0xFF / (float)0xFF;
    color.g = (float)0x8C / (float)0xFF;
    color.b = (float)0x00 / (float)0xFF;
    color.a = 1.0f;

    auto* vis_name = "egraph_projection";
    SV_SHOW_INFO_NAMED(
            vis_name,
            visual::MakeCubesMarker(
                    std::move(viz_points),
                    grid()->resolution(),
                    color,
                    grid()->getReferenceFrame(),
                    vis_name));
}

int DijkstraEgraphHeuristic3D::getGoalHeuristic(const Eigen::Vector3i& dp)
{
    Cell* cell = &m_dist_grid(dp.x(), dp.y(), dp.z());

    if (cell->dist == Wall) {
        return Infinity;
    }

    static int last_expand_count = 0;
    int expand_count = 0;
    static int repeat_count = 1;
    while (cell->dist == Unknown && !m_open.empty()) {
        ++expand_count;
        Cell* curr_cell = m_open.min();
        m_open.pop();

        int cidx = std::distance(m_dist_grid.data(), curr_cell);
        size_t cx, cy, cz;
        m_dist_grid.index_to_coord(cidx, cx, cy, cz);
        SMPL_DEBUG_NAMED(LOG, "Expand cell (%zu, %zu, %zu)", cx, cy, cz);

        // relax experience graph adjacency edges
        auto it = m_heur_nodes.find(Eigen::Vector3i(cx, cy, cz));
        if (it != end(m_heur_nodes)) {
            auto& hnode = it->second;
            SMPL_DEBUG_NAMED(LOG, "  %zu adjacent egraph cells", hnode.edges.size());
            for (auto& adj : hnode.edges) {
                auto dx = adj.x() - cx;
                auto dy = adj.y() - cy;
                auto dz = adj.z() - cz;
                auto* ncell = &m_dist_grid(adj.x(), adj.y(), adj.z());

                auto cost = (int)(1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
                auto new_cost = curr_cell->dist + cost;
                if (new_cost < ncell->dist) {
                    ncell->dist = new_cost;
                    if (m_open.contains(ncell)) {
                        SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        m_open.decrease(ncell);
                    } else {
                        SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with egraph edge (-> %d)", adj.x(), adj.y(), adj.z(), new_cost);
                        m_open.push(ncell);
                    }
                }
            }
        }

        // relax neighboring edges
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 & dy == 0 & dz == 0) {
                continue;
            }

            auto sx = cx + dx;
            auto sy = cy + dy;
            auto sz = cz + dz;

            auto* ncell = &m_dist_grid(sx, sy, sz);

            // bounds and obstacle check
            if (ncell->dist == Wall) {
                continue;
            }

            auto cost = (int)(m_eg_eps * 1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
            auto new_cost = curr_cell->dist + cost;

            if (new_cost < ncell->dist) {
                ncell->dist = new_cost;
                if (m_open.contains(ncell)) {
                    SMPL_DEBUG_NAMED(LOG, "  Update cell (%d, %d, %d) with normal edge (-> %d)", (int)sx, (int)sy, (int)sz, new_cost);
                    m_open.decrease(ncell);
                } else {
                    SMPL_DEBUG_NAMED(LOG, "  Insert cell (%d, %d, %d) with normal edge (-> %d)", (int)sx, (int)sy, (int)sz, new_cost);
                    m_open.push(ncell);
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

void DijkstraEgraphHeuristic3D::syncGridAndDijkstra()
{
    auto xc = grid()->numCellsX();
    auto yc = grid()->numCellsY();
    auto zc = grid()->numCellsZ();

    auto cell_count = xc * yc * zc;

    int wall_count = 0;
    for (int x = 0; x < grid()->numCellsX(); ++x) {
    for (int y = 0; y < grid()->numCellsY(); ++y) {
    for (int z = 0; z < grid()->numCellsZ(); ++z) {
        auto radius = m_inflation_radius;
        if (grid()->getDistance(x, y, z) <= radius) {
            m_dist_grid(x + 1, y + 1, z + 1).dist = Wall;
            ++wall_count;
        }
    }
    }
    }

    SMPL_INFO_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

} // namespace smpl
