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

#include <boost/functional/hash.hpp>

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>
#include <smpl/occupancy_grid.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/spatial.h>

namespace smpl {

static const char* LOG = "heuristic.egraph_bfs";

auto SparseEGraphDijkstra3DHeuristic::Vector3iHash::operator()(
    const argument_type& s) const
    -> result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(s.x()));
    boost::hash_combine(seed, std::hash<int>()(s.y()));
    boost::hash_combine(seed, std::hash<int>()(s.z()));
    return seed;
}

bool SparseEGraphDijkstra3DHeuristic::init(
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

    m_dist_grid.resize(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

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

void SparseEGraphDijkstra3DHeuristic::setWeightEGraph(double w)
{
    m_eg_eps = w;
    SMPL_INFO_NAMED(LOG, "egraph_epsilon: %0.3f", m_eg_eps);
}

void SparseEGraphDijkstra3DHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void SparseEGraphDijkstra3DHeuristic::getEquivalentStates(
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

void SparseEGraphDijkstra3DHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    std::vector<ExperienceGraph::node_id> egraph_nodes;
    m_eg->getExperienceGraphNodes(state_id, egraph_nodes);

    for (ExperienceGraph::node_id n : egraph_nodes) {
        const int comp_id = m_component_ids[n];
        for (ExperienceGraph::node_id nn : m_shortcut_nodes[comp_id]) {
            int id = m_eg->getStateID(nn);
            if (id != state_id) {
                shortcut_ids.push_back(id);
            }
        }
    }
}

auto SparseEGraphDijkstra3DHeuristic::getWallsVisualization() -> visual::Marker
{
    std::vector<Vector3> centers;
    for (int x = 0; x < grid()->numCellsX(); x++) {
    for (int y = 0; y < grid()->numCellsY(); y++) {
    for (int z = 0; z < grid()->numCellsZ(); z++) {
        if (m_dist_grid.get(x + 1, y + 1, z + 1).dist == Wall) {
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

auto SparseEGraphDijkstra3DHeuristic::getValuesVisualization() -> visual::Marker
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

double SparseEGraphDijkstra3DHeuristic::getMetricStartDistance(
    double x, double y, double z)
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

double SparseEGraphDijkstra3DHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    Vector3 gp(planningSpace()->goal().pose.translation());
    Eigen::Vector3i dgp;
    grid()->worldToGrid(gp.x(), gp.y(), gp.z(), dgp.x(), dgp.y(), dgp.z());

    Eigen::Vector3i dp;
    grid()->worldToGrid(x, y, z, dp.x(), dp.y(), dp.z());

    const Eigen::Vector3i d = dgp - dp;
    return grid()->resolution() * (abs(d.x()) + abs(d.y() + abs(d.z())));
}

auto SparseEGraphDijkstra3DHeuristic::getExtension(size_t class_code)
    -> Extension*
{
    if (class_code == GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}

void SparseEGraphDijkstra3DHeuristic::updateGoal(const GoalConstraint& goal)
{
    SMPL_INFO_NAMED(LOG, "Update EGraphBfsHeuristic goal");

    m_open.clear();

    // reset all distances
    for (size_t x = 1; x < m_dist_grid.size_x() - 1; ++x) {
    for (size_t y = 1; y < m_dist_grid.size_y() - 1; ++y) {
    for (size_t z = 1; z < m_dist_grid.size_z() - 1; ++z) {
        if (m_dist_grid.get(x, y, z).dist != Wall) {
            m_dist_grid.set(x, y, z, Cell(Unknown));
        }
    }
    }
    }

    projectExperienceGraph();

    Vector3 gp(goal.pose.translation());

    Eigen::Vector3i dgp;
    grid()->worldToGrid(gp.x(), gp.y(), gp.z(), dgp.x(), dgp.y(), dgp.z());

    // precompute shortcuts
    assert(m_component_ids.size() == m_eg->getExperienceGraph()->num_nodes());
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        int comp_id = m_component_ids[*nit];
        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(*nit);
            continue;
        }

        // get the distance of this node to the goal
        Vector3 p;
        m_pp->projectToPoint(m_eg->getStateID(*nit), p);

        const double dist = (gp - p).squaredNorm();

        Vector3 lp;
        m_pp->projectToPoint(m_eg->getStateID(m_shortcut_nodes[comp_id].front()), lp);

        const double curr_dist = (gp - lp).squaredNorm();

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
    Cell* c = &m_dist_grid(dgp.x(), dgp.y(), dgp.z());
    c->dist = 0;
    m_open.push(c);
    m_open_cell_to_pos[c] = dgp;

    SMPL_INFO_NAMED(LOG, "Updated EGraphBfsHeuristic goal");
}

int SparseEGraphDijkstra3DHeuristic::GetGoalHeuristic(int state_id)
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

int SparseEGraphDijkstra3DHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int SparseEGraphDijkstra3DHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

// Project experience graph states down to their 3D projections. Note that this
// should be called once the goal is set in the environment as the projection to
// 3D may be based off of the goal condition (for instance, the desired planning
// frame is determined according to the planning link and a fixed offset)
void SparseEGraphDijkstra3DHeuristic::projectExperienceGraph()
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
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    std::vector<Vector3> viz_points;

    m_projected_nodes.resize(eg->num_nodes());

    size_t proj_node_count = 0;
    size_t proj_edge_count = 0;
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

        HeuristicNode& hnode = eit->second;

        hnode.up_nodes.push_back(*nit);

        auto adj = eg->adjacent_nodes(*nit);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            // project adjacent experience graph state and discretize
            int second_id = m_eg->getStateID(*ait);
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

    SMPL_INFO("Projected experience graph contains %zu nodes and %zu edges", proj_node_count, proj_edge_count);

    int comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (m_component_ids[*nit] != -1) {
            continue;
        }

        std::vector<ExperienceGraph::node_id> frontier;
        frontier.push_back(*nit);
        while (!frontier.empty()) {
            ExperienceGraph::node_id n = frontier.back();
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

int SparseEGraphDijkstra3DHeuristic::getGoalHeuristic(const Eigen::Vector3i& dp)
{
    if (m_dist_grid.get(dp.x(), dp.y(), dp.z()).dist == Wall) {
        return Infinity;
    }

    Cell* cell = &m_dist_grid(dp.x(), dp.y(), dp.z());

    static int last_expand_count = 0;
    int expand_count = 0;
    static int repeat_count = 1;
    while (cell->dist == Unknown && !m_open.empty()) {
        ++expand_count;
        Cell* curr_cell = m_open.min();
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
                Cell* ncell = &m_dist_grid(adj.x(), adj.y(), adj.z());

                auto dp = adj - pos;
                int cost = (int)(1000.0 * std::sqrt((double)dp.squaredNorm()));
                int new_cost = curr_cell->dist + cost;
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
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 & dy == 0 & dz == 0) {
                continue;
            }

            Eigen::Vector3i spos = pos + Eigen::Vector3i(dx, dy, dz);

            Cell* ncell = &m_dist_grid(spos.x(), spos.y(), spos.z());

            // bounds and obstacle check
            if (ncell->dist == Wall) {
                continue;
            }

            int cost = (int)(m_eg_eps * 1000.0 * std::sqrt((double)(dx * dx + dy * dy + dz * dz)));
            int new_cost = curr_cell->dist + cost;

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

void SparseEGraphDijkstra3DHeuristic::syncGridAndDijkstra()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();

    const int cell_count = xc * yc * zc;

    int wall_count = 0;
    for (int x = 0; x < grid()->numCellsX(); ++x) {
    for (int y = 0; y < grid()->numCellsY(); ++y) {
    for (int z = 0; z < grid()->numCellsZ(); ++z) {
        if (grid()->getDistance(x, y, z) <= m_inflation_radius) {
            m_dist_grid(x + 1, y + 1, z + 1).dist = Wall;
            ++wall_count;
        }
    }
    }
    }

    SMPL_INFO_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

} // namespace smpl
