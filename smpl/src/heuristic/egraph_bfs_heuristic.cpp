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

namespace sbpl {
namespace motion {

auto DijkstraEgraphHeuristic3D::Vector3iHash::operator()(const argument_type& s) const
    -> result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(s.x()));
    boost::hash_combine(seed, std::hash<int>()(s.y()));
    boost::hash_combine(seed, std::hash<int>()(s.z()));
    return seed;
}

DijkstraEgraphHeuristic3D::DijkstraEgraphHeuristic3D(
    const RobotPlanningSpacePtr& ps,
    const OccupancyGrid* _grid)
:
    RobotHeuristic(ps, _grid),
    m_pp(nullptr)
{
    m_eg_eps = 5.0;

    m_pp = ps->getExtension<PointProjectionExtension>();
    m_eg = ps->getExtension<ExperienceGraphExtension>();

    if (!m_pp) {
        ROS_WARN_NAMED(params()->heuristic_log, "EgraphBfsHeuristic recommends PointProjectionExtension");
    }
    if (!m_eg) {
        ROS_WARN_NAMED(params()->heuristic_log, "EgraphBfsHeuristic recommends ExperienceGraphExtension");
    }

    size_t num_cells_x = grid()->numCellsX() + 2;
    size_t num_cells_y = grid()->numCellsY() + 2;
    size_t num_cells_z = grid()->numCellsZ() + 2;

    m_dist_grid.assign(num_cells_x, num_cells_y, num_cells_z, Cell(Unknown));

    ROS_INFO("Create dijkstra distance grid of size %zu x %zu x %zu", num_cells_x, num_cells_y, num_cells_z);

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
    ROS_INFO("Project experience graph into three-dimensional grid");
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        ROS_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        // project experience graph state to point and discretize
        int first_id = m_eg->getStateID(*nit);
        Eigen::Vector3d p;
        m_pp->projectToPoint(first_id, p);
        Eigen::Vector3i dp;
        grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
        if (!grid()->isInBounds(dp.x(), dp.y(), dp.z())) {
            continue;
        }
        dp += Eigen::Vector3i::Ones();

        // insert node into down-projected experience graph
        std::vector<Eigen::Vector3i> empty;
        auto ent = m_egraph_edges.insert(std::make_pair(dp, std::move(empty)));
        auto eit = ent.first;
        if (ent.second) {
            ROS_INFO("Inserted down-projected cell (%d, %d, %d) into experience graph heuristic", dp.x(), dp.y(), dp.z());
        }

        auto adj = eg->adjacent_nodes(*nit);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            // project adjacent experience graph state and discretize
            int second_id = m_eg->getStateID(*ait);
            Eigen::Vector3d q;
            m_pp->projectToPoint(second_id, q);
            Eigen::Vector3i dq;
            grid()->worldToGrid(q.x(), q.y(), q.z(), dq.x(), dq.y(), dq.z());
            if (!grid()->isInBounds(dq.x(), dq.y(), dq.z())) {
                continue;
            }
            dq += Eigen::Vector3i::Ones();

            // insert adjacent edge
            if (std::find(eit->second.begin(), eit->second.end(), dq) !=
                eit->second.end())
            {
                eit->second.push_back(dq);
            }
        }
    }
}

double DijkstraEgraphHeuristic3D::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
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
    return grid()->getResolution() * (abs(dx) + abs(dy) + abs(dz));;
}

double DijkstraEgraphHeuristic3D::getMetricGoalDistance(double x, double y, double z)
{
    Eigen::Vector3d gp(
            planningSpace()->goal().tgt_off_pose[0],
            planningSpace()->goal().tgt_off_pose[1],
            planningSpace()->goal().tgt_off_pose[2]);
    Eigen::Vector3i dgp;
    grid()->worldToGrid(gp.x(), gp.y(), gp.z(), dgp.x(), dgp.y(), dgp.z());

    Eigen::Vector3i dp;
    grid()->worldToGrid(x, y, z, dp.x(), dp.y(), dp.z());

    const Eigen::Vector3i d = dgp - dp;
    return grid()->getResolution() * (abs(d.x()) + abs(d.y() + abs(d.z())));
}

void DijkstraEgraphHeuristic3D::updateGoal(const GoalConstraint& goal)
{
    ROS_INFO_NAMED(params()->heuristic_log, "Update EGraphBfsHeuristic goal");

    // reset all distances
    for (size_t x = 1; x < m_dist_grid.xsize() - 1; ++x) {
    for (size_t y = 1; y < m_dist_grid.ysize() - 1; ++y) {
    for (size_t z = 1; z < m_dist_grid.zsize() - 1; ++z) {
        Cell& c = m_dist_grid(x, y, z);
        if (c.dist != Wall) {
            c.dist = Unknown;
        }
    } } }

    Eigen::Vector3i gp;
    grid()->worldToGrid(
            goal.tgt_off_pose[0],
            goal.tgt_off_pose[1],
            goal.tgt_off_pose[2],
            gp.x(), gp.y(), gp.z());

    if (!grid()->isInBounds(gp.x(), gp.y(), gp.z())) {
        ROS_WARN("Cell (%d, %d, %d) is outside heuristic bounds", gp.x(), gp.y(), gp.z());
        return;
    }

    gp += Eigen::Vector3i::Ones();

    m_open.clear();
    Cell* c = &m_dist_grid(gp.x(), gp.y(), gp.z());
    c->dist = 0;
    m_open.push(c);

    ROS_INFO_NAMED(params()->heuristic_log, "Updated EGraphBfsHeuristic goal");
}

int DijkstraEgraphHeuristic3D::GetGoalHeuristic(int state_id)
{
    // project and discretize state
    Eigen::Vector3d p;
    m_pp->projectToPoint(state_id, p);
    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    if (!grid()->isInBounds(dp.x(), dp.y(), dp.z())) {
        return Infinity;
    }

    dp += Eigen::Vector3i::Ones();
    Cell* cell = &m_dist_grid(dp.x(), dp.y(), dp.z());

    if (cell->dist == Wall) {
        return Infinity;
    }

    while (cell->dist == Unknown && !m_open.empty()) {
        Cell* curr_cell = m_open.min();
        m_open.pop();

        int cidx = std::distance(m_dist_grid.data(), curr_cell);
        size_t cx, cy, cz;
        m_dist_grid.index_to_coord(cidx, cx, cy, cz);
        ROS_DEBUG_NAMED(params()->heuristic_log, "Expand cell (%zu, %zu, %zu)", cx, cy, cz);

        // relax experience graph adjacency edges
        auto it = m_egraph_edges.find(Eigen::Vector3i(cx, cy, cz));
        if (it != m_egraph_edges.end()) {
            for (const Eigen::Vector3i& adj : it->second) {
                const int dx = adj.x() - cx;
                const int dy = adj.y() - cy;
                const int dz = adj.z() - cz;
                Cell* ncell = &m_dist_grid(adj.x(), adj.y(), adj.z());

                const int cost = (int)std::sqrt(1000 * (dx * dx + dy * dy + dz * dz));
                const int new_cost = curr_cell->dist + cost;
                if (new_cost < ncell->dist) {
                    ncell->dist = new_cost;
                    if (m_open.contains(ncell)) {
                        ROS_DEBUG_NAMED(params()->heuristic_log, "  Update cell (%d, %d, %d) with egraph edge", adj.x(), adj.y(), adj.z());
                        m_open.decrease(ncell);
                    } else {
                        ROS_DEBUG_NAMED(params()->heuristic_log, "  Insert cell (%d, %d, %d) with egraph edge", adj.x(), adj.y(), adj.z());
                        m_open.push(ncell);
                    }
                }
            }
        }

        // relax neighboring edges
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            const int sx = cx + dx;
            const int sy = cy + dy;
            const int sz = cz + dz;

            Cell* ncell = &m_dist_grid(sx, sy, sz);

            // bounds and obstacle check
            if (ncell->dist == Wall) {
                continue;
            }

            const int cost = (int)std::sqrt(
                    m_eg_eps *
                    1000 *
                    (dx * dx + dy * dy + dz * dz));
            const int new_cost = curr_cell->dist + cost;

            if (new_cost < ncell->dist) {
                ncell->dist = new_cost;
                if (m_open.contains(ncell)) {
                    ROS_DEBUG_NAMED(params()->heuristic_log, "  Update cell (%d, %d, %d) with normal edge", sx, sy, sz);
                    m_open.decrease(ncell);
                } else {
                    ROS_DEBUG_NAMED(params()->heuristic_log, "  Insert cell (%d, %d, %d) with normal edge", sx, sy, sz);
                    m_open.push(ncell);
                }
            }
        }
        }
        }
    }

    if (cell->dist > Infinity) {
        return Infinity;
    }
    return cell->dist;
}

int DijkstraEgraphHeuristic3D::GetStartHeuristic(int state_id)
{
    return 0;
}

int DijkstraEgraphHeuristic3D::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace motion
} // namespace sbpl
