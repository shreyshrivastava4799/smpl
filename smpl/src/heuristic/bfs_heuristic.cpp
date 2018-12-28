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

#include <smpl/heuristic/bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/colors.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/grid/grid.h>
#include <smpl/occupancy_grid.h>
#include <smpl/stl/algorithm.h>
#include <smpl/stl/memory.h>

namespace smpl {

static const char* LOG = "heuristic.bfs";

BFSHeuristic::~BFSHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BFSHeuristic::Init(DiscreteSpace* space, const OccupancyGrid* grid)
{
    if (grid == NULL) {
        return false;
    }

    auto project_to_point = space->GetExtension<IProjectToPoint>();
    if (project_to_point == NULL) {
        return false;
    }

    if (!Heuristic::Init(space)) {
        return false;
    }

    m_project_to_point = project_to_point;
    m_grid = grid;
    SyncGridAndBFS();
    return true;
}

double BFSHeuristic::GetInflationRadius() const
{
    return m_inflation_radius;
}

void BFSHeuristic::SetInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

int BFSHeuristic::GetCostPerCell() const
{
    return m_cost_per_cell;
}

void BFSHeuristic::SetCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

auto BFSHeuristic::GetGrid() const -> const OccupancyGrid*
{
    return m_grid;
}

auto BFSHeuristic::GetWallsVisualization() const -> visual::Marker
{
    auto centers = std::vector<Vector3>();
    auto dimX = GetGrid()->numCellsX();
    auto dimY = GetGrid()->numCellsY();
    auto dimZ = GetGrid()->numCellsZ();
    for (auto x = 0; x < dimX; x++) {
    for (auto y = 0; y < dimY; y++) {
    for (auto z = 0; z < dimZ; z++) {
        if (m_bfs->isWall(x, y, z)) {
            Vector3 p;
            GetGrid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    return visual::MakeCubesMarker(
            centers,
            GetGrid()->resolution(),
            visual::MakeColorHexARGB(0xFF6495ED),
            GetGrid()->getReferenceFrame(),
            "bfs_walls");
}

auto BFSHeuristic::GetValuesVisualization() -> visual::Marker
{
    auto all_invalid = true;
    for (auto& cell : m_goal_cells) {
        if (!m_bfs->isWall(cell.x, cell.y, cell.z)) {
            all_invalid = false;
            break;
        }
    }

    // no goal cells or all invalid => all invalid
    if (all_invalid) {
        return visual::MakeEmptyMarker();
    }

    if (m_start_state_id < 0) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(m_start_state_id);
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    auto max_cost = (int)(1.1 * (double)start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    // arbitrary limit on size of visualization...64Mb worth of points+colors
    auto max_points =
            (64u * 1024u * 1024u) /
            (sizeof(visual::Color) + sizeof(Vector3));

    auto points = std::vector<Vector3>();
    auto colors = std::vector<visual::Color>();

    struct CostCell
    {
        int x, y, z, g;
    };

    auto cells = std::queue<CostCell>();
    auto visited = Grid3<bool>(GetGrid()->numCellsX(), GetGrid()->numCellsY(), GetGrid()->numCellsZ(), false);
    for (auto& cell : m_goal_cells) {
        if (!m_bfs->isWall(cell.x, cell.y, cell.z)) {
            visited(cell.x, cell.y, cell.z) = true;
            cells.push({ cell.x, cell.y, cell.z, 0 });
        }
    }

    while (!cells.empty()) {
        auto c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            auto cost_pct = (float)c.g / (float)max_cost;

            auto color = visual::MakeColorHSV(300.0f - 300.0f * cost_pct);

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);
            color.a = 1.0f;

            Vector3 p;
            GetGrid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            points.push_back(p);

            colors.push_back(color);
        }

        auto d = m_cost_per_cell * m_bfs->getDistance(c.x, c.y, c.z);

        for (auto dx = -1; dx <= 1; ++dx) {
        for (auto dy = -1; dy <= 1; ++dy) {
        for (auto dz = -1; dz <= 1; ++dz) {
            if ((dx | dy | dz) == 0) {
                continue;
            }

            auto sx = c.x + dx;
            auto sy = c.y + dy;
            auto sz = c.z + dz;

            // check if neighbor is valid
            if (!m_bfs->inBounds(sx, sy, sz) || m_bfs->isWall(sx, sy, sz)) {
                continue;
            }

            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            auto dd = m_cost_per_cell * m_bfs->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }

    return visual::MakeCubesMarker(
            std::move(points),
            0.5 * GetGrid()->resolution(),
            std::move(colors),
            GetGrid()->getReferenceFrame(),
            "bfs_values");
}

void BFSHeuristic::SyncGridAndBFS()
{
    auto xc = GetGrid()->numCellsX();
    auto yc = GetGrid()->numCellsY();
    auto zc = GetGrid()->numCellsZ();
    m_bfs = make_unique<BFS_3D>(xc, yc, zc);
    auto wall_count = 0;
    for (auto x = 0; x < xc; ++x) {
    for (auto y = 0; y < yc; ++y) {
    for (auto z = 0; z < zc; ++z) {
        if (GetGrid()->getDistance(x, y, z) <= m_inflation_radius) {
            m_bfs->setWall(x, y, z);
            ++wall_count;
        }
    }
    }
    }

    auto cell_count = xc * yc * zc;
    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int BFSHeuristic::GetGoalHeuristic(int state_id)
{
    auto p = m_project_to_point->ProjectToPoint(state_id);

    Eigen::Vector3i dp;
    GetGrid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    return GetBFSCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
}

auto BFSHeuristic::GetMetricStartDistance(double x, double y, double z) -> double
{
    auto p = m_project_to_point->ProjectToPoint(m_start_state_id);

    int sx, sy, sz;
    GetGrid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    GetGrid()->worldToGrid(x, y, z, gx, gy, gz);

    auto dx = sx - gx;
    auto dy = sy - gy;
    auto dz = sz - gz;
    return GetGrid()->resolution() * (std::abs(dx) + std::abs(dy) + std::abs(dz));
}

auto BFSHeuristic::GetMetricGoalDistance(double x, double y, double z) -> double
{
    int gx, gy, gz;
    GetGrid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * GetGrid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * GetGrid()->resolution();
    }
}

bool BFSHeuristic::UpdateStart(int state_id)
{
    m_start_state_id = state_id;
    return true;
}

bool BFSHeuristic::UpdateGoal(GoalConstraint* goal)
{
    // poses array extension is preferred over post extension
    auto* get_pose_array = goal->GetExtension<IGetPoseArray>();
    if (get_pose_array != NULL) {
        auto poses = get_pose_array->GetPoseArray();
        auto cell_coords = std::vector<int>();
        for (auto i = 0; i < poses.second; ++i) {
            auto& goal_pose = poses.first[i];
            int gx, gy, gz;
            GetGrid()->worldToGrid(
                    goal_pose.translation()[0],
                    goal_pose.translation()[1],
                    goal_pose.translation()[2],
                    gx, gy, gz);

            SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

            if (!m_bfs->inBounds(gx, gy, gz)) {
                SMPL_WARN_NAMED(LOG, "Heuristic goal is out of BFS bounds");
                continue;
            }

            cell_coords.push_back(gx);
            cell_coords.push_back(gy);
            cell_coords.push_back(gz);

            m_goal_cells.emplace_back(gx, gy, gz);
        }

        m_bfs->run(begin(cell_coords), end(cell_coords));
        return true;
    }

    auto* get_pose = goal->GetExtension<IGetPose>();
    if (get_pose != NULL) {
        auto goal_pose = get_pose->GetPose();
        // TODO: This assumes goal.pose is initialized, regardless of what kind
        // of goal this is. For joint state goals, we should project the start
        // state to a goal position, since we can't reliably expect goal.pose
        // to be valid.
        int gx, gy, gz;
        GetGrid()->worldToGrid(
                goal_pose.translation()[0],
                goal_pose.translation()[1],
                goal_pose.translation()[2],
                gx, gy, gz);

        SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

        if (!m_bfs->inBounds(gx, gy, gz)) {
            SMPL_WARN_NAMED(LOG, "Heuristic goal is out of BFS bounds");
            return true; // graceful failure here
        }

        m_goal_cells.emplace_back(gx, gy, gz);

        m_bfs->run(gx, gy, gz);
        return true;
    }

    return false;
}

auto BFSHeuristic::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<Heuristic>() ||
        class_id == GetClassCode<IGoalHeuristic>() ||
        class_id == GetClassCode<IMetricGoalHeuristic>() ||
        class_id == GetClassCode<IMetricStartHeuristic>())
    {
        return this;
    }

    return NULL;
}

int BFSHeuristic::GetBFSCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    } else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    } else {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

} // namespace smpl
