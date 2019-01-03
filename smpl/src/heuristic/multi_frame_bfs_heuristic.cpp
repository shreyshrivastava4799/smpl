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

#include <smpl/heuristic/multi_frame_bfs_heuristic.h>

// system includes
#include <Eigen/Core>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/colors.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/stl/algorithm.h>

namespace smpl {

static const char* LOG = "heuristic.mfbfs";

static
int combine_costs(int c1, int c2)
{
    return c1 + c2;
}

static
auto DiscretizePoint(const OccupancyGrid* grid, const Vector3& p)
    -> Eigen::Vector3i
{
    auto dp = Eigen::Vector3i();
    grid->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    return dp;
}

static
int GetBFSCostToGoal(
    const BFS_3D& bfs, int x, int y, int z, int cost_per_cell)
{
    if (!bfs.inBounds(x, y, z)) {
        return MultiFrameBFSHeuristic::Infinity;
    } else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return MultiFrameBFSHeuristic::Infinity;
    } else {
        return cost_per_cell * bfs.getDistance(x, y, z);
    }
}

static
int GetGoalHeuristicMF(
    const MultiFrameBFSHeuristic* heur, int state_id, bool use_ee)
{
    auto p = heur->m_pp->ProjectToPoint(state_id);
    auto dp = DiscretizePoint(heur->m_grid, p);
    auto h_planning_frame = GetBFSCostToGoal(*heur->m_bfs, dp.x(), dp.y(), dp.z(), heur->m_cost_per_cell);

    int h_planning_link = 0;
    if (use_ee) {
        auto& state = heur->m_ers->ExtractState(state_id);
        auto pose = heur->m_fk_iface->computeFK(state);
        auto eex = DiscretizePoint(heur->m_grid, pose.translation());
        h_planning_link = GetBFSCostToGoal(*heur->m_ee_bfs, eex[0], eex[1], eex[2], heur->m_cost_per_cell);
    }

    return combine_costs(h_planning_frame, h_planning_link);
}

MultiFrameBFSHeuristic::~MultiFrameBFSHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiFrameBFSHeuristic::Init(
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

    auto* extract_state = space->GetExtension<IExtractRobotState>();
    if (extract_state == NULL) {
        return false;
    }

    auto* fk = space->GetRobotModel()->GetExtension<IForwardKinematics>();
    if (fk == NULL) {
        return false;
    }

    if (!Heuristic::Init(space)) {
        return false;
    }

    m_pos_offset[0] = m_pos_offset[1] = m_pos_offset[2] = 0.0;
    m_grid = grid;
    m_pp = project_to_point;
    m_ers = extract_state;
    m_fk_iface = fk;

    SyncGridAndBFS();

    return true;
}

void MultiFrameBFSHeuristic::SyncGridAndBFS()
{
    auto xc = m_grid->numCellsX();
    auto yc = m_grid->numCellsY();
    auto zc = m_grid->numCellsZ();
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    m_ee_bfs.reset(new BFS_3D(xc, yc, zc));
    auto cell_count = xc * yc * zc;
    auto wall_count = 0;
    for (auto z = 0; z < zc; ++z) {
    for (auto y = 0; y < yc; ++y) {
    for (auto x = 0; x < xc; ++x) {
        auto radius = m_inflation_radius;
        if (m_grid->getDistance(x, y, z) <= radius) {
            m_bfs->setWall(x, y, z);
            m_ee_bfs->setWall(x, y, z);
            ++wall_count;
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

void MultiFrameBFSHeuristic::SetOffset(double x, double y, double z)
{
    m_pos_offset[0] = x;
    m_pos_offset[1] = y;
    m_pos_offset[2] = z;
}

auto MultiFrameBFSHeuristic::GetInflationRadius() const -> double
{
    return m_inflation_radius;
}

void MultiFrameBFSHeuristic::SetInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

int MultiFrameBFSHeuristic::GetCostPerCell() const
{
    return m_cost_per_cell;
}

void MultiFrameBFSHeuristic::SetCostPerCell(int cost)
{
    m_cost_per_cell = cost;
}

auto MultiFrameBFSHeuristic::GetGrid() const -> const OccupancyGrid*
{
    return m_grid;
}

auto MultiFrameBFSHeuristic::GetWallsVisualization() const -> visual::Marker
{
    auto points = std::vector<Vector3>();
    auto dimX = m_grid->numCellsX();
    auto dimY = m_grid->numCellsY();
    auto dimZ = m_grid->numCellsZ();
    for (auto z = 0; z < dimZ; z++) {
    for (auto y = 0; y < dimY; y++) {
    for (auto x = 0; x < dimX; x++) {
        if (m_bfs->isWall(x, y, z)) {
            Vector3 p;
            m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            points.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", points.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    return visual::MakeCubesMarker(
            points,
            m_grid->resolution(),
            color,
            m_grid->getReferenceFrame(),
            "bfs_walls");
}

auto MultiFrameBFSHeuristic::GetValuesVisualization() const -> visual::Marker
{
    if (m_start_state_id < 0) {
        return visual::MakeEmptyMarker();
    }

    // factor in the ee bfs values? This doesn't seem to make a whole lot of
    // sense since the color would be derived from colocated cell values
    auto factor_ee = false;

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    auto start_heur = GetGoalHeuristicMF(this, m_start_state_id, factor_ee);

    auto edge_cost = m_cost_per_cell;

    auto max_cost = (int)(1.1 * start_heur);

    // ...and this will also flush the bfs...

    auto points = std::vector<Vector3>();
    auto colors = std::vector<visual::Color>();
    for (auto z = 0; z < m_grid->numCellsZ(); ++z) {
    for (auto y = 0; y < m_grid->numCellsY(); ++y) {
    for (auto x = 0; x < m_grid->numCellsX(); ++x) {
        // skip cells without valid distances from the start
        if (m_bfs->isWall(x, y, z) || m_bfs->isUndiscovered(x, y, z)) {
            continue;
        }

        auto d = edge_cost * m_bfs->getDistance(x, y, z);
        auto eed = factor_ee ? edge_cost * m_ee_bfs->getDistance(x, y, z) : 0;
        auto cost_pct = (float)combine_costs(d, eed) / (float)(max_cost);

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

    return visual::MakeCubesMarker(
            std::move(points),
            0.5 * m_grid->resolution(),
            std::move(colors),
            m_grid->getReferenceFrame(),
            "bfs_values");
}

int MultiFrameBFSHeuristic::GetGoalHeuristic(int state_id)
{
    return GetGoalHeuristicMF(this, state_id, true);
}

auto MultiFrameBFSHeuristic::GetMetricStartDistance(
    double x, double y, double z) -> double
{
    auto p = m_pp->ProjectToPoint(m_start_state_id);
    auto sp = DiscretizePoint(m_grid, p);
    auto disc_p = DiscretizePoint(m_grid, Vector3(x, y, z));
    auto dp = sp - disc_p;
    return m_grid->resolution() * (abs(dp.x()) + abs(dp.y()) + abs(dp.z()));
}

auto MultiFrameBFSHeuristic::GetMetricGoalDistance(
    double x, double y, double z) -> double
{
    auto p = DiscretizePoint(m_grid, Vector3(x, y, z));
    if (!m_bfs->inBounds(p.x(), p.y(), p.z())) {
        return (double)BFS_3D::WALL * m_grid->resolution();
    }
    return (double)m_bfs->getDistance(p.x(), p.y(), p.z()) * m_grid->resolution();
}

bool MultiFrameBFSHeuristic::UpdateStart(int state_id)
{
    m_start_state_id = state_id;
    return true;
}

bool MultiFrameBFSHeuristic::UpdateGoal(GoalConstraint* goal)
{
    SMPL_DEBUG_NAMED(LOG, "Update goal");

    auto* get_goal_position = goal->GetExtension<IGetPose>();
    if (get_goal_position == NULL) {
        return false;
    }

    auto goal_pose = get_goal_position->GetPose();

    auto offset_pose = Affine3(
            goal_pose *
            Translation3(m_pos_offset[0], m_pos_offset[1], m_pos_offset[2]));

    auto ogp = DiscretizePoint(m_grid, offset_pose.translation());
    auto gp = DiscretizePoint(m_grid, goal_pose.translation());

    SMPL_DEBUG_NAMED(LOG, "Setting the Two-Point BFS heuristic goals (%d, %d, %d), (%d, %d, %d)",
            ogp.x(), ogp.y(), ogp.z(), gp.x(), gp.y(), gp.z());

    if (m_bfs->inBounds(ogp.x(), ogp.y(), ogp.z()) &&
        m_ee_bfs->inBounds(gp.x(), gp.y(), gp.z()))
    {
        m_bfs->run(ogp.x(), ogp.y(), ogp.z());
        m_ee_bfs->run(gp.x(), gp.y(), gp.z());
    } else {
        SMPL_WARN_NAMED(LOG, "Heuristic goal is out of BFS bounds");
    }

    return true;
}

auto MultiFrameBFSHeuristic::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<Heuristic>() ||
        class_code == GetClassCode<IGoalHeuristic>() ||
        class_code == GetClassCode<IMetricGoalHeuristic>() ||
        class_code == GetClassCode<IMetricStartHeuristic>())
    {
        return this;
    }
    return NULL;
}

} // namespace smpl
