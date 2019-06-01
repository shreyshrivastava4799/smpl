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

#include <smpl/graph/adaptive_workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/collision_checker.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>

constexpr auto PlanAdaptiveGridVisName = "adaptive_grid_plan";
constexpr auto TrackAdaptiveGridVisName = "adaptive_grid_track";

namespace smpl {

bool operator==(const AdaptiveGridState& a, const AdaptiveGridState& b)
{
    return std::tie(a.gx, a.gy, a.gz) == std::tie(b.gx, b.gy, b.gz);
}

auto operator<<(std::ostream& o, const AdaptiveGridState& s) -> std::ostream&
{
    o << "{ x: " << s.gx << ", y: " << s.gy << ", z: " << s.gy << " }";
    return o;
}

bool operator==(const AdaptiveWorkspaceState& a, const AdaptiveWorkspaceState& b)
{
    return a.coord == b.coord;
}

auto operator<<(std::ostream& o, const AdaptiveWorkspaceState& s) -> std::ostream&
{
    o << "{ coord: " << s.coord << ", state: " << s.state << " }";
    return o;
}

} // namespace smpl

auto std::hash<smpl::AdaptiveGridState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<decltype(s.gx)>()(s.gx));
    boost::hash_combine(seed, std::hash<decltype(s.gy)>()(s.gy));
    boost::hash_combine(seed, std::hash<decltype(s.gz)>()(s.gz));
    return seed;
}

auto std::hash<smpl::AdaptiveWorkspaceState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace smpl {

static
void InitMotionPrimitives(AdaptiveWorkspaceLattice* lattice)
{
    // TODO: Factor out copy-pasta from WorkspaceLattice?
    lattice->m_hi_prims.clear();

    auto prim = MotionPrimitive();

    // create 26-connected position motions
    for (auto dx = -1; dx <= 1; ++dx) {
    for (auto dy = -1; dy <= 1; ++dy) {
    for (auto dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }

        auto d = std::vector<double>(GetNumDOFs(&lattice->m_proj), 0.0);
        d[0] = lattice->m_proj.res[0] * dx;
        d[1] = lattice->m_proj.res[1] * dy;
        d[2] = lattice->m_proj.res[2] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        lattice->m_hi_prims.push_back(prim);
    }
    }
    }

    // create 2-connected motions for rotation and free angle motions
    for (auto a = 3; a < lattice->m_proj.dof_count; ++a) {
        auto d = std::vector<double>(lattice->m_proj.dof_count, 0.0);

        d[a] = lattice->m_proj.res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        lattice->m_hi_prims.push_back(prim);

        d[a] = lattice->m_proj.res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        lattice->m_hi_prims.push_back(prim);
    }

    // create 26-connected ld position motions
    lattice->m_lo_prims.clear();
    for (auto dx = -1; dx <= 1; ++dx) {
    for (auto dy = -1; dy <= 1; ++dy) {
    for (auto dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }

         lattice->m_lo_prims.emplace_back(
                lattice->m_proj.res[0] * dx,
                lattice->m_proj.res[1] * dy,
                lattice->m_proj.res[2] * dz);
    }
    }
    }
}

static
int ReserveHashEntry(AdaptiveWorkspaceLattice* lattice, bool hid)
{
    AdaptiveState* entry;
    if (hid) {
        entry = new AdaptiveWorkspaceState;
    } else {
        entry = new AdaptiveGridState;
    }
    entry->hid = hid;

    int state_id = (int)lattice->m_states.size();

    // map state id -> state
    lattice->m_states.push_back(entry);

    return state_id;
}

static
auto GetHashEntry(
    const AdaptiveWorkspaceLattice* lattice,
    int state_id)
    -> AdaptiveState*
{
    assert(state_id >= 0 && state_id < lattice->m_states.size());
    return lattice->m_states[state_id];
}

static
auto GetHashEntryHi(
    const AdaptiveWorkspaceLattice* lattice,
    int state_id)
    -> AdaptiveWorkspaceState*
{
    auto* state = GetHashEntry(lattice, state_id);
    if (state->hid) {
        return (AdaptiveWorkspaceState*)state;
    }
    return NULL;
}

static
auto GetHashEntryLo(
    const AdaptiveWorkspaceLattice* lattice,
    int state_id)
    -> AdaptiveGridState*
{
    auto* state = GetHashEntry(lattice, state_id);
    if (!state->hid) {
        return (AdaptiveGridState*)state;
    }
    return NULL;
}

static
int GetHashEntryHi(
    AdaptiveWorkspaceLattice* lattice,
    const WorkspaceCoord& coord)
{
    auto state = AdaptiveWorkspaceState();
    state.coord = coord;
    auto sit = lattice->m_hi_to_id.find(&state);
    if (sit == lattice->m_hi_to_id.end()) {
        return -1;
    }
    return sit->second;
}

static
int GetHashEntryLo(
    AdaptiveWorkspaceLattice* lattice,
    int x, int y, int z)
{
    auto state = AdaptiveGridState();
    state.gx = x;
    state.gy = y;
    state.gz = z;
    auto sit = lattice->m_lo_to_id.find(&state);
    if (sit == lattice->m_lo_to_id.end()) {
        return -1;
    }
    return sit->second;
}

static
int CreateStateHi(
    AdaptiveWorkspaceLattice* lattice,
    const WorkspaceCoord& coord,
    const RobotState& state)
{
    auto state_id = ReserveHashEntry(lattice, true);
    auto* hi_state = GetHashEntryHi(lattice, state_id);
    hi_state->coord = coord;
    hi_state->state = state;
    lattice->m_hi_to_id[hi_state] = state_id;
    return state_id;
}

static
int CreateStateLo(
    AdaptiveWorkspaceLattice* lattice,
    int x, int y, int z,
    double wx, double wy, double wz)
{
    auto state_id = ReserveHashEntry(lattice, false);
    auto* lo_state = GetHashEntryLo(lattice, state_id);
    lo_state->gx = x;
    lo_state->gy = y;
    lo_state->gz = z;
    lo_state->x = wx;
    lo_state->y = wy;
    lo_state->z = wz;
    return state_id;
}

static
bool IsHighDimensional(
    const AdaptiveWorkspaceLattice* lattice,
    int gx, int gy, int gz)
{
    if (lattice->m_plan_mode) {
        return lattice->m_dim_grid(gx, gy, gz).plan_hd;
    } else {
        return lattice->m_dim_grid(gx, gy, gz).trak_hd;
    }
}

static
auto GetStateVisualization(
    AdaptiveWorkspaceLattice* lattice,
    const RobotState& state,
    const std::string& ns)
    -> std::vector<visual::Marker>
{
    auto markers = lattice->GetCollisionChecker()->GetCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

static
auto GetAdaptiveGridVisualization(
    AdaptiveWorkspaceLattice* lattice,
    bool plan_mode)
    -> visual::Marker
{
    auto points = std::vector<Vector3>();
    for (auto x = 0; x < lattice->m_grid->numCellsX(); ++x) {
    for (auto y = 0; y < lattice->m_grid->numCellsY(); ++y) {
    for (auto z = 0; z < lattice->m_grid->numCellsZ(); ++z) {
        if (IsHighDimensional(lattice, x, y, z)) {
            auto p = Vector3();
            lattice->m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            points.push_back(p);
        }
    }
    }
    }

    auto color = visual::Color{ };
    if (lattice->m_plan_mode) {
        color = visual::Color{ 1.0f, 0.0f, 0.0f, 1.0f };
    } else {
        color = visual::Color{ 0.0f, 0.0f, 1.0f, 1.0f };
    }

    SMPL_INFO("Visualize %zu/%zu adaptive cells", points.size(), lattice->m_dim_grid.size());

    return MakeCubesMarker(
        std::move(points),
        0.5 * lattice->m_grid->resolution(),
        color,
        lattice->m_grid->getReferenceFrame(),
        plan_mode ? PlanAdaptiveGridVisName : TrackAdaptiveGridVisName);
}

static
void GetActions(
    AdaptiveWorkspaceLattice* lattice,
    const AdaptiveWorkspaceState& state,
    std::vector<Action>& actions)
{
    actions.clear();
    actions.reserve(lattice->m_hi_prims.size());

    auto cont_state = WorkspaceState();
    StateCoordToWorkspace(&lattice->m_proj, state.coord, cont_state);

    SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "Create actions for state: " << cont_state);

    for (auto pidx = 0; pidx < lattice->m_hi_prims.size(); ++pidx) {
        auto& prim = lattice->m_hi_prims[pidx];

        auto action = Action();
        action.reserve(prim.action.size());

        auto final_state = cont_state;
        for (auto& delta_state : prim.action) {
            for (int d = 0; d < lattice->m_proj.dof_count; ++d) {
                final_state[d] += delta_state[d];
            }

            angles::normalize_euler_zyx(&final_state[3]);

            action.push_back(final_state);
        }

        actions.push_back(std::move(action));
    }

#if 0
    if (lattice->m_ik_amp_enabled && numHeuristics() > 0) {
        auto* h = heuristic(0);
        auto goal_dist = h->getMetricGoalDistance(
                cont_state[0], cont_state[1], cont_state[2]);
        if (goal_dist < lattice->m_ik_amp_thresh) {
            auto ik_sol = std::vector<double>();
            if (lattice->m_ik_iface->ComputeIK(goal().pose, state.state, ik_sol)) {
                auto final_state = WorkspaceState();
                StateRobotToWorkspace(&lattice->m_proj, ik_sol, final_state);
                auto action = Action(1);
                action[0] = final_state;
                actions.push_back(std::move(action));
            }
        }
    }
#endif
}

static
bool CheckAction(
    AdaptiveWorkspaceLattice* lattice,
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    auto wptraj = std::vector<RobotState>();
    wptraj.reserve(action.size());

    // check waypoints for ik solutions and joint limits
    for (auto widx = 0; widx < action.size(); ++widx) {
        auto& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "        " << widx << ": " << istate);

        auto irstate = RobotState();
        if (!StateWorkspaceToRobot(&lattice->m_proj, istate, state, irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "         -> failed to find ik solution");
            return false;
        }

        wptraj.push_back(irstate);

        if (!lattice->GetRobotModel()->CheckJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> violates joint limits");
            return false;
        }
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!lattice->GetCollisionChecker()->IsStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path to first waypoint in collision");
        return false;
    }

    for (auto widx = 1; widx < wptraj.size(); ++widx) {
        auto& prev_istate = wptraj[widx - 1];
        auto& curr_istate = wptraj[widx];
        if (!lattice->GetCollisionChecker()->IsStateToStateValid(prev_istate, curr_istate)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "        -> path between waypoints in collision");
            return false;
        }
    }

    if (final_rstate != NULL) {
        *final_rstate = wptraj.back();
    }
    return true;
}

static
void GetSuccsLo(
    AdaptiveWorkspaceLattice* lattice,
    const AdaptiveGridState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f)", state.gx, state.gy, state.gz, state.x, state.y, state.z);

    auto* vis_name = "expansion_lo";
    auto m = visual::MakeSphereMarker(
            state.x, state.y, state.z,
            lattice->m_grid->resolution(),
            30,
            lattice->m_grid->getReferenceFrame(),
            vis_name,
            0);

    SV_SHOW_INFO_NAMED(vis_name, m);

    SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "  actions: %zu", lattice->m_lo_prims.size());
    for (auto aidx = 0; aidx < lattice->m_lo_prims.size(); ++aidx) {
        auto& dx = lattice->m_lo_prims[aidx];

        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "    action %zu", aidx);

        double succ_pos[3] =
        {
            state.x + dx.x(),
            state.y + dx.y(),
            state.z + dx.z()
        };

        int tgx, tgy, tgz;
        lattice->m_grid->worldToGrid(
                succ_pos[0], succ_pos[1], succ_pos[2],
                tgx, tgy, tgz);

        if (!lattice->m_grid->isInBounds(tgx, tgy, tgz)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> out of bounds (%d, %d, %d)", tgx, tgy, tgz);
            continue;
        }

        auto sphere_radius = 0.02;
        if (lattice->m_grid->getDistance(tgx, tgy, tgz) < sphere_radius) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> in collision (%d, %d, %d)", tgx, tgy, tgz);
            continue;
        }

        if (IsHighDimensional(lattice, tgx, tgy, tgz)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> high-dimensional");
            // TODO: high-dimensional transitions
            // sample roll, pitch, yaw, and free angles

            auto succ_state = WorkspaceState(lattice->m_proj.dof_count);
            succ_state[0] = succ_pos[0];
            succ_state[1] = succ_pos[1];
            succ_state[2] = succ_pos[2];
            auto yaw_samples = 4;
            auto pitch_samples = 3;
            auto roll_samples = 4;
            // TODO: sampling of free angle vector
//            std::vector<double> fav_samples(freeAngleCount, 4);
            for (auto y = 0; y < yaw_samples; ++y) {
            for (auto p = 0; p < pitch_samples; ++p) {
            for (auto r = 0; r < roll_samples; ++r) {
                auto yaw = y * (2.0 * M_PI) / (double)yaw_samples;
                auto pitch = p * (M_PI / (double)(pitch_samples - 1));
                auto roll = r * (2.0 * M_PI) / (double)roll_samples;
                succ_state[3] = roll;
                succ_state[4] = pitch;
                succ_state[5] = yaw;

                auto succ_coord = WorkspaceCoord();
                auto final_rstate = RobotState();
                StateWorkspaceToCoord(&lattice->m_proj, succ_state, succ_coord);
                if (!StateWorkspaceToRobot(&lattice->m_proj, succ_state, final_rstate)) {
                    continue;
                }

                auto succ_id = GetHashEntryHi(lattice, succ_coord);
                if (succ_id < 0) {
                    succ_id = CreateStateHi(lattice, succ_coord, final_rstate);
                }

                succs->push_back(succ_id);
                costs->push_back(30);
            }
            }
            }
        } else {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> low-dimensional");

            int succ_coord[3];
            PosWorkspaceToCoord(&lattice->m_proj, succ_pos, succ_coord);

            int succ_id = GetHashEntryLo(lattice, tgx, tgy, tgz);
            if (succ_id < 0) {
                succ_id = CreateStateLo(
                    lattice,
                    succ_coord[0], succ_coord[1], succ_coord[2],
                    succ_pos[0], succ_pos[1], succ_pos[2]);
            }

            succs->push_back(succ_id);
            costs->push_back(30);
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "          succ: { id: %d, coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f), cost: %d }", succs->back(), succ_coord[0], succ_coord[1], succ_coord[2], succ_pos[0], succ_pos[1], succ_pos[2], costs->back());
        }
    }
}

static
void GetSuccsHi(
    AdaptiveWorkspaceLattice* lattice,
    const AdaptiveWorkspaceState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << state.coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  state: " << state.state);
    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, GetStateVisualization(lattice, state.state, vis_name));

    auto actions = std::vector<Action>();
    GetActions(lattice, state, actions);

    SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "  actions: %zu", actions.size());
    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      waypoints: %zu", action.size());

        auto final_rstate = RobotState();
        if (!CheckAction(lattice, state.state, action, &final_rstate)) {
            continue;
        }

        auto& final_state = action.back();

        // project state to 3d
        int gx, gy, gz;
        lattice->m_grid->worldToGrid(
                final_state[0], final_state[1], final_state[2], gx, gy, gz);

        if (!lattice->m_grid->isInBounds(gx, gy, gz)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> out of bounds (%d, %d, %d)", gx, gy, gz);
            continue;
        }

        auto succ_coord = WorkspaceCoord();
        StateWorkspaceToCoord(&lattice->m_proj, final_state, succ_coord);
        if (IsHighDimensional(lattice, gx, gy, gz)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> high-dimensional");
            int succ_id = GetHashEntryHi(lattice, succ_coord);
            if (succ_id < 0) {
                succ_id = CreateStateHi(lattice, succ_coord, final_rstate);
            }

            succs->push_back(succ_id);
            costs->push_back(120);

            SMPL_DEBUG_STREAM_NAMED(G_SUCCESSORS_LOG, "         succ: { id: " << succs->back() << ", coord: " << succ_coord << ", state: " << final_rstate << ", cost: " << costs->back() << " }");
        } else if (lattice->m_plan_mode) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> low-dimensional");
            int succ_id = GetHashEntryLo(lattice, succ_coord[0], succ_coord[1], succ_coord[2]);
            if (succ_id < 0) {
                succ_id = CreateStateLo(
                        lattice,
                        succ_coord[0], succ_coord[1], succ_coord[2],
                        final_state[0], final_state[1], final_state[2]);
            }
            succs->push_back(succ_id);
            costs->push_back(120);
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "         succ: { id: %d, coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f), cost: %d }", succs->back(), succ_coord[0], succ_coord[1], succ_coord[2], final_state[0], final_state[1], final_state[2], costs->back());
        } else {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "      -> outside tunnel");
        }
    }
}

AdaptiveWorkspaceLattice::~AdaptiveWorkspaceLattice()
{
    for (auto* state : m_states) {
        if (state->hid) {
            auto* hi_state = (AdaptiveWorkspaceState*)state;
            delete hi_state;
        } else {
            auto* lo_state = (AdaptiveGridState*)state;
            delete lo_state;
        }
    }
}

bool AdaptiveWorkspaceLattice::Init(
    RobotModel* robot,
    CollisionChecker* checker,
    const WorkspaceProjectionParams& params,
    const OccupancyGrid* grid)
{
    if (m_grid == NULL) {
        return false;
    }

    if (!DiscreteSpace::Init(robot, checker)) {
        return false;
    }

    if (!InitWorkspaceProjection(&m_proj, robot, params)) {
        return false;
    }

    m_grid = grid;

    m_dim_grid.assign(
            m_grid->numCellsX(),
            m_grid->numCellsY(),
            m_grid->numCellsZ(),
            AdaptiveGridCell());

    InitMotionPrimitives(this);
    return true;
}

void AdaptiveWorkspaceLattice::PrintState(int state_id, bool verbose, FILE* f)
{
    if (f == NULL) {
        f = stdout;
    }

    auto* state = GetHashEntry(this, state_id);
    std::stringstream ss;
    if (state->hid) {
        auto* hi_state = (AdaptiveWorkspaceState*)state;
        ss << *hi_state;
    } else {
        auto* lo_state = (AdaptiveGridState*)state;
        ss << *lo_state;
    }

    if (f == stdout) {
        SMPL_DEBUG_NAMED(G_LOG, "%s", ss.str().c_str());
    } else if (f == stderr) {
        SMPL_WARN_NAMED(G_LOG, "%s", ss.str().c_str());
    } else {
        fprintf(f, "%s\n", ss.str().c_str());
    }
}

int AdaptiveWorkspaceLattice::GetStateID(const RobotState& state)
{
    if (state.size() < GetRobotModel()->JointVariableCount()) {
        SMPL_ERROR_NAMED(G_LOG, "state contains insufficient coordinate positions");
        return -1;
    }

    if (!GetRobotModel()->CheckJointLimits(state)) {
        SMPL_ERROR_NAMED(G_LOG, "state violates joint limits");
        return false;
    }

    auto state_coord = WorkspaceCoord();
    StateRobotToCoord(&m_proj, state, state_coord);

    auto state_id = GetHashEntryHi(this, state_coord);
    if (state_id < 0) {
        state_id = CreateStateHi(this, state_coord, state);
    }

    return state_id;
}

bool AdaptiveWorkspaceLattice::ExtractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    for (auto i = 0; i < ids.size(); ++i) {
        auto curr_id = ids[i];

        auto* state = GetHashEntryHi(this, curr_id);
        if (state == NULL) {
            SMPL_ERROR_NAMED(G_LOG, "Intermediate state is not high-dimensional");
            return false;
        }
        path.push_back(state->state);
    }

    return true;
}

auto AdaptiveWorkspaceLattice::ProjectToPoint(int state_id) -> Vector3
{
    auto* state = m_states[state_id];
    if (state->hid) {
        auto* hi_state = (AdaptiveWorkspaceState*)state;
        auto state = WorkspaceState();
        StateCoordToWorkspace(&m_proj, hi_state->coord, state);
        return Vector3(state[0], state[1], state[2]);
    } else {
        auto* lo_state = (AdaptiveGridState*)state;
        return Vector3(lo_state->x, lo_state->y, lo_state->z);
    }
}

bool AdaptiveWorkspaceLattice::AddHighDimRegion(int state_id)
{
    // TODO: add high dimensional region around the goal state

    auto* state = m_states[state_id];

    Eigen::Vector3i gp;

    auto real_pt = ProjectToPoint(state_id);
    m_grid->worldToGrid(real_pt.x(), real_pt.y(), real_pt.z(), gp.x(), gp.y(), gp.z());

    SMPL_INFO_NAMED(G_LOG, "Region center: (%d, %d, %d)", gp.x(), gp.y(), gp.z());

    if (!m_grid->isInBounds(gp.x(), gp.y(), gp.z())) {
        SMPL_INFO_NAMED(G_LOG, " -> out of bounds");
        return false;
    }

    ++m_dim_grid(gp.x(), gp.y(), gp.z()).grow_count;
    auto radius = m_region_radius * m_dim_grid(gp.x(), gp.y(), gp.z()).grow_count;
    SMPL_INFO_NAMED(G_LOG, "  radius: %d", radius);

    // TODO: mark cells as in high-dimensional region
    auto marked = 0;
    for (auto dx = -radius; dx <= radius; ++dx) {
    for (auto dy = -radius; dy <= radius; ++dy) {
    for (auto dz = -radius; dz <= radius; ++dz) {
        auto p = Eigen::Vector3i(gp + Eigen::Vector3i(dx, dy, dz));
        if (m_grid->isInBounds(p.x(), p.y(), p.z())) {
            m_dim_grid(p.x(), p.y(), p.z()).plan_hd = true;
            ++marked;
        }
    }
    }
    }

    SMPL_INFO_NAMED(G_LOG, "Marked %d cells as high-dimensional", marked);

    return true;
}

bool AdaptiveWorkspaceLattice::SetTunnel(const std::vector<int>& states)
{
    // clear the tunnel grid
    // TODO: retain the list of points in the tunnel and clear only those points
    for (auto it = m_dim_grid.begin(); it != m_dim_grid.end(); ++it) {
        it->trak_hd = false;
    }

    auto tunnel = std::vector<Eigen::Vector3i>();
    for (auto state_id : states) {
        auto real_pt = ProjectToPoint(state_id);
        auto disc_pt = Eigen::Vector3i();
        m_grid->worldToGrid(real_pt.x(), real_pt.y(), real_pt.z(), disc_pt.x(), disc_pt.y(), disc_pt.z());

        if (!m_grid->isInBounds(disc_pt.x(), disc_pt.y(), disc_pt.z())) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to create tunnel. State (%d, %d, %d) out of bounds", disc_pt.x(), disc_pt.y(), disc_pt.z());
            return false;
        }

        tunnel.emplace_back(disc_pt);
    }

    // TODO: dijkstra/breadth-first search out from tunnel to fill states at
    // tunnel-width away
    auto marked = 0;
    for (auto& gp : tunnel) {
        auto radius = m_tunnel_radius;

        for (auto dx = -radius; dx <= radius; ++dx) {
        for (auto dy = -radius; dy <= radius; ++dy) {
        for (auto dz = -radius; dz <= radius; ++dz) {
            auto p = Eigen::Vector3i(gp + Eigen::Vector3i(dx, dy, dz));
            if (m_grid->isInBounds(p.x(), p.y(), p.z())) {
                m_dim_grid(p.x(), p.y(), p.z()).trak_hd = true;
                ++marked;
            }
        }
        }
        }
    }
    SMPL_INFO_NAMED(G_LOG, "Marked %d cells as tunnel cells", marked);

    return true;
}

bool AdaptiveWorkspaceLattice::IsExecutable(const std::vector<int>& states) const
{
    for (auto state_id : states) {
        auto* state = GetHashEntry(this, state_id);
        if (!state->hid) {
            return false;
        }
    }
    return true;
}

bool AdaptiveWorkspaceLattice::SetTrackMode(const std::vector<int>& tunnel)
{
    if (!SetTunnel(tunnel)) {
        return false;
    }
    m_plan_mode = false;
    SV_SHOW_INFO_NAMED(TrackAdaptiveGridVisName, GetAdaptiveGridVisualization(this, m_plan_mode));
    return true;
}

bool AdaptiveWorkspaceLattice::SetPlanMode()
{
    m_plan_mode = true;
    SV_SHOW_INFO_NAMED(PlanAdaptiveGridVisName, GetAdaptiveGridVisualization(this, m_plan_mode));
    return true;
}

void AdaptiveWorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);

    auto* state = m_states[state_id];
    if (state->hid) {
        auto* hi_state = (AdaptiveWorkspaceState*)state;
        GetSuccsHi(this, *hi_state, succs, costs);
    } else {
        auto* lo_state = (AdaptiveGridState*)state;
        GetSuccsLo(this, *lo_state, succs, costs);
    }
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, " -> %zu successors", succs->size());
}

auto AdaptiveWorkspaceLattice::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<DiscreteSpace>() ||
        class_id == GetClassCode<RobotPlanningSpace>() ||
        class_id == GetClassCode<IExtractRobotState>() ||
        class_id == GetClassCode<IProjectToPoint>() ||
        class_id == GetClassCode<IAdaptiveGraph>() ||
        class_id == GetClassCode<ISearchable>())
    {
        return this;
    }
    return NULL;
}

} // namespace smpl
