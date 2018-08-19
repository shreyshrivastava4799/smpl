#include <smpl/graph/simple_workspace_lattice_action_space.h>

// project includes
#include <smpl/angles.h>
#include <smpl/robot_model.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {

bool InitSimpleWorkspaceLatticeActions(
    WorkspaceLattice* space,
    SimpleWorkspaceLatticeActionSpace* actions)
{
    actions->space = space;

    actions->m_prims.clear();

    MotionPrimitive prim;

    auto add_xyz_prim = [&](int dx, int dy, int dz)
    {
        std::vector<double> d(space->dofCount(), 0.0);
        d[FK_PX] = space->resolution()[FK_PX] * dx;
        d[FK_PY] = space->resolution()[FK_PY] * dy;
        d[FK_PZ] = space->resolution()[FK_PZ] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        actions->m_prims.push_back(prim);
    };

#if 0
    // create 26-connected position motions
    for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
    for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }
        add_xyz_prim(dx, dy, dz);
    }
    }
    }
#endif

    add_xyz_prim(-1, 0, 0);
    add_xyz_prim(1, 0, 0);
    add_xyz_prim(0, 1, 0);
    add_xyz_prim(0, -1, 0);
    add_xyz_prim(0, 0, 1);
    add_xyz_prim(0, 0, -1);

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < space->dofCount(); ++a) {
        std::vector<double> d(space->dofCount(), 0.0);

        d[a] = space->resolution()[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);

        d[a] = space->resolution()[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);
    }

    return true;
}

void SimpleWorkspaceLatticeActionSpace::apply(
    const WorkspaceLatticeState& state,
    std::vector<WorkspaceAction>& actions)
{
    actions.reserve(actions.size() + m_prims.size());

    WorkspaceState cont_state;
    space->stateCoordToWorkspace(state.coord, cont_state);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  create actions for workspace state: " << cont_state);

    for (auto& prim : m_prims) {
        WorkspaceAction action;
        action.reserve(prim.action.size());

        auto final_state = cont_state;
        for (auto& delta_state : prim.action) {
            // increment the state
            for (size_t d = 0; d < space->dofCount(); ++d) {
                final_state[d] += delta_state[d];
            }

            angles::normalize_euler_zyx(&final_state[3]);

            action.push_back(final_state);
        }

        actions.push_back(std::move(action));
    }

    if (m_ik_amp_enabled && space->numHeuristics() > 0) {
        auto* h = space->heuristic(0);
        auto goal_dist = h->getMetricGoalDistance(
                cont_state[FK_PX], cont_state[FK_PY], cont_state[FK_PZ]);
        if (goal_dist < m_ik_amp_thresh) {
            RobotState ik_sol;
            if (space->m_ik_iface->computeIK(space->goal().pose, state.state, ik_sol)) {
                WorkspaceState final_state;
                space->stateRobotToWorkspace(ik_sol, final_state);
                WorkspaceAction action(1);
                action[0] = final_state;
                actions.push_back(std::move(action));
            }
        }
    }
}

} // namespace smpl

