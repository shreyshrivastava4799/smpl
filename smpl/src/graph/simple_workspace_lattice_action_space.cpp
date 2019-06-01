#include <smpl/graph/simple_workspace_lattice_action_space.h>

// project includes
#include <smpl/angles.h>
#include <smpl/robot_model.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/planning_params.h>
#include <smpl/heuristic/heuristic.h>
#include <smpl/graph/goal_constraint.h>

namespace smpl {

bool InitSimpleWorkspaceLatticeActions(
    WorkspaceLattice* space,
    SimpleWorkspaceLatticeActionSpace* actions)
{
    actions->space = space;

    actions->m_prims.clear();

    auto prim = MotionPrimitive();

    auto add_xyz_prim = [&](int dx, int dy, int dz)
    {
        auto d = std::vector<double>(GetNumDOFs(&space->m_proj), 0.0);
        d[FK_PX] = space->m_proj.res[FK_PX] * dx;
        d[FK_PY] = space->m_proj.res[FK_PY] * dy;
        d[FK_PZ] = space->m_proj.res[FK_PZ] * dz;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        actions->m_prims.push_back(prim);
    };

#if 0
    // create 26-connected position motions
    for (auto dx = -1; dx <= 1; ++dx) {
    for (auto dy = -1; dy <= 1; ++dy) {
    for (auto dz = -1; dz <= 1; ++dz) {
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
    for (auto a = 3; a < space->m_proj.dof_count; ++a) {
        std::vector<double> d(space->m_proj.dof_count, 0.0);

        d[a] = space->m_proj.res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);

        d[a] = space->m_proj.res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);
    }

    return true;
}

void SimpleWorkspaceLatticeActionSpace::Apply(
    const WorkspaceLatticeState& state,
    std::vector<WorkspaceAction>& actions)
{
    actions.reserve(actions.size() + m_prims.size());

    auto cont_state = WorkspaceState();
    StateCoordToWorkspace(&space->m_proj, state.coord, cont_state);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  create actions for workspace state: " << cont_state);

    for (auto& prim : m_prims) {
        auto action = WorkspaceAction();
        action.reserve(prim.action.size());

        auto final_state = cont_state;
        for (auto& delta_state : prim.action) {
            // increment the state
            for (size_t d = 0; d < space->m_proj.dof_count; ++d) {
                final_state[d] += delta_state[d];
            }

            angles::normalize_euler_zyx(&final_state[3]);

            action.push_back(final_state);
        }

        actions.push_back(std::move(action));
    }

    if (m_ik_amp_enabled &
        (m_goal_heuristic != NULL) &
        (m_get_goal_pose != NULL))
    {
        SMPL_DEBUG("attempt ik");
        auto goal_dist = m_goal_heuristic->GetMetricGoalDistance(
                cont_state[FK_PX], cont_state[FK_PY], cont_state[FK_PZ]);
        if (goal_dist < m_ik_amp_thresh) {
            SMPL_DEBUG("try ik");
            auto ik_sol = RobotState();
            auto goal_pose = m_get_goal_pose->GetPose();
            if (space->m_proj.ik_iface->ComputeIK(
                    goal_pose, state.state, ik_sol))
            {
                SMPL_DEBUG(" -> ik succeeded");
                auto final_state = WorkspaceState();
                StateRobotToWorkspace(&space->m_proj, ik_sol, final_state);
                auto action = WorkspaceAction(1);
                action[0] = std::move(final_state);
                actions.push_back(std::move(action));
            }
        }
    }
}

bool SimpleWorkspaceLatticeActionSpace::UpdateHeuristics(
    Heuristic** heuristics,
    int count)
{
    if (count > 0) {
        auto* h_first = heuristics[0];
        m_goal_heuristic = h_first->GetExtension<IMetricGoalHeuristic>();
    }
    return true;
}

bool SimpleWorkspaceLatticeActionSpace::UpdateGoal(GoalConstraint* goal)
{
    m_get_goal_pose = goal->GetExtension<IGetPose>();
    return true;
}

} // namespace smpl

