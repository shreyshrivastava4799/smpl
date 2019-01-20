// standard includes
#include <cmath>
#include <iostream>
#include <chrono>

// system includes
#define SMPL_CONSOLE_ROS
#include <smpl/collision_checker.h>
#include <smpl/console/ansi.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/cost_function.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manipulation_action_space.h>
#include <smpl/graph/goal_constraint.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/zero_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>

template <class CharT, class Traits = std::char_traits<CharT>>
auto donothing(std::basic_ostream<CharT, Traits>& o)
    -> std::basic_ostream<CharT, Traits>&
{ return o; }

const bool g_colorize = true;
#define COLOR(o, color) if (g_colorize) { o << (color); }

/// \brief Defines a Robot Model for an (x, y) point robot
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// IForwardKinematics: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class KinematicVehicleModel :
    public virtual smpl::RobotModel,
    public smpl::IForwardKinematics
{
public:

    KinematicVehicleModel() : smpl::RobotModel(), smpl::IForwardKinematics()
    {
        const std::vector<std::string> joint_names = { "x", "y" };
        setPlanningJoints(joint_names);
    }

    /// \name Required Public Functions from IForwardKinematics
    ///@{
    Eigen::Affine3d computeFK(const smpl::RobotState& state) override
    {
        return Eigen::Affine3d(Eigen::Translation3d(state[0], state[1], 0.0));
    }
    ///@}

    /// \name Required Public Functions from Robot Model
    ///@{
    double minPosLimit(int jidx) const override { return 0.0; }
    double maxPosLimit(int jidx) const override { return 0.0; }
    bool hasPosLimit(int jidx) const override { return false; }
    bool isContinuous(int jidx) const override { return false; }
    double velLimit(int jidx) const override { return 0.0; }
    double accLimit(int jidx) const override { return 0.0; }

    bool checkJointLimits(
        const smpl::RobotState& angles,
        bool verbose = false) override
    {
        return true;
    }
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final
    {
        if (class_code == smpl::GetClassCode<RobotModel>() ||
            class_code == smpl::GetClassCode<IForwardKinematics>())
        {
            return this;
        } else {
            return nullptr;
        }
    }
    ///@}
};

/// \brief Defines a collision checker for an (x,y) point robot in a grid world.
class GridCollisionChecker : public smpl::CollisionChecker
{
public:

    GridCollisionChecker(smpl::OccupancyGrid* grid) :
        Extension(), m_grid(grid)
    { }

    /// \name Required Functions from Extension
    ///@{
    auto GetExtension(size_t class_code) -> Extension* final
    {
        if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
            return this;
        }
        return nullptr;
    }
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(const smpl::RobotState& state, bool verbose) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;
    ///@}

private:

    // a bit heavy-weight for this, since it overlays a distance transform
    smpl::OccupancyGrid* m_grid;
};

bool GridCollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    if (state.size() < 2) {
        SMPL_ERROR("State contains insufficient data");
        return false;
    }
    double x = state[0];
    double y = state[1];
    double z = 0.0;
    if (!m_grid->isInBounds(x, y, z)) {
        SMPL_DEBUG("state (%0.3f, %0.3f) is out of bounds", x, y);
        return false;
    }
    if (m_grid->getDistanceFromPoint(x, y, z) <= 0.0) {
        SMPL_DEBUG("state (%0.3f, %0.3f) is occupied", x, y);
        return false;
    }
    return true;
}

bool GridCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    std::vector<smpl::RobotState> path;
    if (!interpolatePath(start, finish, path)) {
        return false;
    }
    return std::all_of(
        path.begin(), path.end(),
        [&](const smpl::RobotState& state)
        {
            return isStateValid(state, false);
        });
}

bool GridCollisionChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    m_grid->resolution();
    const Eigen::Vector2d vstart(start[0], start[1]);
    const Eigen::Vector2d vfinish(finish[0], finish[1]);
    int num_waypoints =
            (int)std::ceil((vfinish - vstart).norm() / m_grid->resolution());
    num_waypoints = std::max(num_waypoints, 2);
    SMPL_DEBUG("interpolate path with %d waypoints", num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
        const double alpha = (double)i / (double)(num_waypoints - 1);
        Eigen::Vector2d vinterm = (1.0 - alpha) * vstart + alpha * vfinish;
        smpl::RobotState istate(2);
        istate[0] = vinterm.x();
        istate[1] = vinterm.y();
        path.push_back(std::move(istate));
    }
    return true;
}

/// Add a simple box obstacle in the center of the grid
void SetupOccupancyGrid(smpl::OccupancyGrid& grid)
{
    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    std::vector<Eigen::Vector3d> points;

    // add horizontal strip down the middle, with holes on the ends
    for (int gx = 1; gx < x_count - 1; ++gx) {
        double cx, cy, cz;
        grid.gridToWorld(gx, y_count >> 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) + 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) - 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);
    }

    SMPL_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}

void PrintGrid(std::ostream& o, smpl::OccupancyGrid& grid)
{
    COLOR(o, smpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, smpl::console::reset);
    o << '\n';

    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        COLOR(o, smpl::console::yellow);
        o << '|';
        COLOR(o, smpl::console::reset);
        for (int x = 0; x < grid.numCellsX(); ++x) {
            if (grid.getDistance(x, y, 0) <= 0) {
                COLOR(o, smpl::console::red);
                o << "X";
                COLOR(o, smpl::console::reset);
            } else {
                o << " ";
            }
        }
        COLOR(o, smpl::console::yellow);
        o << '|';
        COLOR(o, smpl::console::reset);
        o << '\n';
    }

    COLOR(o, smpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, smpl::console::reset);
    o << '\n';
}

void PrintActionSpace(const smpl::ManipulationActionSpace& actions)
{
    SMPL_INFO("Action Space:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES; ++i) {
        auto prim_type = (smpl::MotionPrimitive::Type)i;
        if (actions.IsMotionTypeEnabled(prim_type)) {
            SMPL_INFO("  %s enabled @ %0.3f",
                    to_cstring(prim_type),
                    actions.GetMotionTypeThreshold(prim_type));
        } else {
            SMPL_INFO("  %s disabled", to_cstring(prim_type));
        }
    }

    SMPL_INFO("short distance motion primitives:");
    for (auto& prim : actions.m_short_dist_mprims) {
        SMPL_INFO_STREAM("  " << prim.action);
    }
    SMPL_INFO("long distance motion primitives:");
    for (auto& prim : actions.m_long_dist_mprims) {
        SMPL_INFO_STREAM("  " << prim.action);
    }
}

void PrintSolution(
    std::ostream& o,
    const smpl::OccupancyGrid& grid,
    const std::vector<smpl::RobotState>& path)
{
    std::vector<std::pair<int, int>> discrete_states;
    for (auto& point : path) {
        int dx = (int)(point[0] / grid.resolution());
        int dy = (int)(point[1] / grid.resolution());
        discrete_states.push_back(std::make_pair(dx, dy));
    }

    COLOR(o, smpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, smpl::console::reset);
    o << '\n';

    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        COLOR(o, smpl::console::yellow);
        o << '|';
        COLOR(o, smpl::console::reset);
        for (int x = 0; x < grid.numCellsX(); ++x) {
            auto it = std::find(begin(discrete_states), end(discrete_states), std::make_pair(x, y));
            if (it != end(discrete_states)) {
                COLOR(o, smpl::console::cyan);
                o << 'P';
                COLOR(o, smpl::console::reset);
            } else {
                if (grid.getDistance(x, y, 0) <= 0) {
                    COLOR(o, smpl::console::red);
                    o << 'X';
                    COLOR(o, smpl::console::reset);
                } else {
                    o << " ";
                }
            }
        }
        COLOR(o, smpl::console::yellow);
        o << '|';
        COLOR(o, smpl::console::reset);
        o << '\n';
    }

    COLOR(o, smpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, smpl::console::reset);
    o << '\n';
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: xytheta <mprim filepath>\n");
        return 1;
    }

    auto* mprim_path = argv[1];
    SMPL_INFO("Load motion primitives from %s", mprim_path);

    ////////////////////////////////
    // Initialize the Robot Model //
    ////////////////////////////////

    auto robot_model = KinematicVehicleModel();

    auto res = 1.0; //0.02; // match resolution of grid and state space

    //////////////////////////////////////
    // Initialize the Collision Checker //
    //////////////////////////////////////

    auto grid_res = res;
    auto world_size_x = 50.0;
    auto world_size_y = 50.0;
    auto world_size_z = 1.5 * grid_res;
    auto world_origin_x = 0.0;
    auto world_origin_y = 0.0;
    auto world_origin_z = 0.0;
    auto max_distance_m = 4.0;
    auto ref_count = false;
    auto grid = smpl::OccupancyGrid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
    SetupOccupancyGrid(grid);
    PrintGrid(std::cout, grid);

    auto cc = GridCollisionChecker(&grid);

    ///////////////////////////////////
    // Initialize the Discrete Space //
    ///////////////////////////////////

    auto resolutions = std::vector<double>{ res, res };
    auto actions = smpl::ManipulationActionSpace();
#if 1
    auto cost_fun = smpl::L2NormCostFunction();
#else
    auto cost_fun = smpl::UniformCostFunction();
    cost_fun.cost_per_action = 1000;
#endif

    auto space = smpl::ManipLattice();

    if (!space.Init(&robot_model, &cc, resolutions, &actions, &cost_fun)) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    space.SetVisualizationFrameId("map"); // for correct rviz visualization

    if (!actions.Init(&space)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }

    if (!actions.Load(mprim_path)) {
        return 1;
    }
    PrintActionSpace(actions);

    if (!cost_fun.Init(&space)) {
        SMPL_ERROR("Failed to initialize Cost Function");
        return 1;
    }

    ////////////////////////////////////
    // Initialize the Goal Constraint //
    ////////////////////////////////////

    auto goal = smpl::UniqueGoalState();
    if (!goal.Init(&space)) {
        SMPL_ERROR("Failed to initialize goal constraint");
        return 1;
    }

    //////////////////////////////
    // Initialize the Heuristic //
    //////////////////////////////

#if 1
    auto h = smpl::JointDistHeuristic();
#else
    auto h = smpl::ZeroHeuristic();
#endif
    if (!h.Init(&space)) {
        SMPL_ERROR("Failed to initialize Joint Dist Heuristic");
        return 1;
    }

    //////////////////////////////////////////////
    // Associate Heuristics with Discrete Space //
    //////////////////////////////////////////////

    // TODO: kinda lame that we have to maintain this pointer...maybe its worth
    // having add/remove functions instead
    smpl::Heuristic* hp = &h;
    if (!space.UpdateHeuristics(&hp, 1)) {
        SMPL_ERROR("Failed to update heuristics for some reason");
        return 1;
    }

    ///////////////////////
    // Initialize Search //
    ///////////////////////

    auto search = smpl::ARAStar();
    if (!Init(&search, &space, &h)) {
        SMPL_ERROR("Failed to initialize ARA*");
        return 1;
    }

    /////////////////////////////
    // Update Planning Problem //
    /////////////////////////////

    auto start_x = 0.5 * world_size_x;
    auto start_y = 0.33 * world_size_y;
    auto start_state = space.GetDiscreteCenter({ start_x, start_y });
    auto start_state_id = space.GetStateID(start_state);

    auto goal_x = 0.5 * world_size_x;
    auto goal_y = 0.66 * world_size_y;
    auto goal_state = space.GetDiscreteCenter({ goal_x, goal_y });
    auto goal_state_id = space.GetStateID(goal_state);

    goal.SetGoalStateID(goal_state_id);

    if (!space.UpdateStart(start_state_id)) {
        SMPL_ERROR("Failed to update start in the graph");
        return 1;
    }

    if (!space.UpdateGoal(&goal)) {
        SMPL_ERROR("Failed to update goal in the graph");
        return 1;
    }

    if (!h.UpdateStart(start_state_id)) {
        SMPL_ERROR("Failed to update start in the heuristic");
        return 1;
    }
    if (!h.UpdateGoal(&goal)) {
        SMPL_ERROR("Failed to update goal in the heuristic");
        return 1;
    }

    if (!search.UpdateStart(start_state_id)) {
        SMPL_ERROR("Failed to update start in the search");
        return 1;
    }

    if (!search.UpdateGoal(&goal)) {
        SMPL_ERROR("Failed to update goal in the search");
        return 1;
    }

    /////////////////
    // Plan a path //
    /////////////////

#if 1
    SetInitialEps(&search, 5.0);
#else
    SetInitialEps(&search, 1.0);
#endif
    SetDeltaEps(&search, 0.2);

    auto search_params = smpl::TimeoutCondition();
    search_params.type = smpl::TimeoutCondition::TIME;
    search_params.bounded = false; //true;
    search_params.improve = true;
    search_params.max_allowed_time_init = std::chrono::seconds(1);
    search_params.max_allowed_time = std::chrono::seconds(1);

    auto then = std::chrono::high_resolution_clock::now();
    auto solution = std::vector<int>();
    int solcost;
    auto bret = (bool)Replan(&search, search_params, &solution, &solcost);
    if (!bret) {
        SMPL_ERROR("Search failed to find a solution");
        return 1;
    }
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<double>(now - then).count();

    //////////////////////////////////////
    // Extract path from Planning Space //
    //////////////////////////////////////

    auto path = std::vector<smpl::RobotState>();
    if (!space.ExtractPath(solution, path)) {
        SMPL_ERROR("Failed to extract path");
    }

    auto print_solution = false;
    auto print_path = false;
    auto draw_path = true;

    SMPL_INFO("Path found!");
    SMPL_INFO("  Solution Cost: %d", solcost);
    SMPL_INFO("  Final Epsilon: %f", GetSolutionEps(&search));
    SMPL_INFO("  Planning Time: %0.3f", elapsed);
    SMPL_INFO("  Expansion Count (total): %d", GetNumExpansions(&search));
    SMPL_INFO("  Expansion Count (initial): %d", GetNumExpansionsInitialEps(&search));

    SMPL_INFO("  Solution (%zu)", solution.size());
    if (print_solution) {
        for (auto state_id : solution) {
            SMPL_INFO("    %d", state_id);
        }
    }
    SMPL_INFO("  Path (%zu)", path.size());

    if (print_path) {
        for (auto& point : path) {
            SMPL_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
        }
    }

    if (draw_path) {
        PrintSolution(std::cout, grid, path);
    }

    return 0;
}
