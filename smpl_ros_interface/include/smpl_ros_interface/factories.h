#ifndef SMPL_ROS_FACTORIES_H
#define SMPL_ROS_FACTORIES_H

// standard includes
#include <memory>
#include <string>

namespace smpl {

class CollisionChecker;
class OccupancyGrid;
class PlanningParams;
class RobotPlanningSpace;
class RobotModel;
class Heuristic;
class Search;

bool IsMultiDOFJointVariable(
    const std::string& name,
    std::string* joint_name = NULL,
    std::string* local_name = NULL);

//////////////////////////////
// Planning Space Factories //
//////////////////////////////

auto MakeManipLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>;

auto MakeManipLatticeEGraph(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>;

auto MakeWorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>;

auto MakeWorkspaceLatticeEGraph(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>;

auto MakeAdaptiveWorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<RobotPlanningSpace>;

/////////////////////////
// Heuristic Factories //
/////////////////////////

auto MakeMultiFrameBFSHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<Heuristic>;

auto MakeBFSHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& param,
    const OccupancyGrid* grid)
    -> std::unique_ptr<Heuristic>;

auto MakeEuclidDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<Heuristic>;

auto MakeJointDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<Heuristic>;

auto MakeDijkstraEgraphHeuristic3D(
    RobotPlanningSpace* space,
    const PlanningParams& params,
    const OccupancyGrid* grid)
    -> std::unique_ptr<Heuristic>;

auto MakeJointDistEGraphHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<Heuristic>;

//////////////////////
// Search Factories //
//////////////////////

auto MakeARAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

auto MakeAWAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

auto MakeMHAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

auto MakeLARAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

auto MakeEGWAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

auto MakePADAStar(
    RobotPlanningSpace* space,
    Heuristic* heuristic,
    const PlanningParams& params)
    -> std::unique_ptr<Search>;

} // namespace smpl

#endif

