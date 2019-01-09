#ifndef SMPL_GOAL_CONSTRAINT_H
#define SMPL_GOAL_CONSTRAINT_H

// standard includes
#include <vector>

// system includes
#include <Eigen/StdVector>

// project includes
#include <smpl/extension.h>
#include <smpl/spatial.h>
#include <smpl/types.h>
#include <smpl/debug/marker.h>

namespace smpl {

class DiscreteSpace;
class IExtractRobotState;
class IProjectToPose;
class IProjectToPoint;
class IForwardKinematics;

class GoalConstraint : public virtual Extension
{
public:

    virtual ~GoalConstraint() { }

    virtual bool Init(DiscreteSpace* space);

    virtual bool IsGoal(int state_id) = 0;

    auto GetPlanningSpace() -> DiscreteSpace*;
    auto GetPlanningSpace() const -> const DiscreteSpace*;

public:

    DiscreteSpace* m_space;
};

// Return the pose of at least one goal state.
class IGetPose : public virtual Extension
{
public:

    virtual auto GetPose() -> Affine3 = 0;
};

class IGetPoseArray : public virtual Extension
{
public:

    virtual auto GetPoseArray() -> std::pair<Affine3*, int> = 0;
};

// Return the position of at least one goal state.
class IGetPosition : public virtual Extension
{
public:

    virtual auto GetPosition() -> Vector3 = 0;
};

// Return the robot state of at least one goal state.
class IGetRobotState : public virtual Extension
{
public:

    virtual auto GetState() -> RobotState = 0;
};

// The most specific kind of goal constraint. One unique state is identified as
// the goal state. May provide a GetState and various projection interfaces if
// the associated robot model/graph/collision detector support the necessary
// interfaces.
class UniqueGoalState :
    public GoalConstraint,
    private IGetPose,
    private IGetPosition,
    private IGetRobotState
{
public:

    int GetGoalStateID() const;
    void SetGoalStateID(int state_id);

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    int m_goal_state_id = -1;

    IExtractRobotState* m_extract_robot_state = NULL;
    IProjectToPose*     m_project_to_pose = NULL;
    IProjectToPoint*    m_project_to_point = NULL;

    auto GetState() -> RobotState final;
    auto GetPose() -> Affine3 final;
    auto GetPosition() -> Vector3 final;
};

// Represents the set of all states within some tolerance, along each axis,
// around a unique state
class JointStateGoal :
    public GoalConstraint,
    public IGetRobotState,
    private IGetPosition,
    private IGetPose
{
public:

    bool Init(DiscreteSpace* space) final;

    auto GetGoalState() const -> const RobotState&;
    void SetGoalState(const RobotState& state);

    auto GetGoalTolerance() const -> const std::vector<double>&;
    void SetGoalTolerance(const std::vector<double>& tolerance);

    auto GetState() -> RobotState final;

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    IExtractRobotState* m_extract_state = NULL;
    IForwardKinematics* m_fk = NULL;

    RobotState          m_state;
    std::vector<double> m_tolerance;

    auto GetPose() -> Affine3 final;
    auto GetPosition() -> Vector3 final;
};

class PointGoal :
    public GoalConstraint,
    public IGetPosition
{
public:

    Vector3 point;
    double tolerance[3];

    bool Init(DiscreteSpace* space) final;

    auto GetPosition() -> Vector3 final;

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    IProjectToPoint* m_project_to_point = NULL;
};

struct PoseTolerance
{
    double xyz[3];
    double rpy[3];
};

class PoseGoal :
    public GoalConstraint,
    public IGetPose,
    public IGetPosition
{
public:

    Affine3 pose;
    PoseTolerance tolerance;

    bool Init(DiscreteSpace* space) final;

    auto GetPose() -> Affine3 final;
    auto GetPosition() -> Vector3 final;

    auto GetVisualization(const std::string& frame_id) const -> std::vector<visual::Marker>;

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    IProjectToPose* m_project_to_pose = NULL;
};

template <class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

class MultiPoseGoal :
    public GoalConstraint,
    public IGetPoseArray
{
public:

    AlignedVector<Affine3> poses;
    PoseTolerance tolerance;

    auto GetPoseArray() -> std::pair<Affine3*, int> final;

    bool Init(DiscreteSpace* space) final;

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    IProjectToPose* m_project_to_pose = NULL;
};

using GoalConstraintFn = bool (*)(void* user, const RobotState& state);

class UserGoal : public GoalConstraint
{
public:

    GoalConstraintFn check_goal = NULL;
    void* check_goal_user = NULL;

    bool Init(DiscreteSpace* space) final;

    bool IsGoal(int state_id) final;

    auto GetExtension(size_t class_id) -> Extension* final;

public:

    IExtractRobotState* m_extract_state = 0;
};

} // namespace smpl

#endif

