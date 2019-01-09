#include <smpl/graph/goal_constraint.h>

// project includes
#include <smpl/angles.h>
#include <smpl/robot_model.h>
#include <smpl/graph/discrete_space.h>
#include <smpl/debug/marker_utils.h>

namespace smpl {

static
bool WithinPositionTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    auto dx = std::fabs(A.translation()[0] - B.translation()[0]);
    auto dy = std::fabs(A.translation()[1] - B.translation()[1]);
    auto dz = std::fabs(A.translation()[2] - B.translation()[2]);
    return (dx <= tol[0]) & (dy <= tol[1]) & (dz <= tol[2]);
}

static
bool WithinTolerance(const Vector3& u, const Vector3& v, const double tol[3])
{
    auto dx = v.x() - u.x();
    auto dy = v.y() - u.y();
    auto dz = v.z() - u.z();
    return (dx <= tol[0]) & (dy <= tol[1]) & (dz <= tol[2]);
}

static
bool WithinOrientationTolerance(
    const Affine3& A,
    const Affine3& B,
    const double tol[3])
{
    auto qg = Quaternion(B.rotation());
    auto q = Quaternion(A.rotation());
    auto dist = GetAngularDistance(q, qg);
    return dist <= tol[0];
}

static
auto WithinTolerance(
    const Affine3& A,
    const Affine3& B,
    const PoseTolerance& tolerance)
    -> bool
{
    return WithinPositionTolerance(A, B, tolerance.xyz) &&
            WithinOrientationTolerance(A, B, tolerance.rpy);
}

////////////////////
// GoalConstraint //
////////////////////

bool GoalConstraint::Init(DiscreteSpace* space)
{
    if (space == NULL) return false;
    m_space = space;
    return true;
}

auto GoalConstraint::GetPlanningSpace() -> DiscreteSpace*
{
    return m_space;
}

auto GoalConstraint::GetPlanningSpace() const -> const DiscreteSpace*
{
    return m_space;
}

/////////////////////
// UniqueGoalState //
/////////////////////

int UniqueGoalState::GetGoalStateID() const
{
    return m_goal_state_id;
}

void UniqueGoalState::SetGoalStateID(int state_id)
{
    m_goal_state_id = state_id;
}

bool UniqueGoalState::IsGoal(int state_id)
{
    return state_id == m_goal_state_id;
}

auto UniqueGoalState::GetExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<GoalConstraint>()) {
        return this;
    }

    if (class_code == GetClassCode<IGetRobotState>()) {
        auto* space = GetPlanningSpace();
        auto* accessor = space->GetExtension<IExtractRobotState>();
        if (accessor != NULL) {
            m_extract_robot_state = accessor;
            return this;
        }
    }

    if (class_code == GetClassCode<IGetPose>()) {
        auto* space = GetPlanningSpace();
        auto* projection = space->GetExtension<IProjectToPose>();
        if (projection != NULL) {
            m_project_to_pose = projection;
            return this;
        }
    }

    if (class_code == GetClassCode<IGetPosition>()) {
        auto* space = GetPlanningSpace();
        auto* projection = space->GetExtension<IProjectToPoint>();
        if (projection != NULL) {
            m_project_to_point = projection;
            return this;
        }
    }

    return NULL;
}

auto UniqueGoalState::GetState() -> RobotState
{
    return m_extract_robot_state->ExtractState(m_goal_state_id);
}

auto UniqueGoalState::GetPose() -> Affine3
{
    return m_project_to_pose->ProjectToPose(m_goal_state_id);
}

auto UniqueGoalState::GetPosition() -> Vector3
{
    return m_project_to_point->ProjectToPoint(m_goal_state_id);
}

////////////////////
// JointStateGoal //
////////////////////

bool JointStateGoal::Init(DiscreteSpace* space)
{
    auto* access_state = space->GetExtension<IExtractRobotState>();
    if (access_state == NULL) return false;

    if (!GoalConstraint::Init(space)) return false;

    m_extract_state = access_state;
    return true;
}

auto JointStateGoal::GetGoalState() const -> const RobotState&
{
    return m_state;
}

void JointStateGoal::SetGoalState(const RobotState& state)
{
    m_state = state;
}

auto JointStateGoal::GetGoalTolerance() const -> const std::vector<double>&
{
    return m_tolerance;
}

void JointStateGoal::SetGoalTolerance(const std::vector<double>& tolerance)
{
    m_tolerance = tolerance;
}

auto JointStateGoal::GetState() -> RobotState
{
    return m_state;
}

bool JointStateGoal::IsGoal(int state_id)
{
    assert(m_state.size() == m_tolerance.size());

    auto robot_model = GetPlanningSpace()->GetRobotModel();

    auto& state = m_extract_state->ExtractState(state_id);
    assert(state.size() == m_state.size());

    for (auto i = 0; i < m_state.size(); i++) {
        if (robot_model->isContinuous(i)) {
            auto dist = shortest_angle_dist(m_state[i], state[i]);
            if (dist > m_tolerance[i]) {
                return false;
            }
        } else {
            auto dist = std::fabs(state[i] - m_state[i]);
            if (dist > m_tolerance[i]) {
                return false;
            }
        }
    }

    return true;
}

auto JointStateGoal::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<GoalConstraint>() ||
        class_id == GetClassCode<IGetRobotState>())
    {
        return this;
    }

    if (class_id == GetClassCode<IGetPosition>() ||
        class_id == GetClassCode<IGetPose>())
    {
        if (m_fk == NULL) {
            auto* space = GetPlanningSpace();
            auto* robot_model = space->GetRobotModel();
            auto* fk = robot_model->GetExtension<IForwardKinematics>();
            if (fk != NULL) {
                m_fk = fk;
                return this;
            }
        } else {
            return this;
        }
    }

    return NULL;
}

auto JointStateGoal::GetPose() -> Affine3
{
    return m_fk->computeFK(m_state);
}

auto JointStateGoal::GetPosition() -> Vector3
{
    return m_fk->computeFK(m_state).translation();
}

///////////////
// PointGoal //
///////////////

bool PointGoal::Init(DiscreteSpace* space)
{
    auto* project_to_point = space->GetExtension<IProjectToPoint>();
    if (project_to_point == NULL) return false;

    if (!GoalConstraint::Init(space)) return false;

    m_project_to_point = project_to_point;
    return true;
}

auto PointGoal::GetPosition() -> Vector3
{
    return point;
}

bool PointGoal::IsGoal(int state_id)
{
    auto p = m_project_to_point->ProjectToPoint(state_id);
    return WithinTolerance(p, this->point, this->tolerance);
}

auto PointGoal::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<GoalConstraint>() ||
        class_id == GetClassCode<IGetPosition>())
    {
        return this;
    }

    return NULL;
};

//////////////
// PoseGoal //
//////////////

bool PoseGoal::Init(DiscreteSpace* space)
{
    auto* project_to_pose = space->GetExtension<IProjectToPose>();
    if (project_to_pose == NULL) return false;

    if (!GoalConstraint::Init(space)) return false;

    m_project_to_pose = project_to_pose;
    return true;
}

auto PoseGoal::GetPose() -> Affine3
{
    return pose;
}

auto PoseGoal::GetPosition() -> Vector3
{
    return pose.translation();
}

auto PoseGoal::GetVisualization(const std::string& frame_id) const -> std::vector<visual::Marker>
{
    return visual::MakePoseMarkers(this->pose, frame_id, "pose_goal");
}

bool PoseGoal::IsGoal(int state_id)
{
    auto pose = m_project_to_pose->ProjectToPose(state_id);
    return WithinTolerance(pose, this->pose, this->tolerance);
}

auto PoseGoal::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<GoalConstraint>() ||
        class_id == GetClassCode<IGetPose>() ||
        class_id == GetClassCode<IGetPosition>())
    {
        return this;
    }
    return NULL;
}

///////////////////
// MultiPoseGoal //
///////////////////

auto MultiPoseGoal::GetPoseArray() -> std::pair<Affine3*, int>
{
    return std::make_pair(poses.data(), (int)poses.size());
}

bool MultiPoseGoal::Init(DiscreteSpace* space)
{
    auto* project_to_pose = space->GetExtension<IProjectToPose>();
    if (project_to_pose == NULL) return false;

    if (!GoalConstraint::Init(space)) return false;

    m_project_to_pose = project_to_pose;
    return true;
}

bool MultiPoseGoal::IsGoal(int state_id)
{
    auto pose = m_project_to_pose->ProjectToPose(state_id);
    for (auto& goal_pose : poses) {
        if (WithinTolerance(goal_pose, pose, tolerance)) {
            return true;
        }
    }
    return false;
}

auto MultiPoseGoal::GetExtension(size_t class_id) -> Extension*
{
    if (class_id == GetClassCode<GoalConstraint>() ||
        class_id == GetClassCode<IGetPoseArray>())
    {
        return this;
    }
    return NULL;
}

} // namespace smpl
