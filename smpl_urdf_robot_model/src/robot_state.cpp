#include <smpl_urdf_robot_model/robot_state.h>

// project includes
#include <smpl_urdf_robot_model/robot_model.h>

namespace smpl {

static
Affine3 ComputeJointTransform(const Joint* joint, double* variables)
{
    switch (joint->type) {
    case JointType::Fixed:
        return Affine3::Identity();
    case JointType::Revolute:
        return Affine3(AngleAxis(variables[0], joint->axis));
    case JointType::Prismatic:
        return Affine3(Translation3(variables[0] * joint->axis));
    case JointType::Planar:
        return Translation3(variables[0], variables[1], 0.0) *
                AngleAxis(variables[1], Vector3::UnitZ());
    case JointType::Floating:
        return Translation3(variables[0], variables[1], variables[2]) *
                Quaternion(variables[6], variables[3], variables[4], variables[5]);
    default:
        assert(0);
    }
}

static
void GetTransformVariables(
    const Joint* joint,
    const Affine3* transform,
    double* variables)
{
    switch (joint->type) {
    case JointType::Fixed:
        return;
    case JointType::Revolute:
    {
        Quaternion q(transform->rotation());
        size_t idx;
        joint->axis.array().abs().maxCoeff(&idx);
        variables[0] = 2.0 * atan2(q.vec()[idx] / joint->axis[idx], q.w());
        return;
    }
    case JointType::Prismatic:
    {
        auto val = transform->translation().dot(joint->axis);
        variables[0] = val;
        return;
    }
    case JointType::Planar:
    {
        variables[0] = transform->translation().x();
        variables[1] = transform->translation().y();

        Quaternion q(transform->rotation());
        auto s_squared = 1.0 - (q.w() * q.w());
        if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
            variables[2] = 0.0;
        } else {
            auto s = 1.0 / sqrt(s_squared);
            variables[2] = (2.0 * acos(q.w())) * (q.z() * s);
        }
        return;
    }
    case JointType::Floating:
    {
        variables[0] = transform->translation().x();
        variables[1] = transform->translation().y();
        variables[2] = transform->translation().z();
        Quaternion q(transform->rotation());
        variables[3] = q.x();
        variables[4] = q.y();
        variables[5] = q.z();
        variables[6] = q.w();
        return;
    }
    default:
        assert(0);
        return;
    }
}

bool Init(
    RobotState* out,
    const RobotModel* model,
    bool with_velocities,
    bool with_accelerations)
{
    RobotState robot_state;

    robot_state.model = model;

    int mult = 1;
    if (with_velocities) {
        ++mult;
    }
    if (with_accelerations) {
        ++mult;
    }
    robot_state.values.resize(mult * GetVariableCount(model));

    double* data = robot_state.values.data();
    robot_state.positions = data;
    data += GetVariableCount(model);
    if (with_velocities) {
        robot_state.velocities = data;
        data += GetVariableCount(model);
    }
    if (with_accelerations) {
        robot_state.accelerations = data;
        data += GetVariableCount(model);
    }

    robot_state.transforms.resize(
            GetLinkCount(model) +
            GetJointCount(model) +
            GetCollisionBodyCount(model) +
            GetVisualBodyCount(model));

    robot_state.link_transforms = robot_state.transforms.data();
    robot_state.joint_transforms = robot_state.link_transforms + GetLinkCount(model);
    robot_state.link_collision_transforms = robot_state.joint_transforms + GetJointCount(model);
    robot_state.link_visual_transforms = robot_state.link_collision_transforms + GetCollisionBodyCount(model);

    *out = std::move(robot_state);
    return true;
}

auto GetRobotModel(const RobotState* state) -> const RobotModel*
{
    return state->model;
}

void SetToDefaultValues(RobotState* state)
{
    for (int i = 0; i < GetVariableCount(state->model); ++i) {
        state->positions[i] = GetDefaultPosition(
                state->model, GetVariable(state->model, i));
    }
    state->dirty = true;
}

void SetVariablePositions(RobotState* state, const double* positions)
{
    std::copy(positions, positions + GetVariableCount(state->model),
            state->positions);
    state->dirty = true;
}

void SetVariablePosition(
    RobotState* state,
    const JointVariable* variable,
    double position)
{
    return SetVariablePosition(
            state, GetVariableIndex(state->model, variable), position);
}

void SetVariablePosition(
    RobotState* state,
    int index,
    double position)
{
    state->positions[index] = position;
    state->dirty = true;
}

auto GetVariablePositions(const RobotState* state) -> const double*
{
    return state->positions;
}

auto GetVariablePosition(const RobotState* state, const JointVariable* variable)
    -> double
{
    return GetVariablePosition(state, GetVariableIndex(state->model, variable));
}

auto GetVariablePosition(const RobotState* state, int index) -> double
{
    return state->positions[index];
}

bool HasVariableVelocities(const RobotState* state)
{
    return state->velocities != NULL;
}

void SetVariableVelocities(RobotState* state, const double* velocities)
{
    std::copy(velocities, velocities + GetVariableCount(state->model),
            state->velocities);
}

void SetVariableVelocity(
    RobotState* state,
    const JointVariable* variable,
    double v)
{
    return SetVariableVelocity(
            state, GetVariableIndex(state->model, variable), v);
}

void SetVariableVelocity(RobotState* state, int index, double v)
{
    state->velocities[index] = v;
}

auto GetVariableVelocities(const RobotState* state) -> const double*
{
    return state->velocities;
}

auto GetVariableVelocity(const RobotState* state, const JointVariable* variable)
    -> double
{
    return GetVariableVelocity(state, GetVariableIndex(state->model, variable));
}

auto GetVariableVelocity(const RobotState* state, int index) -> double
{
    return state->velocities[index];
}

bool HasVariableAccelerations(const RobotState* state)
{
    return state->accelerations != NULL;
}

void SetVariableAccelerations(RobotState* state, const double* accelerations)
{
    std::copy(accelerations, accelerations + GetVariableCount(state->model),
            state->accelerations);
}

void SetVariableAcceleration(
    RobotState* state,
    const JointVariable* variable,
    double a)
{
    return SetVariableAcceleration(
            state, GetVariableIndex(state->model, variable), a);
}

void SetVariableAcceleration(RobotState* state, int index, double a)
{
    state->accelerations[index] = a;
}

auto GetVariableAccelerations(const RobotState* state) -> const double*
{
    return state->accelerations;
}

auto GetVariableAcceleration(
    const RobotState* state,
    const JointVariable* variable) -> double
{
    return GetVariableAcceleration(
            state, GetVariableIndex(state->model, variable));
}

auto GetVariableAcceleration(const RobotState* state, int index) -> double
{
    return state->accelerations[index];
}

void SetJointPositions(
    RobotState* state,
    const Joint* joint,
    const double* positions)
{
    std::copy(
            positions,
            positions + GetVariableCount(joint),
            state->positions + GetVariableIndex(state->model, GetFirstVariable(joint)));
    state->dirty = true;
}

void SetJointPositions(RobotState* state, int index, const double* positions)
{
    return SetJointPositions(state, GetJoint(state->model, index), positions);
}

void SetJointPositions(
    RobotState* state,
    const Joint* joint,
    const Affine3* transform)
{
    double positions[7];
    GetTransformVariables(joint, transform, positions);
    return SetJointPositions(state, joint, positions);
}

void SetJointPositions(RobotState* state, int index, const Affine3* transform)
{
    return SetJointPositions(state, GetJoint(state->model, index), transform);
}

void SetJointVelocities(
    RobotState* state,
    const Joint* joint,
    const double* velocities)
{
    std::copy(
            velocities,
            velocities + GetVariableCount(joint),
            state->velocities + GetVariableIndex(state->model, GetFirstVariable(joint)));
}

void SetJointVelocities(RobotState* state, int index, const double* velocities)
{
    return SetJointVelocities(state, GetJoint(state->model, index), velocities);
}

void SetJointAccelerations(
    RobotState* state,
    const Joint* joint,
    const double* accelerations)
{
    std::copy(
            accelerations,
            accelerations + GetVariableCount(joint),
            state->accelerations + GetVariableIndex(state->model, GetFirstVariable(joint)));
}

void SetJointAccelerations(
    RobotState* state,
    int index,
    const double* accelerations)
{
    return SetJointAccelerations(state, GetJoint(state->model, index), accelerations);
}

static
auto GetJointPositions(RobotState* state, const Joint* joint) -> double*
{
    return state->positions + GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointPositions(const RobotState* state, const Joint* joint)
    -> const double*
{
    return GetJointPositions(const_cast<RobotState*>(state), joint);
}

auto GetJointPositions(const RobotState* state, int index) -> const double*
{
    return GetJointPositions(state, GetJoint(state->model, index));
}

auto GetJointVelocities(const RobotState* state, const Joint* joint)
    -> const double*
{
    return state->velocities +
            GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointVelocities(const RobotState* state, int index) -> const double*
{
    return GetJointVelocities(state, GetJoint(state->model, index));
}

auto GetJointAccelerations(const RobotState* state, const Joint* joint)
    -> const double*
{
    return state->accelerations +
            GetVariableIndex(state->model, GetFirstVariable(joint));
}

auto GetJointAccelerations(const RobotState* state, int index) -> const double*
{
    return GetJointAccelerations(state, GetJoint(state->model, index));
}

/////////////////////////// DANGER ZONE ///////////////////////////

void UpdateTransforms(RobotState* state)
{
    if (!state->dirty) return;

    std::vector<const Joint*> joints;
    joints.push_back(GetRootJoint(state->model));

    while (!joints.empty()) {
        auto* joint = joints.back();
        joints.pop_back();

        // update the joint transform
        auto& joint_transform =
                state->joint_transforms[GetJointIndex(state->model, joint)];
        joint_transform =
                ComputeJointTransform(joint, GetJointPositions(state, joint));

        // update the child link transform
        auto& link_transform =
                state->link_transforms[GetLinkIndex(state->model, joint->child)];
        if (joint->parent != NULL) {
            // parent_link * origin * joint transform
            auto& parent_transform =
                    state->link_transforms[
                            GetLinkIndex(state->model, joint->parent)];
            link_transform = parent_transform * joint->origin * joint_transform;
        } else {
            link_transform = joint->origin * joint_transform;
        }

        // update all child collision and visual transforms
        for (auto& collision : joint->child->collision) {
            auto& collision_transform =
                    state->link_collision_transforms[
                            GetCollisionBodyIndex(state->model, &collision)];
            collision_transform = link_transform * collision.origin;
        }

        for (auto& visual : joint->child->visual) {
            auto& visual_transform =
                    state->link_visual_transforms[
                            GetVisualBodyIndex(state->model, &visual)];
            visual_transform = link_transform * visual.origin;
        }

        // recurse on children
        for (auto* child = joint->child->children; child != NULL; child = child->sibling) {
            joints.push_back(child);
        }
    }

    state->dirty = false;
}

void UpdateLinkTransforms(RobotState* state)
{
    UpdateTransforms(state);
}

void UpdateLinkTransform(RobotState* state, const Link* link)
{
    UpdateTransforms(state);
}

void UpdateLinkTransform(RobotState* state, int index)
{
    UpdateTransforms(state);
}

void UpdateCollisionBodyTransforms(RobotState* state)
{
    UpdateTransforms(state);
}

void UpdateCollisionBodyTransform(
    RobotState* state,
    const LinkCollision* collision)
{
    UpdateTransforms(state);
}

void UpdateCollisionBodyTransform(RobotState* state, int index)
{
    UpdateTransforms(state);
}

void UpdateVisualBodyTransforms(RobotState* state)
{
    UpdateTransforms(state);
}

void UpdateVisualBodyTransform(RobotState* state, const LinkVisual* visual)
{
    UpdateTransforms(state);
}

void UpdateVisualBodyTransform(RobotState* state, int index)
{
    UpdateTransforms(state);
}

auto GetLinkTransform(const RobotState* state, const Link* link)
    -> const Affine3*
{
    return GetLinkTransform(state, GetLinkIndex(state->model, link));
}

auto GetLinkTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_transforms[index];
}

auto GetCollisionBodyTransform(
    const RobotState* state,
    const LinkCollision* collision)
    -> const Affine3*
{
    return GetCollisionBodyTransform(
            state, GetCollisionBodyIndex(state->model, collision));
}

auto GetCollisionBodyTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_collision_transforms[index];
}

auto GetVisualBodyTransform(
    const RobotState* state,
    const LinkVisual* visual)
    -> const Affine3*
{
    return GetVisualBodyTransform(
            state, GetVisualBodyIndex(state->model, visual));
}

auto GetVisualBodyTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->link_visual_transforms[index];
}

auto GetJointTransform(const RobotState* state, const Joint* joint)
    -> const Affine3*
{
    return GetJointTransform(state, GetJointIndex(state->model, joint));
}

auto GetJointTransform(const RobotState* state, int index) -> const Affine3*
{
    return &state->joint_transforms[index];
}

bool IsJointTransformDirty(const RobotState* state, const Joint* joint)
{
    return state->dirty;
}

bool IsLinkTransformDirty(const RobotState* state, const Link* link)
{
    return state->dirty;
}

bool IsCollisionBodyTransformDirty(const RobotState* state, const Link* link, int index)
{
    return state->dirty;
}

bool IsVisualBodyTransformDirty(const RobotState* state, const Link* link, int index)
{
    return state->dirty;
}

bool IsDirty(const RobotState* state)
{
    return state->dirty;
}

bool SatisfiesBounds(const RobotState* state)
{
    return false;
}

bool SatisfiesBounds(const RobotState* state, const Joint* joint)
{
    return false;
}

bool SatisfiesBounds(const RobotState* state, const JointVariable* variable)
{
    return false;
}

} // namespace smpl

