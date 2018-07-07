#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_H

#include <smpl_urdf_robot_model/types.h>

namespace smpl {

struct RobotModel;
struct Joint;
struct Link;
struct JointVariable;
struct LinkCollision;
struct LinkVisual;

struct RobotState;

bool Init(
    RobotState* state,
    const RobotModel* model,
    bool with_velocities = false,
    bool with_accelerations = false);

auto GetRobotModel(const RobotState* state) -> const RobotModel*;

///////////////
// Positions //
///////////////

void SetToDefaultValues(RobotState* state);

void SetVariablePositions(RobotState* state, const double* positions);
void SetVariablePosition(RobotState* state, const JointVariable* variable, double position);
void SetVariablePosition(RobotState* state, int index, double position);

auto GetVariablePositions(const RobotState* state) -> const double*;
auto GetVariablePosition(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariablePosition(const RobotState* state, int index) -> double;

////////////////
// Velocities //
////////////////

bool HasVariableVelocities(const RobotState* state);

void SetVariableVelocities(RobotState* state, const double* velocities);
void SetVariableVelocity(RobotState* state, const JointVariable* variable, double v);
void SetVariableVelocity(RobotState* state, int index, double v);

auto GetVariableVelocities(const RobotState* state) -> const double*;
auto GetVariableVelocity(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariableVelocity(const RobotState* state, int index) -> double;

///////////////////
// Accelerations //
///////////////////

bool HasVariableAccelerations(const RobotState* state);

void SetVariableAccelerations(RobotState* state, const double* accelerations);
void SetVariableAcceleration(RobotState* state, const JointVariable* variable, double a);
void SetVariableAcceleration(RobotState* state, int index, double a);

auto GetVariableAccelerations(const RobotState* state) -> const double*;
auto GetVariableAcceleration(const RobotState* state, const JointVariable* variable) -> double;
auto GetVariableAcceleration(const RobotState* state, int index) -> double;

////////////
// Joints //
////////////

void SetJointPositions(RobotState* state, const Joint* joint, const double* positions);
void SetJointPositions(RobotState* state, int index, const double* positions);

void SetJointPositions(RobotState* state, const Joint* joint, const Affine3* transform);
void SetJointPositions(RobotState* state, int index, const Affine3* transform);

void SetJointVelocities(RobotState* state, const Joint* joint, const double* velocities);
void SetJointVelocities(RobotState* state, int index, const double* velocities);

void SetJointAccelerations(RobotState* state, const Joint* joint, const double* accelerations);
void SetJointAccelerations(RobotState* state, int index, const double* accelerations);

auto GetJointPositions(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointPositions(const RobotState* state, int index) -> const double*;

auto GetJointVelocities(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointVelocities(const RobotState* state, int index) -> const double*;

auto GetJointAccelerations(const RobotState* state, const Joint* joint) -> const double*;
auto GetJointAccelerations(const RobotState* state, int index) -> const double*;

////////////////
// Transforms //
////////////////

void UpdateTransforms(RobotState* state);

void UpdateLinkTransforms(RobotState* state);
void UpdateLinkTransform(RobotState* state, const Link* link);
void UpdateLinkTransform(RobotState* state, int index);

void UpdateCollisionBodyTransforms(RobotState* state);
void UpdateCollisionBodyTransform(RobotState* state, const LinkCollision* collision);
void UpdateCollisionBodyTransform(RobotState* state, int index);

void UpdateVisualBodyTransforms(RobotState* state);
void UpdateVisualBodyTransform(RobotState* state, const LinkVisual* visual);
void UpdateVisualBodyTransform(RobotState* state, int index);

auto GetLinkTransform(const RobotState* state, const Link* link) -> const Affine3*;
auto GetLinkTransform(const RobotState* state, int index) -> const Affine3*;
auto GetCollisionBodyTransform(const RobotState* state, const LinkCollision* collision) -> const Affine3*;
auto GetCollisionBodyTransform(const RobotState* state, int index) -> const Affine3*;
auto GetVisualBodyTransform(const RobotState* state, const LinkVisual* visual) -> const Affine3*;
auto GetVisualBodyTransform(const RobotState* state, int index) -> const Affine3*;

auto GetJointTransform(const RobotState* state, const Joint* joint) -> const Affine3*;
auto GetJointTransform(const RobotState* state, int joint) -> const Affine3*;

bool IsJointTransformDirty(const RobotState* state, const Joint* joint);
bool IsLinkTransformDirty(const RobotState* state, const Link* link);
bool IsCollisionBodyTransformDirty(const RobotState* state, const Link* link, int index);
bool IsVisualBodyTransformDirty(const RobotState* state, const Link* link, int index);
bool IsDirty(const RobotState* state);

/////////////////////
// Bounds checking //
/////////////////////

bool SatisfiesBounds(const RobotState* state);
bool SatisfiesBounds(const RobotState* state, const Joint* joint);
bool SatisfiesBounds(const RobotState* state, const JointVariable* variable);

///////////////////
// Visualization //
///////////////////

// TODO: additional options: color, namespace, lifetime, frame
#if 0
void MakeRobotVisualization(const RobotState* state);
void MakeRobotVisualization(const RobotState* state, const Link* link);
#endif

////////////////
// Definition //
////////////////

struct RobotState
{
    const RobotModel*       model = NULL;

    std::vector<double>     values;
    double*                 positions = NULL;
    double*                 velocities = NULL;
    double*                 accelerations = NULL;

    std::vector<Affine3>    transforms;
    Affine3*                link_transforms = NULL;
    Affine3*                joint_transforms = NULL;
    Affine3*                link_collision_transforms = NULL;
    Affine3*                link_visual_transforms = NULL;
#if 0
    const Joint*            dirty_joint = NULL;
#else
    bool                    dirty = true;
#endif
};

} // namespace smpl

#endif

