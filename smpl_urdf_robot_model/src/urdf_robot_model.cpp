#include <smpl_urdf_robot_model/urdf_robot_model.h>

// project includes
#include <smpl_urdf_robot_model/robot_state_bounds.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h>

namespace smpl {
namespace urdf {

bool Init(
    URDFRobotModel* model,
    const RobotModel* robot_model,
    const std::vector<std::string>* planning_joint_names)
{
    URDFRobotModel urdf_model;
    auto planning_variables = std::vector<std::string>();

    for (auto& joint_name : *planning_joint_names) {
        auto* joint = GetJoint(robot_model, &joint_name);
        if (joint == NULL) {
            return false;
        }
        for (auto& variable : Variables(joint)) {
            planning_variables.push_back(*GetName(&variable));

            // cache variable limits
            auto* limits = GetVariableLimits(&variable);
            URDFRobotModel::VariableProperties props;
            props.bounded = limits->has_position_limits;

            if (((joint->type == JointType::Revolute) & (!limits->has_position_limits)) |
                ((joint->type == JointType::Planar) & (&variable == joint->vfirst + 2)))
            {
                props.continuous = true;
            } else {
                props.continuous = false;
            }

            props.min_position = limits->min_position;
            props.max_position = limits->max_position;
            props.vel_limit = limits->max_velocity;
            props.acc_limit = limits->max_effort; // TODO: hmm...
            model->vprops.push_back(props);

            // initialize mapping from planning group variable to state variable
            auto index = GetVariableIndex(robot_model, &variable);
            model->planning_to_state_variable.push_back(index);
        }
    }

    model->SetPlanningJoints(planning_variables);
    model->robot_model = robot_model;

    if (!InitRobotState(&model->robot_state, robot_model)) {
        return false;
    }

    return true;
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const char* link_name)
{
    auto* link = GetLink(urdf_model->robot_model, link_name);
    if (link == NULL) return false;
    return SetPlanningLink(urdf_model, link);
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const std::string* link_name)
{
    return SetPlanningLink(urdf_model, link_name->c_str());
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const Link* link)
{
    urdf_model->planning_link = link;
    return true;
}

void SetReferenceState(URDFRobotModel* model, const double* positions)
{
    SetVariablePositions(&model->robot_state, positions);
}

static
void UpdateState(URDFRobotModel* model, const smpl::RobotState* state)
{
    for (auto i = 0; i < model->JointVariableCount(); ++i) {
        SetVariablePosition(
                &model->robot_state,
                model->planning_to_state_variable[i],
                (*state)[i]);
    }
}

auto URDFRobotModel::ComputeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    UpdateState(this, &state);
    return *GetUpdatedLinkTransform(&this->robot_state, this->planning_link);
}

double URDFRobotModel::MinPosLimit(int jidx) const
{
    return this->vprops[jidx].min_position;
}

double URDFRobotModel::MaxPosLimit(int jidx) const
{
    return this->vprops[jidx].max_position;
}

bool URDFRobotModel::HasPosLimit(int jidx) const
{
    return this->vprops[jidx].bounded;
}

bool URDFRobotModel::IsContinuous(int jidx) const
{
    return this->vprops[jidx].continuous;
}

double URDFRobotModel::VelLimit(int jidx) const
{
    return this->vprops[jidx].vel_limit;
}

double URDFRobotModel::AccLimit(int jidx) const
{
    return this->vprops[jidx].acc_limit;
}

bool URDFRobotModel::CheckJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    // TODO: Add a SatisfiesBounds overload that accepts a value for the
    // variable instead of looking up the value within a RobotState.
    UpdateState(this, &state);
    for (auto i = 0; i < this->JointVariableCount(); ++i) {
        auto* var = GetVariable(this->robot_model, this->planning_to_state_variable[i]);
        if (!SatisfiesBounds(&this->robot_state, var)) {
            return false;
        }
    }
    return true;
}

auto URDFRobotModel::GetVisualization(const smpl::RobotState& state)
    -> std::vector<visual::Marker>
{
    UpdateState(this, &state);
    UpdateVisualBodyTransforms(&this->robot_state);
    return MakeRobotVisualization(
            &this->robot_state,
            smpl::visual::Color{ 0.5f, 0.5f, 0.5f, 1.0f });

}

auto URDFRobotModel::GetExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::RobotModel>()) return this;
    if (class_code == smpl::GetClassCode<smpl::IForwardKinematics>()) return this;
    return NULL;
}

} // namespace urdf
} // namespace smpl

