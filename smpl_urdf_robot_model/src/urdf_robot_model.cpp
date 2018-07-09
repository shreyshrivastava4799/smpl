#include <smpl_urdf_robot_model/urdf_robot_model.h>

namespace smpl {

bool Init(
    URDFRobotModel* out,
    const RobotModel* robot_model,
    const std::vector<std::string>* planning_joint_names)
{
    URDFRobotModel urdf_model;
    std::vector<std::string> planning_variables;

    for (auto& joint_name : *planning_joint_names) {
        auto* joint = GetJoint(robot_model, &joint_name);
        if (joint == NULL) return false;
        for (auto& variable : Variables(joint)) {
            planning_variables.push_back(*GetName(&variable));
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
            urdf_model.vprops.push_back(props);

            auto index = GetVariableIndex(robot_model, &variable);
            urdf_model.planning_to_state_variable.push_back(index);
        }
    }

    urdf_model.setPlanningJoints(planning_variables);
    urdf_model.robot_model = robot_model;
    if (!Init(&urdf_model.robot_state, robot_model)) return false;

    *out = std::move(urdf_model);
    return true;
}

bool SetPlanningLink(URDFRobotModel* urdf_model, const std::string* link_name)
{
    auto* link = GetLink(urdf_model->robot_model, link_name);
    if (link == NULL) return false;
    urdf_model->planning_link = link;
    return true;
}

static
void UpdateState(URDFRobotModel* model, const sbpl::motion::RobotState* state)
{
    for (auto i = 0; i < model->jointVariableCount(); ++i) {
        SetVariablePosition(&model->robot_state, model->planning_to_state_variable[i], (*state)[i]);
    }
}

auto URDFRobotModel::computeFK(const sbpl::motion::RobotState& state)
    -> Eigen::Affine3d
{
    UpdateState(this, &state);
    UpdateLinkTransform(&this->robot_state, this->planning_link);
    return *GetLinkTransform(&this->robot_state, this->planning_link);
}

double URDFRobotModel::minPosLimit(int jidx) const
{
    return this->vprops[jidx].min_position;
}

double URDFRobotModel::maxPosLimit(int jidx) const
{
    return this->vprops[jidx].max_position;
}

bool URDFRobotModel::hasPosLimit(int jidx) const
{
    return this->vprops[jidx].bounded;
}

bool URDFRobotModel::isContinuous(int jidx) const
{
    return this->vprops[jidx].continuous;
}

double URDFRobotModel::velLimit(int jidx) const
{
    return this->vprops[jidx].vel_limit;
}

double URDFRobotModel::accLimit(int jidx) const
{
    return this->vprops[jidx].acc_limit;
}

bool URDFRobotModel::checkJointLimits(
    const sbpl::motion::RobotState& state,
    bool verbose)
{
    return false;
}

auto URDFRobotModel::getExtension(size_t class_code) -> sbpl::motion::Extension*
{
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::RobotModel>()) return this;
    if (class_code == sbpl::motion::GetClassCode<sbpl::motion::ForwardKinematicsInterface>()) return this;
    return NULL;
}

} // namespace smpl

