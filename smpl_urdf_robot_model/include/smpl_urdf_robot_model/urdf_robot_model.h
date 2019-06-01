#ifndef SMPL_URDF_ROBOT_MODEL_URDF_ROBOT_MODEL_H
#define SMPL_URDF_ROBOT_MODEL_URDF_ROBOT_MODEL_H

// system includes
#include <smpl/robot_model.h>

// project includes
#include <smpl_urdf_robot_model/robot_state.h>

namespace smpl {
namespace urdf {

struct RobotModel;

struct URDFRobotModel :
    public virtual smpl::RobotModel,
    public virtual smpl::IForwardKinematics
{
    struct VariableProperties
    {
        double min_position;
        double max_position;
        double vel_limit;
        double acc_limit;
        bool continuous;
        bool bounded;
    };

    const ::smpl::urdf::RobotModel* robot_model = NULL;

    // persistent robot state to cache unchanging transforms
    RobotState robot_state;

    std::vector<VariableProperties> vprops;
    std::vector<int> planning_to_state_variable;
    const Link* planning_link = NULL;

    auto ComputeFK(const smpl::RobotState& state)
        -> Eigen::Affine3d override;

    double MinPosLimit(int jidx) const override;
    double MaxPosLimit(int jidx) const override;
    bool HasPosLimit(int jidx) const override;
    bool IsContinuous(int jidx) const override;
    double VelLimit(int jidx) const override;
    double AccLimit(int jidx) const override;
    bool CheckJointLimits(
        const smpl::RobotState& state,
        bool verbose = false) override;

    auto GetVisualization(const smpl::RobotState& state)
        -> std::vector<visual::Marker> final;

    auto GetExtension(size_t class_code) -> smpl::Extension* final;
};

bool Init(
    URDFRobotModel* urdf_model,
    const RobotModel* robot_model,
    const std::vector<std::string>* planning_joint_names);
bool Init(
    URDFRobotModel* urdf_model,
    const RobotModel* robot_model,
    const std::vector<const Joint*>* planning_joints);

bool SetPlanningLink(URDFRobotModel* urdf_model, const char* link_name);
bool SetPlanningLink(URDFRobotModel* urdf_model, const std::string* link_name);
bool SetPlanningLink(URDFRobotModel* urdf_model, const Link* link);

void SetReferenceState(URDFRobotModel* urdf_model, const double* positions);

} // namespace urdf
} // namespace smpl

#endif

