#include <smpl_moveit_interface/interface/ik_command_widget.h>

// system includes
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>

// project includes
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <smpl_moveit_interface/interface/robot_command_model.h>
#include <smpl_moveit_interface/interface/utils.h>

namespace sbpl_interface {

IKCommandWidget::IKCommandWidget(
    RobotCommandModel* model,
    QWidget* parent)
:
    QWidget(parent)
{
    ROS_INFO("Construct IKCommandWidget");
    connect(model, SIGNAL(robotLoaded()), this, SLOT(updateRobotModel()));
    connect(model, SIGNAL(robotStateChanged()), this, SLOT(updateRobotState()));

    m_model = model;

    m_spinbox_x = new QDoubleSpinBox;
    m_spinbox_y = new QDoubleSpinBox;
    m_spinbox_z = new QDoubleSpinBox;

    m_spinbox_x->setMinimum(-100.0);
    m_spinbox_x->setMaximum(100.0);
    m_spinbox_x->setSingleStep(0.01);
    m_spinbox_x->setDecimals(3);
    m_spinbox_x->setSuffix("m");

    m_spinbox_y->setMinimum(-100.0);
    m_spinbox_y->setMaximum(100.0);
    m_spinbox_y->setSingleStep(0.01);
    m_spinbox_y->setDecimals(3);
    m_spinbox_y->setSuffix("m");

    m_spinbox_z->setMinimum(-100.0);
    m_spinbox_z->setMaximum(100.0);
    m_spinbox_z->setSingleStep(0.01);
    m_spinbox_z->setDecimals(3);
    m_spinbox_z->setSuffix("m");

    m_spinbox_rz = new QDoubleSpinBox;
    m_spinbox_rz->setMinimum(-180.0);
    m_spinbox_rz->setMaximum(180.0);
    m_spinbox_rz->setSingleStep(1.0);
    m_spinbox_rz->setWrapping(true);
    m_spinbox_rz->setSuffix(QChar(0260));

    m_spinbox_ry = new QDoubleSpinBox;
    m_spinbox_ry->setMinimum(0.0);
    m_spinbox_ry->setMaximum(180.0);
    m_spinbox_ry->setSingleStep(1.0);
    m_spinbox_ry->setWrapping(true);
    m_spinbox_ry->setSuffix(QChar(0260));

    m_spinbox_rx = new QDoubleSpinBox;
    m_spinbox_rx->setMinimum(-180.0);
    m_spinbox_rx->setMaximum(180.0);
    m_spinbox_rx->setSingleStep(1.0);
    m_spinbox_rx->setWrapping(true);
    m_spinbox_rx->setSuffix(QChar(0260));

    auto* layout = new QGridLayout;
    layout->addWidget(new QLabel(tr("T(x,y,z):")));
    layout->addWidget(m_spinbox_x, 0, 1);
    layout->addWidget(m_spinbox_y, 0, 2);
    layout->addWidget(m_spinbox_z, 0, 3);
    layout->addWidget(new QLabel(tr("R(z,y,x):")));
    layout->addWidget(m_spinbox_rz, 1, 1);
    layout->addWidget(m_spinbox_ry, 1, 2);
    layout->addWidget(m_spinbox_rx, 1, 3);

    this->setLayout(layout);

    connect(m_spinbox_x, SIGNAL(valueChanged(double)), this, SLOT(updatePositionX(double)));
    connect(m_spinbox_y, SIGNAL(valueChanged(double)), this, SLOT(updatePositionY(double)));
    connect(m_spinbox_z, SIGNAL(valueChanged(double)), this, SLOT(updatePositionZ(double)));
    connect(m_spinbox_rx, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRX(double)));
    connect(m_spinbox_ry, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRY(double)));
    connect(m_spinbox_rz, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRZ(double)));
}

void IKCommandWidget::updateRobotModel()
{
    ROS_INFO("Update robot model");
}

void IKCommandWidget::updateRobotState()
{
    ROS_INFO("Update robot state");

    if (m_active_group_name.empty()) return;

    auto& robot_model = m_model->getRobotModel();
    auto* robot_state = m_model->getRobotState();
    if (robot_model == NULL | robot_state == NULL) return;

    auto* joint_group = robot_model->getJointModelGroup(m_active_group_name);
    if (joint_group == NULL) return;

    auto tips = GetTipLinks(*joint_group);
    if (tips.empty()) return;

    auto tip = tips[0];

    auto fk_pose = robot_state->getGlobalLinkTransform(tip);

    ROS_INFO("Set spinbox values to (%f, %f, %f)",
            fk_pose.translation().x(),
            fk_pose.translation().y(),
            fk_pose.translation().z());

    disconnect(m_spinbox_x, SIGNAL(valueChanged(double)), this, SLOT(updatePositionX(double)));
    disconnect(m_spinbox_y, SIGNAL(valueChanged(double)), this, SLOT(updatePositionY(double)));
    disconnect(m_spinbox_z, SIGNAL(valueChanged(double)), this, SLOT(updatePositionZ(double)));
    disconnect(m_spinbox_rx, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRX(double)));
    disconnect(m_spinbox_ry, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRY(double)));
    disconnect(m_spinbox_rz, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRZ(double)));

    m_spinbox_x->setValue(fk_pose.translation().x());
    m_spinbox_y->setValue(fk_pose.translation().y());
    m_spinbox_z->setValue(fk_pose.translation().z());

    double yaw, pitch, roll;
    smpl::get_euler_zyx(fk_pose.rotation(), yaw, pitch, roll);
    m_spinbox_rx->setValue(smpl::to_degrees(roll));
    m_spinbox_ry->setValue(smpl::to_degrees(pitch));
    m_spinbox_rz->setValue(smpl::to_degrees(yaw));

    connect(m_spinbox_x, SIGNAL(valueChanged(double)), this, SLOT(updatePositionX(double)));
    connect(m_spinbox_y, SIGNAL(valueChanged(double)), this, SLOT(updatePositionY(double)));
    connect(m_spinbox_z, SIGNAL(valueChanged(double)), this, SLOT(updatePositionZ(double)));
    connect(m_spinbox_rx, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRX(double)));
    connect(m_spinbox_ry, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRY(double)));
    connect(m_spinbox_rz, SIGNAL(valueChanged(double)), this, SLOT(updatePositionRZ(double)));
}

void IKCommandWidget::setActiveJointGroup(const std::string& group_name)
{
    ROS_INFO("Set active joint group to '%s'", group_name.c_str());
    m_active_group_name = group_name;
}

template <class UpdateFn>
void UpdateIKPose(IKCommandWidget* widget, UpdateFn update)
{
    ROS_INFO("Update position z");
    auto& robot_model = widget->m_model->getRobotModel();
    auto* robot_state = widget->m_model->getRobotState();
    if (robot_model == NULL | robot_state == NULL) return;

    auto* joint_group = robot_model->getJointModelGroup(widget->m_active_group_name);
    if (joint_group == NULL) return;

    auto tips = GetTipLinks(*joint_group);
    if (tips.empty()) return;

    auto tip = tips[0];

    auto ik_pose = robot_state->getGlobalLinkTransform(tip);
    if (!update(&ik_pose)) {
        return;
    }

    auto ik_state = *robot_state;

    if (ik_state.setFromIK(joint_group, ik_pose)) {
        ROS_INFO("IK Succeeded");
        widget->m_model->setVariablePositions(ik_state.getVariablePositions());
    } else {
        ROS_WARN("IK failed");
    }
}

void IKCommandWidget::updatePositionX(double value)
{
    ROS_INFO("Update position x");
    auto update = [value](Eigen::Affine3d* pose)
    {
        if (pose->translation().x() == value) return false;
        pose->translation().x() = value;
        return true;
    };

    UpdateIKPose(this, update);
}

void IKCommandWidget::updatePositionY(double value)
{
    ROS_INFO("Update position y");
    auto update = [value](Eigen::Affine3d* pose)
    {
        if (pose->translation().y() == value) return false;
        pose->translation().y() = value;
        return true;
    };

    UpdateIKPose(this, update);
}

void IKCommandWidget::updatePositionZ(double value)
{
    ROS_INFO("Update position z");
    auto update = [value](Eigen::Affine3d* pose)
    {
        if (pose->translation().z() == value) return false;
        pose->translation().z() = value;
        return true;
    };

    UpdateIKPose(this, update);
}

void IKCommandWidget::updatePositionRX(double value)
{
    ROS_INFO("Update roll");
    auto update = [value](Eigen::Affine3d* pose)
    {
        double yaw, pitch, roll;
        smpl::get_euler_zyx(pose->rotation(), yaw, pitch, roll);

        if (roll == smpl::to_radians(value)) return false;
        *pose = smpl::MakeAffine(pose->translation().x(), pose->translation().y(), pose->translation().z(), yaw, pitch, smpl::to_radians(value));
        return true;
    };
    UpdateIKPose(this, update);
}

void IKCommandWidget::updatePositionRY(double value)
{
    ROS_INFO("Update roll");
    auto update = [value](Eigen::Affine3d* pose)
    {
        double yaw, pitch, roll;
        smpl::get_euler_zyx(pose->rotation(), yaw, pitch, roll);

        if (pitch == smpl::to_radians(value)) return false;
        *pose = smpl::MakeAffine(pose->translation().x(), pose->translation().y(), pose->translation().z(), yaw, smpl::to_radians(value), roll);
        return true;
    };
    UpdateIKPose(this, update);
}

void IKCommandWidget::updatePositionRZ(double value)
{
    ROS_INFO("Update roll");
    auto update = [value](Eigen::Affine3d* pose)
    {
        double yaw, pitch, roll;
        smpl::get_euler_zyx(pose->rotation(), yaw, pitch, roll);

        if (yaw == smpl::to_radians(value)) return false;
        *pose = smpl::MakeAffine(pose->translation().x(), pose->translation().y(), pose->translation().z(), smpl::to_radians(value), pitch, roll);
        return true;
    };
    UpdateIKPose(this, update);
}

} // namespace sbpl_interface
