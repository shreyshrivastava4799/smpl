#include <smpl_moveit_interface/interface/ik_command_widget.h>

// system includes
#include <QDoubleSpinBox>
#include <QGridLayout>

// project includes
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
    m_spinbox_ry = new QDoubleSpinBox;
    m_spinbox_rx = new QDoubleSpinBox;

    auto* layout = new QGridLayout;
    layout->addWidget(m_spinbox_x, 0, 0);
    layout->addWidget(m_spinbox_y, 0, 1);
    layout->addWidget(m_spinbox_z, 0, 2);
    layout->addWidget(m_spinbox_rz, 1, 0);
    layout->addWidget(m_spinbox_ry, 1, 1);
    layout->addWidget(m_spinbox_rx, 1, 2);

    this->setLayout(layout);

    connect(m_spinbox_x, SIGNAL(valueChanged(double)), this, SLOT(updatePositionX(double)));
    connect(m_spinbox_y, SIGNAL(valueChanged(double)), this, SLOT(updatePositionY(double)));
    connect(m_spinbox_z, SIGNAL(valueChanged(double)), this, SLOT(updatePositionZ(double)));
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
    m_spinbox_x->setValue(fk_pose.translation().x());
    m_spinbox_y->setValue(fk_pose.translation().y());
    m_spinbox_z->setValue(fk_pose.translation().z());
}

void IKCommandWidget::setActiveJointGroup(const std::string& group_name)
{
    ROS_INFO("Set active joint group to '%s'", group_name.c_str());
    m_active_group_name = group_name;
}

void IKCommandWidget::updatePositionX(double value)
{
    ROS_INFO("Update position x");
    auto& robot_model = m_model->getRobotModel();
    auto* robot_state = m_model->getRobotState();
    if (robot_model == NULL | robot_state == NULL) return;

    auto* joint_group = robot_model->getJointModelGroup(m_active_group_name);
    if (joint_group == NULL) return;

    auto tips = GetTipLinks(*joint_group);
    if (tips.empty()) return;

    auto tip = tips[0];

    auto ik_pose = robot_state->getGlobalLinkTransform(tip);
    if (ik_pose.translation().x() == value) return;
    ik_pose.translation().x() = value;

    auto ik_state = *robot_state;

    if (ik_state.setFromIK(joint_group, ik_pose)) {
        ROS_INFO("IK Succeeded");
        m_model->setVariablePositions(ik_state.getVariablePositions());
    } else {
        ROS_WARN("IK failed");
    }
}

void IKCommandWidget::updatePositionY(double value)
{
    ROS_INFO("Update position y");
    auto& robot_model = m_model->getRobotModel();
    auto* robot_state = m_model->getRobotState();
    if (robot_model == NULL | robot_state == NULL) return;

    auto* joint_group = robot_model->getJointModelGroup(m_active_group_name);
    if (joint_group == NULL) return;

    auto tips = GetTipLinks(*joint_group);
    if (tips.empty()) return;

    auto tip = tips[0];

    auto ik_pose = robot_state->getGlobalLinkTransform(tip);
    if (ik_pose.translation().y() == value) return;
    ik_pose.translation().y() = value;

    auto ik_state = *robot_state;

    if (ik_state.setFromIK(joint_group, ik_pose)) {
        ROS_INFO("IK Succeeded");
        m_model->setVariablePositions(ik_state.getVariablePositions());
    } else {
        ROS_WARN("IK failed");
    }
}

void IKCommandWidget::updatePositionZ(double value)
{
    ROS_INFO("Update position z");
    auto& robot_model = m_model->getRobotModel();
    auto* robot_state = m_model->getRobotState();
    if (robot_model == NULL | robot_state == NULL) return;

    auto* joint_group = robot_model->getJointModelGroup(m_active_group_name);
    if (joint_group == NULL) return;

    auto tips = GetTipLinks(*joint_group);
    if (tips.empty()) return;

    auto tip = tips[0];

    auto ik_pose = robot_state->getGlobalLinkTransform(tip);
    if (ik_pose.translation().z() == value) return;
    ik_pose.translation().z() = value;

    auto ik_state = *robot_state;

    if (ik_state.setFromIK(joint_group, ik_pose)) {
        ROS_INFO("IK Succeeded");
        m_model->setVariablePositions(ik_state.getVariablePositions());
    } else {
        ROS_WARN("IK failed");
    }
}

} // namespace sbpl_interface
