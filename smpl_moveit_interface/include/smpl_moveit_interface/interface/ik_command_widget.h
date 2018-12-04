#ifndef SMPL_MOVEIT_INTERFACE_IK_COMMAND_WIDGET_H
#define SMPL_MOVEIT_INTERFACE_IK_COMMAND_WIDGET_H

// standard includes
#include <string>

// system includes
#include <QWidget>

class QDoubleSpinBox;

namespace sbpl_interface {

class RobotCommandModel;

class IKCommandWidget : public QWidget
{
    Q_OBJECT

public:

    IKCommandWidget(RobotCommandModel* model, QWidget* parent = 0);

public Q_SLOTS:

    void setActiveJointGroup(const std::string& group_name);

public:

    RobotCommandModel* m_model = 0;

    std::string m_active_group_name;

    QDoubleSpinBox* m_spinbox_x = 0;
    QDoubleSpinBox* m_spinbox_y = 0;
    QDoubleSpinBox* m_spinbox_z = 0;
    QDoubleSpinBox* m_spinbox_rz = 0;
    QDoubleSpinBox* m_spinbox_ry = 0;
    QDoubleSpinBox* m_spinbox_rx = 0;

private Q_SLOTS:

    void updateRobotModel();
    void updateRobotState();

    void updatePositionX(double value);
    void updatePositionY(double value);
    void updatePositionZ(double value);

    void updatePositionRX(double value);
    void updatePositionRY(double value);
    void updatePositionRZ(double value);
};

} // namespace sbpl_interface

#endif
