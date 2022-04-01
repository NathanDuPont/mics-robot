#pragma once

#include <algorithm>
#include <initializer_list>
#include "lib-rr/nodes/NodeManager.h"
#include "api.h"
#include "lib-rr/nodes/subsystems/IDriveNode.h"
#include "lib-rr/nodes/actuator_nodes/MotorNode.h"
#include "lib-rr/nodes/sensor_nodes/ControllerNode.h"
#include "lib-rr/nodes/sensor_nodes/InertialSensorNode.h"
#include "lib-rr/kinematics/HolonomicDriveKinematics.h"
#include "lib-rr/eigen/Eigen/Dense"

class HolonomicDriveNode : public IDriveNode {
public: 
    struct HolonomicFourMotors {
        MotorNode* left_front_motor;
        MotorNode* left_rear_motor;
        MotorNode* right_front_motor;
        MotorNode* right_rear_motor;
    };

    HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        InertialSensorNode* inertial_sensor, HolonomicFourMotors motors, HolonomicDriveKinematics kinematics);

    void initialize();

    void resetEncoders();

    IDriveNode::FourMotorDriveEncoderVals getIntegratedEncoderVals();

    void setDriveVoltage(int x_voltage, int theta_voltage);

    void setDriveVoltage(int x_voltage, int y_voltage, int theta_voltage);

    void setDriveVoltage(int left_x, int left_y, int right_x, int right_y);

    void setDriveVelocity(float x_velocity, float theta_velocity);

    void setDriveVelocity(float x_velocity, float y_velocity, float theta_velocity);

    void teleopPeriodic();

    void autonPeriodic();

    ~HolonomicDriveNode();

private:
    pros::Controller* m_controller;

    InertialSensorNode* m_inertial_sensor;

    std::string m_handle_name;

    HolonomicFourMotors m_motors;

    HolonomicDriveKinematics m_kinematics;

    Eigen::Vector2d controller_target_velocity;
    Eigen::Vector2d field_target_velocity;
    double rotation_velocity;

    void m_setLeftFrontVoltage(int voltage);

    void m_setLeftRearVoltage(int voltage);

    void m_setRightFrontVoltage(int voltage);

    void m_setRightRearVoltage(int voltage);

    void m_setLeftFrontVelocity(float velocity);

    void m_setLeftRearVelocity(float velocity);

    void m_setRightFrontVelocity(float velocity);

    void m_setRightRearVelocity(float velocity);

    void m_fieldOrientedControl();

    void m_tankControl();
};
