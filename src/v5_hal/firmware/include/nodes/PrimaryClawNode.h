#pragma once

#include "lib-rr/nodes/NodeManager.h"
#include "lib-rr/nodes/subsystems/IClawNode.h"
#include "lib-rr/nodes/sensor_nodes/ControllerNode.h"
#include "lib-rr/nodes/actuator_nodes/ADIDigitalOutNode.h"
#include "pros/misc.h"

class PrimaryClawNode : public IClawNode {
private:
    bool front_claw_open = false;
    bool l1_previous_state = false;
    ControllerNode* m_controller;
    ADIDigitalOutNode* m_primary_claw;
    
public:
    PrimaryClawNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, ADIDigitalOutNode* primary_claw);

    void initialize();

    void useClaw(bool opened);

    void teleopPeriodic();

    void autonPeriodic();

    ~PrimaryClawNode();
};