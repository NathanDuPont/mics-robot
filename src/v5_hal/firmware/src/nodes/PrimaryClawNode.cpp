#include "nodes/PrimaryClawNode.h"

PrimaryClawNode::PrimaryClawNode(NodeManager* node_manager, std::string handle_name, 
		ControllerNode* controller, ADIDigitalOutNode* primary_claw) : 
		IClawNode(node_manager, handle_name), 
		m_controller(controller),
		m_primary_claw(primary_claw) {

}

void PrimaryClawNode::initialize() {

}

void PrimaryClawNode::useClaw(bool opened) {
	if (opened) {
		m_primary_claw->setValue(0);
	} else {
        m_primary_claw->setValue(1);
	}
}
void PrimaryClawNode::teleopPeriodic() {
	bool l1_current_state = m_controller->getController()->get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (l1_current_state == 1 && l1_previous_state == 0) {
			front_claw_open = !front_claw_open;
		}

	l1_previous_state = l1_current_state;

	useClaw(front_claw_open);
}

void PrimaryClawNode::autonPeriodic() {

}

PrimaryClawNode::~PrimaryClawNode() {

}