#include "nodes/BackClawNode.h"

BackClawNode::BackClawNode(NodeManager* node_manager, std::string handle_name, 
		ControllerNode* controller, ADIDigitalOutNode* back_claw) : 
		IClawNode(node_manager, handle_name), 
		m_controller(controller),
		m_back_claw(back_claw) {
	
}

void BackClawNode::initialize() {

}

void BackClawNode::useClaw(bool opened) {
	if (opened) {
		m_back_claw->setValue(0);
	} else {
        m_back_claw->setValue(1);
	}
}

void BackClawNode::teleopPeriodic() {
	bool a_current_state = m_controller->getController()->get_digital(pros::E_CONTROLLER_DIGITAL_A);

	if (a_current_state == 1 && a_previous_state == 0) {
			back_claw_open = !back_claw_open;
		}

	a_previous_state = a_current_state;

    useClaw(back_claw_open);
}

void BackClawNode::autonPeriodic() {

}

BackClawNode::~BackClawNode() {

}