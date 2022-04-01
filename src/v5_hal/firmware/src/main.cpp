#include "main.h"

NodeManager* nodeManager;

ControllerNode* controller;

InertialSensorNode* inertialSensorNode;

ADIEncoderNode* xOdomEncoder;
ADIEncoderNode* yOdomEncoder;

OdometryNode* odomNode;

HolonomicDriveNode* holonomicDriveNode;
MotorNode* leftFrontMotor;
MotorNode* leftRearMotor;
MotorNode* rightFrontMotor;
MotorNode* rightRearMotor;

ADIMotorNode* clawMotor;
ADIMotorNode* liftMotor;

ConnectionCheckerNode* connectionCheckerNode;

// Declare all robot nodes here:

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Create the node manager
	nodeManager = new NodeManager(pros::millis);

	// Initialize a static logger
	Logger::giveNodeManager(nodeManager);

	// Initialize all robot nodes here:

	controller = new ControllerNode(nodeManager, "controller");

	inertialSensorNode = new InertialSensorNode(nodeManager, "inertialSensor", 19);

	xOdomEncoder = new ADIEncoderNode(nodeManager, 'A', 'B', "xOdomEncoder", false);
	yOdomEncoder = new ADIEncoderNode(nodeManager, 'C', 'D', "yOdomEncoder", false);

	odomNode = new OdometryNode(nodeManager, "odometry", xOdomEncoder, yOdomEncoder, inertialSensorNode, OdometryNode::FOLLOWER);

	leftFrontMotor = new MotorNode(nodeManager, 11,"leftFrontMotor", false);
	leftRearMotor = new MotorNode(nodeManager, 12,"leftRearMotor", false);
	rightFrontMotor = new MotorNode(nodeManager, 13,"rightFrontMotor", true);
	rightRearMotor = new MotorNode(nodeManager, 14,"rightRearMotor", true);

	HolonomicDriveNode::HolonomicFourMotors holonomicDriveMotors = {
		leftFrontMotor,
		leftRearMotor,
		rightFrontMotor,
		rightRearMotor
	};

	EncoderConfig holonomicEncoderConfig = {
		0, // Initial ticks
		600, // Ticks per RPM
		1.975 // Wheel diameter
	};

	HolonomicDriveKinematics::HolonomicWheelLocations holonomicWheelLocations = {
		Vector2d(-3.576, 3.576), // Left front
		Vector2d(-3.576, -3.576), // Left rear
		Vector2d(3.576, 3.576), // Right front
		Vector2d(3.576, -3.576) // Right rear
	};

	HolonomicDriveKinematics holonomicDriveKinematics(holonomicEncoderConfig, holonomicWheelLocations);

	holonomicDriveNode = new HolonomicDriveNode(nodeManager, "drivetrain", controller, inertialSensorNode, holonomicDriveMotors, holonomicDriveKinematics);

	// Call the node manager to initialize all of the nodes above
	nodeManager->initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	while (pros::competition::is_disabled()) {
		nodeManager->m_handle->spinOnce();
	}
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Reset all nodes to default configuration
	nodeManager->reset();

	// Reset the chosen autonomous and initialize
	// auton_manager_node->selected_auton->AutonInit();
	
	// Execute autonomous code
	while (pros::competition::is_autonomous()) {
		nodeManager->executeAuton();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 *
 * NOTE: If custom code is needed outside of the node manager, it should be put
 * into a different task with a wait. Each node has an embedded timing control loop
 * and adding a wait to this thread will disrupt the performance of all nodes.
 */
void opcontrol() {
	// Reset all nodes to default configuration
	nodeManager->reset();
	
	// Execute teleop code
	while (true) {
		nodeManager->executeTeleop();
	}
}
