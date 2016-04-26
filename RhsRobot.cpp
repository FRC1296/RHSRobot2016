/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands sent to the subsystems
 * that implement behaviors for each part for the robot.
 */

#include <ComponentBase.h>
#include <RhsRobot.h>
#include <RobotParams.h>
#include "WPILib.h"

//Robot

RhsRobot::RhsRobot() {
	Controller_1 = NULL;
	drivetrain = NULL;
	autonomous = NULL;
	arm = NULL;
	tail = NULL;
	shooter = NULL;
	hanger = NULL;
	shootSeq = NULL;

	iLoop = 0;
}

RhsRobot::~RhsRobot() {
	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	for(; nextComponent != ComponentSet.end(); ++nextComponent)
	{
		delete (*nextComponent);
	}

	delete Controller_1;
}

void RhsRobot::Init() {
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL; (in constructor)
	 * 			drivetrain = new Drivetrain(); (in RhsRobot::Init())
	 */
	Controller_1 = new Joystick(0);
	drivetrain = new Drivetrain();
	autonomous = new Autonomous();
	arm = new Arm();
	tail = new Tail();
	shooter = new Shooter();
	hanger = new Hanger();
	shootSeq = new ShooterSequence();

	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	if(drivetrain)
	{
		nextComponent = ComponentSet.insert(nextComponent, drivetrain);
	}

	if(autonomous)
	{
		nextComponent = ComponentSet.insert(nextComponent, autonomous);
	}

	if(arm)
	{
		nextComponent = ComponentSet.insert(nextComponent, arm);
	}

	if(tail)
	{
		nextComponent = ComponentSet.insert(nextComponent, tail);
	}

	if(shooter)
	{
		nextComponent = ComponentSet.insert(nextComponent, shooter);
	}

	if(hanger)
	{
		nextComponent = ComponentSet.insert(nextComponent, hanger);
	}
}

void RhsRobot::OnStateChange() {
	std::vector<ComponentBase *>::iterator nextComponent;

	for(nextComponent = ComponentSet.begin();
			nextComponent != ComponentSet.end(); ++nextComponent)
	{
		(*nextComponent)->SendMessage(&robotMessage);
	}
}

void RhsRobot::Run() {
	/* Poll for control data and send messages to each subsystem. Surround blocks with if(component) so entire components can be disabled
	 * by commenting out their construction.
	 * EXAMPLE: if(drivetrain) 
	 * 			{ 
	 * 				//Check joysticks and send messages 
	 * 			}
	 */

	if(autonomous)
	{
		if(GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS)
		{
			// all messages to components will come from the autonomous task
			return;
		}
	}

	if (drivetrain)
	{
#if 0
		robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
		//robotMessage.params.tankDrive.left = TANK_DRIVE_LEFT;
		//robotMessage.params.tankDrive.right = TANK_DRIVE_RIGHT;
		robotMessage.params.tankDrive.left = 0.75;
		robotMessage.params.tankDrive.right = 0.75;
		drivetrain->SendMessage(&robotMessage);
#endif
		robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_CHEEZY;
		 			robotMessage.params.cheezyDrive.wheel = CHEEZY_DRIVE_WHEEL;
		 			robotMessage.params.cheezyDrive.throttle = CHEEZY_DRIVE_THROTTLE;
		 			robotMessage.params.cheezyDrive.bQuickturn = CHEEZY_DRIVE_QUICKTURN;
		drivetrain->SendMessage(&robotMessage);

		if(DRIVE_ZERO_GYRO){
			drivetrain->ZeroGyro();
		}

		if(DRIVE_SEARCHON){
			robotMessage.command = COMMAND_AUTONOMOUS_SEARCHGOAL;
			robotMessage.params.armParams.direction = true;
			drivetrain->SendMessage(&robotMessage);
		}
		if(DRIVE_SEARCHOFF){
			robotMessage.command = COMMAND_AUTONOMOUS_SEARCHGOAL;
			robotMessage.params.armParams.direction = false;
			drivetrain->SendMessage(&robotMessage);
		}
	}

	if(arm){
		if(ARM_ENABLE){
			robotMessage.command = COMMAND_ARM_ENABLE;
			arm->SendMessage(&robotMessage);
		}

		if(ARM_FAR){
			robotMessage.command = COMMAND_ARM_FAR;
			arm->SendMessage(&robotMessage);

			//  TKB - not required for batter shot
			//if(tail){
			//	robotMessage.command = COMMAND_TAIL_LOWER;
			//	tail->SendMessage(&robotMessage);
			//}
			/*}else if(ARM_CLOSE){
			robotMessage.command = COMMAND_ARM_CLOSE;
			arm->SendMessage(&robotMessage);
			if(tail){
				robotMessage.command = COMMAND_TAIL_LOWER;
				tail->SendMessage(&robotMessage);
			}*/
		}else if(ARM_CLOSE){
			robotMessage.command = COMMAND_ARM_CLOSE;
			arm->SendMessage(&robotMessage);
		}
	else if(ARM_MOVE_INTAKE){
			robotMessage.command = COMMAND_ARM_MOVE_INTAKE;
			arm->SendMessage(&robotMessage);
		}else if(ARM_MOVE_RIDE){
			robotMessage.command = COMMAND_ARM_MOVE_RIDE;
			arm->SendMessage(&robotMessage);
		}

		if(ARM_INTAKE_IN){
			robotMessage.command = COMMAND_ARM_INTAKE;
			robotMessage.params.armParams.direction = true;
			arm->SendMessage(&robotMessage);
		}else if (ARM_INTAKE_OUT){
			robotMessage.command = COMMAND_ARM_INTAKE;
			robotMessage.params.armParams.direction = false;
			arm->SendMessage(&robotMessage);
		}
	}

	if(tail){
		if(TAIL_UP && TAIL_DOWN){

		}else
		if(TAIL_UP){
			robotMessage.command = COMMAND_TAIL_RAISE;
			tail->SendMessage(&robotMessage);
		}else
		if(TAIL_DOWN){
			robotMessage.command = COMMAND_TAIL_LOWER;
			tail->SendMessage(&robotMessage);
		}
	}

	if(shooter){
		if(SHOOTER_SHOOT){
			if(arm)
			{
				shootSeq->StartSequence();

			}
		}
	}

	if(hanger){
		if(HANGER_HANG){
			robotMessage.command = COMMAND_HANGER_HANG;
			hanger->SendMessage(&robotMessage);
			robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_CHEEZY;
			 			robotMessage.params.cheezyDrive.wheel = 0;
			 			robotMessage.params.cheezyDrive.throttle = .1;
			 			robotMessage.params.cheezyDrive.bQuickturn = false;
			drivetrain->SendMessage(&robotMessage);
		}
	}

	if((iLoop++ % 50) == 0)
	 	{
	 		robotMessage.command = COMMAND_SYSTEM_CONSTANTS;
	 		robotMessage.params.system.fBattery = DriverStation::GetInstance().GetBatteryVoltage();

	 		// send to interested subsystems

	 		drivetrain->SendMessage(&robotMessage);
	 	}
}

START_ROBOT_CLASS(RhsRobot)
