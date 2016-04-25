/*
 * ShooterSequence.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: Jacob
 */

#include <ShooterSequence.h>
#include "Arm.h"

ShooterSequence::ShooterSequence() : RobotSequence(SHOOTER_SEQ_TASKNAME){

}

ShooterSequence::~ShooterSequence() {

}

void ShooterSequence::Run(){
	printf("shooter seq started\n");
	RobotMessage robotMessage;

	if(Arm::GetEncTarget()== closeEncoderPos || Arm::GetEncTarget()== farEncoderPos){ // Setup arm pos first
		printf("arm in good position\n");
	}else{ // If the arm isnt in position
		printf("moving arm to close\n");
		robotMessage.command = COMMAND_ARM_CLOSE;
		SendMessage(ARM_QUEUE, &robotMessage);
		Wait(armDelay); // Wait for arm to lower
	}

	printf("waiting\n");
	Wait(clawOpenDelay);	// Wait for arm to get in position
	printf("sending message to jaw\n");
	robotMessage.command = COMMAND_SHOOTER_JAW_OPEN;
	SendMessage(SHOOTER_QUEUE, &robotMessage);
	printf("sent message\n");
	printf("waiting\n");


	robotMessage.command = COMMAND_ARM_INTAKE_OUT;
	SendMessage(ARM_QUEUE, &robotMessage);
	printf("waiting\n");
	Wait(rotateBack);

	robotMessage.command = COMMAND_ARM_INTAKE_STOP;
	SendMessage(ARM_QUEUE, &robotMessage);
	printf("waiting\n");

	Wait(preShootDelay);	// Wait for jaw to open

	robotMessage.command = COMMAND_SHOOTER_SHOOTER_OPEN;
	SendMessage(SHOOTER_QUEUE, &robotMessage);


	Wait(postShootDelay);	// Wait for ball to shoot

	robotMessage.command = COMMAND_SHOOTER_SHOOTER_CLOSE;
	SendMessage(SHOOTER_QUEUE, &robotMessage);
	printf("waiting l\n");
	//Wait(clawCloseDelay);	// Wait for shooters to close

	robotMessage.command = COMMAND_SHOOTER_JAW_CLOSE;
	SendMessage(SHOOTER_QUEUE, &robotMessage);

	robotMessage.command = COMMAND_ARM_MOVE_RIDE; // Go back to ride height afterwards
	SendMessage(ARM_QUEUE, &robotMessage);

}


