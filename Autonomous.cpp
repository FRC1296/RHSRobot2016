/** \file
 * Class for our autonomous behaviours
 *
 *  This file contains our autonomous algorithms.  It should detect if we are in
 *  autonomous mode or not, select an algorithm based upon switch settings at
 *  the driver station and implement the behaviours till autonomous mode ends.
 */

#include <Autonomous.h>
#include <AutoParser.h>
#include <ComponentBase.h>
#include "WPILib.h"
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "ShooterSequence.h"

//Robot
#include <RobotParams.h>

using namespace std;

extern "C" {
}

bool Autonomous::CommandResponse(const char *szQueueName) {
	int iPipeXmt;
	bool bReturn = true;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	Message.replyQ = AUTONOMOUS_QUEUE;
	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);

	bReceivedCommandResponse = false;

	while (!bReceivedCommandResponse)
	{
		//purposefully empty
	}

	if(iAutoDebugMode)
	{
		printf("%0.3lf Response received\n", pDebugTimer->Get());
	}

	if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
	{
		SmartDashboard::PutString("Auto Status","auto ok");
		bReturn = true;
	}
	else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		PRINTAUTOERROR;
		bReturn = false;
	}

	return bReturn;
}


//UNTESTED
//USAGE: MultiCommandResponse({DRIVETRAIN_QUEUE, CONVEYOR_QUEUE}, {COMMAND_DRIVETRAIN_STRAIGHT, COMMAND_CONVEYOR_SEEK_TOTE});
bool Autonomous::MultiCommandResponse(vector<char*> szQueueNames, vector<MessageCommand> commands) {
	//wait for several commands at once
	//check that queue list is as long as command list
	if(szQueueNames.size() != commands.size())
	{
		SmartDashboard::PutString("Auto Status","MULTICOMMAND error!");
		return false;
	}
	bool bReturn = true;
	int iPipeXmt;
	uResponseCount = 0;
	//vector<int> iPipesXmt = new vector<int>();
	//send messages to each component
	for (unsigned int i = 0; i < szQueueNames.size(); i++)
	{
		iPipeXmt = open(szQueueNames[i], O_WRONLY);
		wpi_assert(iPipeXmt > 0);

		Message.replyQ = AUTONOMOUS_QUEUE;
		Message.command = commands[i];
		write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
		close(iPipeXmt);
	}

	bReceivedCommandResponse = false;

	while (uResponseCount < szQueueNames.size())
	{
		while (!bReceivedCommandResponse)
		{
			//purposefully empty
		}

		if(iAutoDebugMode)
		{
			printf("%0.3lf Response received\n", pDebugTimer->Get());
		}

		if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
		{
			SmartDashboard::PutString("Auto Status", "auto ok");
			bReturn = true;
		}
		else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
		{
			SmartDashboard::PutString("Auto Status", "EARLY DEATH!");
			bReturn = false;
		}
	}
	return bReturn;
}

bool Autonomous::CommandNoResponse(const char *szQueueName) {
	int iPipeXmt;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);
	return (true);
}

void Autonomous::Delay(float delayTime)
{
	//breaks the delay into little bits to prevent issues in the event of disabling
	for (double fWait = 0.0; fWait < delayTime; fWait += 0.01)
	{
		// if we are paused break the delay into pieces

		while (bPauseAutoMode)
		{
			Wait(0.02);
		}

		Wait(0.01);
	}
}

bool Autonomous::Start()
{
	//TODO write Autonomous::Start()
	return(false);
}

bool Autonomous::Finish()
{
	//TODO write Autonomous::Finish()
	return(false);
}

bool Autonomous::Begin(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_RUN;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::End(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_COMPLETE;
	CommandNoResponse(DRIVETRAIN_QUEUE);
	return (true);
}

bool Autonomous::Stop(char *pCurrLinePos) {
	//tell those who need to know that the autonomous behavior is over - reset variables
	Message.command = COMMAND_DRIVETRAIN_STOP;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Move(char *pCurrLinePos) {
	char *pToken;
	float fLeft;
	float fRight;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fLeft = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fRight = atof(pToken);

	if ((fabs(fLeft) > MAX_VELOCITY_PARAM)
			|| (fabs(fRight) > MAX_VELOCITY_PARAM))
	{
		return (false);
	}
	Message.command = COMMAND_DRIVETRAIN_AUTO_MOVE;
	Message.params.tankDrive.left = fLeft;
	Message.params.tankDrive.right = fRight;

	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMove(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_MSTRAIGHT;
	Message.params.autonomous.driveSpeed = fSpeed;
	Message.params.autonomous.driveDistance = fDistance;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMoveToLine(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","EARLY DEATH!");
		return (false);
	}

	fDistance = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_MLINE;
	Message.params.autonomous.driveSpeed = fSpeed;
	Message.params.autonomous.driveDistance = fDistance;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Straight(char *pCurrLinePos) {
	char *pToken;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		return (false);
	}

	fSpeed = atof(pToken);
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		return (false);
	}

	fTime = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_STRAIGHT;
	Message.params.autonomous.driveSpeed = fSpeed;
	Message.params.autonomous.timeout = fTime;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Search(){
	printf("auto search\n");
	Message.command = COMMAND_AUTONOMOUS_SEARCHGOAL;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::SetAngle(){
	Message.command = COMMAND_DRIVETRAIN_SETANGLE;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Intake(){
	Message.command = COMMAND_AUTONOMOUS_INTAKE;
	return (CommandNoResponse(ARM_QUEUE));
}

bool Autonomous::Ride(){
	Message.command = COMMAND_ARM_MOVE_RIDE;
	CommandNoResponse(ARM_QUEUE);
	Message.command = COMMAND_ARM_INTAKE_STOP;
	return (CommandNoResponse(ARM_QUEUE));
}

bool Autonomous::Throwup(){
	Message.command = COMMAND_AUTONOMOUS_THROWUP;
	return (CommandResponse(ARM_QUEUE));
}

bool Autonomous::Shoot(){
	Message.command = COMMAND_AUTONOMOUS_SHOOT;
	CommandResponse(ARM_QUEUE);
	Message.command = COMMAND_AUTONOMOUS_SHOOT;
	CommandResponse(DRIVETRAIN_QUEUE);
	printf("getting instance\n");
	ShooterSequence ss = ShooterSequence();
	ss.Run();
	printf("got instance\n");
	return true;
}

bool Autonomous::Short(){
	ShooterSequence ss = ShooterSequence();
	ss.Run();
	return true;
}

bool Autonomous::Aim(){
	Message.command = COMMAND_AUTONOMOUS_SHOOT;
	return 	CommandResponse(DRIVETRAIN_QUEUE);
}

bool Autonomous::Lower(){
	Message.command = COMMAND_ARM_AUTO_MOVE_RIDE;
	return(CommandNoResponse(ARM_QUEUE));
}

bool Autonomous::Raise(){
	Message.command = COMMAND_ARM_FAR;
	return(CommandNoResponse(ARM_QUEUE));
}

bool Autonomous::TailDown(){
	Message.command = COMMAND_TAIL_LOWER;
	return(CommandNoResponse(TAIL_QUEUE));
}

bool Autonomous::TailUp(){
	Message.command = COMMAND_TAIL_RAISE;
	return(CommandNoResponse(TAIL_QUEUE));
}

bool Autonomous::RedSense(){
	Message.command = COMMAND_DRIVETRAIN_REDSENSE;
	return(CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Turn(char *pCurrLinePos) {
	char *pToken;
	float fAngle;
	float fTimeout;

	// parse remainder of line to get target angle and timeout
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fAngle = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		return (false);
	}

	fTimeout = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_TURN;
	Message.params.autonomous.turnAngle = fAngle;
	Message.params.autonomous.timeout = fTimeout;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}
