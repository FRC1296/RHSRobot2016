/** \file
 *  Autonomous script parser
 */

#include <Autonomous.h>
#include <AutoParser.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>

#include <WPILib.h>

//Robot

using namespace std;

const char *szTokens[] = {
		"START",
		"FINISH",
		"MODE",
		"DEBUG",
		"MESSAGE",
		"BEGIN",
		"END",
		"DELAY",			//!<(seconds)
		"MOVE",				//!<(left speed) (right speed)
		"MMOVE",			//!<(speed) (distance:inches) (timeout)
		"TURN",				//!<(degrees) (timeout)
		"STRAIGHT",			//!<(speed) (duration)
		//DRIVETRAIN
		"STARTDRIVEFWD",	//!<(drive speed)
		"STARTDRIVEBCK",	//!<(drive speed)
		"STOPDRIVE",
		"NOP" };
//TODO: add START and FINISH, which send messages to all components
// (Begin and End are doing this now, but they shouldn't)

bool Autonomous::Evaluate(std::string rStatement) {
	char *pToken;
	char *pCurrLinePos;
	int iCommand;
	float fParam1;
	bool bReturn = false; ///setting this to true WILL cause auto parsing to quit!
	string rStatus;

	if(rStatement.empty()) {
		printf("statement is empty");
		return (bReturn);
	}

	// process the autonomous motion

	pCurrLinePos = (char *) rStatement.c_str();

	if(*pCurrLinePos == sComment) {
		return (bReturn);
	}

	// find first token

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	if(pToken == NULL)
	{
		SmartDashboard::PutString("Auto Status","DEATH BY PARAMS!");
		PRINTAUTOERROR;
		rStatus.append("missing token");
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
		return (true);
	}

	// which command are we to execute??
	// this can be (easily) be made much faster... any student want to improve on this?

	for(iCommand = AUTO_TOKEN_MODE; iCommand < AUTO_TOKEN_LAST; iCommand++) {
		//printf("comparing %s to %s\n", pToken, szTokens[iCommand]);
		if(!strncmp(pToken, szTokens[iCommand], strlen(szTokens[iCommand]))) {
			break;
		}
	}
	printf("Evaluate: %d\n", 1);

	if(iCommand == AUTO_TOKEN_LAST) {
		// no valid token found
		rStatus.append("no tokens - check script spelling");
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
		return (true);
	}
	printf("Evaluate: %d\n", 2);

	// if we are paused wait here before executing a real command

	while(bPauseAutoMode)
	{
		Wait(0.02);
	}

	// execute the proper command

	if(iAutoDebugMode)
	{
		printf("%0.3lf %s %s\n", pDebugTimer->Get(), pToken, pCurrLinePos);
	}

	printf("Execute Command: %d\n", iCommand);

	switch (iCommand)
	{

	case AUTO_TOKEN_START_AUTO:
		Start();
		rStatus.append("starting auto");
		break;

	case AUTO_TOKEN_FINISH_AUTO:
		Finish();
		rStatus.append("finishing auto");
		break;

	case AUTO_TOKEN_BEGIN:
		Begin(pCurrLinePos);
		rStatus.append("begin");
		break;

	case AUTO_TOKEN_END:
		End(pCurrLinePos);
		rStatus.append("done");
		bReturn = true;
		break;

	case AUTO_TOKEN_DEBUG:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iAutoDebugMode = atoi(pToken);
		break;

	case AUTO_TOKEN_MESSAGE:
		printf("%0.3lf %03d: %s\n", pDebugTimer->Get(), lineNumber, pCurrLinePos);
		break;

	case AUTO_TOKEN_DELAY:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if (pToken == NULL)
		{
			rStatus.append("missing parameter");
		}
		else
		{
			fParam1 = atof(pToken);
			rStatus.append("wait");

			Delay(fParam1);
		}
		break;

	case AUTO_TOKEN_MOVE:
		if (!Move(pCurrLinePos))
		{
			rStatus.append("move error");
		}
		else
		{
			rStatus.append("move");
		}
		break;

	case AUTO_TOKEN_MMOVE:
		if (!MeasuredMove(pCurrLinePos))
		{
			rStatus.append("move error");
		}
		else
		{
			rStatus.append("move");
		}
		break;

	case AUTO_TOKEN_TURN:
		if (!Turn(pCurrLinePos))
		{
			rStatus.append("turn error");
		}
		else
		{
			rStatus.append("turn");
		}
		break;

	case AUTO_TOKEN_STRAIGHT:
		if (!Straight(pCurrLinePos))
		{
			rStatus.append("straight error");
		}
		else
		{
			rStatus.append("straight");
		}
		break;

	case AUTO_TOKEN_START_DRIVE_FWD:
		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if(pToken == NULL)
		{
			SmartDashboard::PutString("Auto Status","EARLY DEATH!");
			PRINTAUTOERROR;
			rStatus.append("missing token");
			bReturn = false;
		}
		else
		{
			Message.params.autonomous.driveSpeed = atof(pToken);

			Message.command = COMMAND_DRIVETRAIN_STRAIGHT;//simply drives forward
			CommandNoResponse(DRIVETRAIN_QUEUE);
		}

		break;

	case AUTO_TOKEN_START_DRIVE_BCK:
		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		if(pToken == NULL)
		{
			SmartDashboard::PutString("Auto Status","EARLY DEATH!");
			PRINTAUTOERROR;
			rStatus.append("missing token");
			bReturn = false;
		}
		else
		{
			Message.params.autonomous.driveSpeed = atof(pToken);

			Message.command = COMMAND_DRIVETRAIN_STRAIGHT;	//simply drives back
			CommandNoResponse(DRIVETRAIN_QUEUE);
		}
		break;

	case AUTO_TOKEN_STOP_DRIVE:
		Message.command = COMMAND_DRIVETRAIN_STOP;
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	default:
		rStatus.append("unknown token");
		break;
	}

	if(bReturn)
	{
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
	}

	SmartDashboard::PutBoolean("bReturn", bReturn);
	return (bReturn);
}
