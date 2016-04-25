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
		"MLINE",			//!<(speed) (distance:inches) (timeout)
		"TURN",				//!<(degrees) (timeout)
		"STRAIGHT",			//!<(speed) (duration)
		"SEARCH",
		"AIM",
		"INTAKE",
		"STOPINTAKE",
		"RIDE",
		"LOWEST",
		"AFTERSHOOT",
		"THROWUP",
		"SHOOT",
		"SETANGLE",
		"LOWER",
		"RAISE",
		"TAILDOWN",
		"TAILUP",
		"REDSENSE",
		//DRIVETRAIN
		"STARTDRIVEFWD",	//!<(drive speed)
		"STARTDRIVEBCK",	//!<(drive speed)
		"STOPDRIVE",
		"SHORT",
		"JAWOPEN",
		"JAWCLOSE",
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

	if(iCommand == AUTO_TOKEN_LAST) {
		// no valid token found
		rStatus.append("no tokens - check script spelling");
		printf("%0.3lf %s\n", pDebugTimer->Get(), rStatement.c_str());
		return (true);
	}

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

	case AUTO_TOKEN_MLINE:
		if (!MeasuredMoveToLine(pCurrLinePos))
		{
			rStatus.append("move line error");
		}
		else
		{
			rStatus.append("move line");
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
	case AUTO_TOKEN_SEARCH:
		printf("search token\n");
		if (!Search())
		{
			rStatus.append("search error");
		}
		else
		{
			rStatus.append("search");
		}
		break;
	case AUTO_TOKEN_AIM:
		if (!Aim())
		{
			rStatus.append("aim error");
		}
		else
		{
			rStatus.append("aiming");
		}
		break;

	case AUTO_TOKEN_INTAKE:
		if(!Intake())
		{
			rStatus.append("intake error");
		}
		else
		{
			rStatus.append("intaking");
		}
		break;

	case AUTO_TOKEN_INTAKESTOP:
		Message.command = COMMAND_ARM_INTAKE_STOP;
		CommandNoResponse(ARM_QUEUE);
		break;

	case AUTO_TOKEN_RIDE:
		if(!Ride())
		{
			rStatus.append("ride error");
		}
		else
		{
			rStatus.append("ride");
		}
		break;
	case AUTO_TOKEN_AFTERSHOOT:
		Message.command = COMMAND_ARM_MOVE_AFTERSHOOT;
		CommandNoResponse(ARM_QUEUE);
		break;
	case AUTO_TOKEN_LOWERINTAKE:
		Message.command = COMMAND_AUTONOMOUS_MOVEINTAKE;
		CommandNoResponse(ARM_QUEUE);
		break;

	case AUTO_TOKEN_THROWUP:
		if(!Throwup())
		{
			rStatus.append("throwup error");
		}
		else
		{
			rStatus.append("throwing up");
		}
		break;

	case AUTO_TOKEN_SHOOT:
		printf("shooting\n");
		if(!Shoot())
		{
			rStatus.append("shoot error");
		}
		else
		{
			rStatus.append("shooting");
		}
		break;
	case AUTO_TOKEN_SETANGLE:
		if(!SetAngle())
		{
			rStatus.append("angle set error");
		}
		else
		{
			rStatus.append("setting angle");
		}
		break;

	case AUTO_TOKEN_LOWER:
		if(!Lower())
		{
			rStatus.append("lower error");
		}
		else
		{
			rStatus.append("lowering");
		}
		break;
	case AUTO_TOKEN_RAISE:
		if(!Raise())
		{
			rStatus.append("lower error");
		}
		else
		{
			rStatus.append("lowering");
		}
		break;

	case AUTO_TOKEN_TAILDOWN:
		if(!TailDown())
		{
			rStatus.append("tail down error");
		}
		else
		{
			rStatus.append("lowering tail");
		}
		break;

	case AUTO_TOKEN_TAILUP:
		if(!TailUp())
		{
			rStatus.append("raise tail error");
		}
		else
		{
			rStatus.append("raising tail");
		}
		break;

	case AUTO_TOKEN_REDSENSE:
		if(!RedSense())
		{
			rStatus.append("sensing error");
		}
		else
		{
			rStatus.append("finding goal with red");
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

	case AUTO_TOKEN_SHORT:
		if(!Short())
		{
			rStatus.append("short error");
		}
		else
		{
			rStatus.append("short shot");
		}
		break;

	case AUTO_TOKEN_JAWOPEN:
		Message.command = COMMAND_SHOOTER_JAW_OPEN;
		CommandNoResponse(SHOOTER_QUEUE);
		break;

	case AUTO_TOKEN_JAWCLOSE:
		Message.command = COMMAND_SHOOTER_JAW_CLOSE;
		CommandNoResponse(SHOOTER_QUEUE);
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
