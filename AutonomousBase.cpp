/** \file
 * The AutonomousBase component class handles basic autonomous functionality.
 */

#include <Autonomous.h>
#include <ComponentBase.h>
#include <RobotParams.h>
#include "WPILib.h"
//Local
#include <iostream>
#include <fstream>
#include <string>


using namespace std;

Autonomous::Autonomous()
: ComponentBase(AUTONOMOUS_TASKNAME, AUTONOMOUS_QUEUE, AUTONOMOUS_PRIORITY)
{
	lineNumber = 0;
	bInAutoMode = false;
	bPauseAutoMode = false;
	bScriptLoaded = false;
	iAutoDebugMode = 0;
	uResponseCount = 0;
	bReceivedCommandResponse = false;
	ReceivedCommand = COMMAND_UNKNOWN;

	pDebugTimer = new Timer();
	pDebugTimer->Start();

	pTask = new Task(AUTONOMOUS_TASKNAME, &Autonomous::StartTask, this);
	wpi_assert(pTask);

	pScript = new Task(AUTOEXEC_TASKNAME, &Autonomous::StartScript, this);
	wpi_assert(pScript);
}

Autonomous::~Autonomous()	//Destructor
{
	delete(pTask);
	delete(pScript);
}

void Autonomous::Init()	//Initializes the autonomous component
{
}
 
void Autonomous::OnStateChange()	//Handles state changes
{
	// to handle unexpected state changes before the auto script finishes (like in OKC last year)
	// we will leave the script running

	if(localMessage.command == COMMAND_ROBOT_STATE_AUTONOMOUS)
	{
		bPauseAutoMode = false;
		bInAutoMode = true;
		pDebugTimer->Reset();
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED)
	{
		bPauseAutoMode = true;
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_DISABLED)
	{
		bPauseAutoMode = true;
	}
}

void Autonomous::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_AUTONOMOUS_RUN:
			break;

		case COMMAND_CHECKLIST_RUN:
			break;

		case COMMAND_AUTONOMOUS_RESPONSE_OK:
			uResponseCount++;
			bReceivedCommandResponse = true;
			ReceivedCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
			break;

		case COMMAND_AUTONOMOUS_RESPONSE_ERROR:
			uResponseCount++;
			bReceivedCommandResponse = true;
			ReceivedCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
			break;

		default:
			break;
	}
}

bool Autonomous::LoadScriptFile()
{
	bool bReturn = true;
	ifstream scriptStream;
	scriptStream.open(AUTONOMOUS_SCRIPT_FILEPATH);
	
	if(scriptStream.is_open())//not working
	{
		for(int i = 0; i < AUTONOMOUS_SCRIPT_LINES; ++i)
		{
			if(!scriptStream.eof())
			{
				getline(scriptStream, script[i]);
				cout << script[i] << endl;
			}
			else
			{
				script[i].clear();
			}
		}

		scriptStream.close();
	}	
	else
	{
		bReturn = false;
	}

	return(bReturn);
}

void Autonomous::DoScript()
{
	//int loadAttemptTally = 0; //for debugging
	SmartDashboard::PutString("Script Line", "DoScript started");
	SmartDashboard::PutString("Auto Status", "Ready to go");
	SmartDashboard::PutBoolean("Script File Loaded", false);
	//printf("DoScript\n");
	
	while(true)
	{
		lineNumber = 0;
		SmartDashboard::PutNumber("Script Line Number", lineNumber);

		//TODO: LoadScriptFile is called every stinking time we want to check load status! Does this eat time?
		//We want to load the file while disabled - this allows us to load new scripts
		if(LoadScriptFile() == false)
		{
			// wait a little and try again, really only useful if when practicing

			SmartDashboard::PutBoolean("Script File Loaded", false);
			Wait(1.0);
		}
		else
		{
			SmartDashboard::PutBoolean("Script File Loaded", true);
			// if there is a script we will execute it some heck or high water!

			while (bInAutoMode)
			{
				SmartDashboard::PutNumber("Script Line Number", lineNumber);

				if (!bPauseAutoMode)
				{
					if (lineNumber < AUTONOMOUS_SCRIPT_LINES)
					{
						// can we have empty lines?  at the end I guess

						if (script[lineNumber].empty() == false)
						{
							// handle pausing in the Evaluate method

							SmartDashboard::PutString("Script Line",
									script[lineNumber].c_str());

							if (Evaluate(script[lineNumber]))
							{
								SmartDashboard::PutString("Script Line", "<NOT RUNNING>");
								break;
							}
						}

						lineNumber++;
					}
					else
					{
						break;
					}
				}
			}

			bInAutoMode = false;
			Wait(0.1);
		}
	}

	bInAutoMode = false;
}
