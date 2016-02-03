/** \file
 * Example of subsystem task declaration.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control a given subsystem.
 *
 */
#ifndef COMPONENT_H
#define COMPONENT_H

/**
	A template class for creating new components
 */
#include <ComponentBase.h>			//For ComponentBase class
#include <pthread.h>

//Robot
#include "WPILib.h"


class Component : public ComponentBase
{
public:
	Component();
	virtual ~Component();
	static void *StartTask(void *pThis)
	{
		((Component *)pThis)->DoWork();
		return(NULL);
	}

private:
	void OnStateChange();
	void Run();
};

#endif			//COMPONENT_H
