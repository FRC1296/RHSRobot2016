/*
 * Tail.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#ifndef SRC_TAIL_H_
#define SRC_TAIL_H_
#include "WPILib.h"
#include <ComponentBase.h>


class Tail : public ComponentBase{
public:
	Tail();
	virtual ~Tail();

	static void *StartTask(void *pThis)
	{
		((Tail *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon* pTailMotor;

	const float fIdleVoltage = .0f;
	const float fRaiseVoltage = 1.0f;
	const float fLowerVoltage = 1.0f;

	void OnStateChange();
	void Run();
	void Lower();
	void Raise();
};

#endif /* SRC_TAIL_H_ */
