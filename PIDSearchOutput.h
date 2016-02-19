/*
 * PIDSearchOutput.h
 *
 *  Created on: Feb 5, 2016
 *      Author: fmj
 */

#ifndef SRC_PIDSEARCHOUTPUT_H_
#define SRC_PIDSEARCHOUTPUT_H_
#include "WPILib.h"
class PIDSearchOutput : public PIDOutput{
public:
	PIDSearchOutput(CANTalon*, CANTalon* , bool);
	virtual ~PIDSearchOutput();
	void PIDWrite(float);

private:
	CANTalon* pLeftOneMotor;

	CANTalon* pRightOneMotor;

	bool bUnderServoControl;
};

#endif /* SRC_PIDSEARCHOUTPUT_H_ */
