/*
 * JawSolenoid.h
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#ifndef SRC_JAWSOLENOID_H_
#define SRC_JAWSOLENOID_H_

#include "WPILib.h"
#include "RobotParams.h"

class JawSolenoid : public SolenoidBase{
public:
	JawSolenoid(int canid);
	virtual ~JawSolenoid();
	void Open();
	void Close();

private:
	const uint8_t uMask = (0x1 << SOL_JAWCLOSE_1) | (0x1 << SOL_JAWCLOSE_2) |
		(0x1 << SOL_JAWOPEN_1) | (0x1 << SOL_JAWOPEN_2);
	const uint8_t uCloseValue = 0x30;
	const uint8_t uOpenValue = 0xC0;
};

#endif /* SRC_JAWSOLENOID_H_ */
