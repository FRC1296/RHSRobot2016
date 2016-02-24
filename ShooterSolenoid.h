/*
 * ShooterSolenoid.h
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#ifndef SRC_SHOOTERSOLENOID_H_
#define SRC_SHOOTERSOLENOID_H_
#include "WPILib.h"
#include "RobotParams.h"
class ShooterSolenoid : public SolenoidBase{
public:
	ShooterSolenoid(int canid);
	virtual ~ShooterSolenoid();
	void Open();
	void Close();

private:
	const uint8_t farMask = (0x1 << SOL_SHOOTER_1) | (0x1 << SOL_SHOOTER_2) | (0x1 << SOL_SHOOTER_3) | (0x1 << SOL_SHOOTER_4);
	const uint8_t closeMask = (0x1 << SOL_SHOOTER_1) | (0x1 << SOL_SHOOTER_3);
	const uint8_t shootValue = 0xf;
};

#endif /* SRC_SHOOTERSOLENOID_H_ */
