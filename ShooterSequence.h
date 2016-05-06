/*
 * ShooterSequence.h
 *
 *  Created on: Apr 18, 2016
 *      Author: Jacob
 */

#ifndef SRC_SHOOTERSEQUENCE_H_
#define SRC_SHOOTERSEQUENCE_H_
#include "WPILib.h"
#include "RobotMessage.h"
#include "RobotParams.h"
#include "RobotSequence.h"

class ShooterSequence : public RobotSequence{
public:
	ShooterSequence();
	virtual ~ShooterSequence();
	void Run();
private:
	const float armDelay = 1.0f;
	const float rotateBack = 0.025;  // wait and rotate the intake
	const float preShootDelay = 0.7;
	const float postShootDelay = 0.5;
	const float clawCloseDelay = 0.5;
};

#endif /* SRC_SHOOTERSEQUENCE_H_ */
