/*
 * CanArmTalon.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#ifndef SRC_CANARMTALON_H_
#define SRC_CANARMTALON_H_
#include "WPILib.h"

class CanArmTalon : public CANTalon{
public:
	CanArmTalon(int canid);
	virtual ~CanArmTalon();

private:
	void PIDWrite(float p);
	double PIDGet();

	const double countsToRadians = 3.14159265/2/4096;
	const double zeroRadian = -1096;
	const double maxAdditive = .2; // max idle value to add at 0 degrees
};

#endif /* SRC_CANARMTALON_H_ */
