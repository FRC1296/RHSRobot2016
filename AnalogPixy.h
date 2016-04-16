/*
 * AnalogPixy.h
 *
 *  Created on: Apr 4, 2016
 *      Author: Jacob
 */

#ifndef SRC_ANALOGPIXY_H_
#define SRC_ANALOGPIXY_H_
#include "WPILib.h"
class AnalogPixy {
public:
	AnalogPixy(int,int,double);
	virtual ~AnalogPixy();
	double PIDGet();
	bool IsConnected();
	double Get();


private:
	AnalogInput* input;
	DigitalInput* isConnected;
	double offset;
};

#endif /* SRC_ANALOGPIXY_H_ */
