/*
 * AnalogPixy.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: Jacob
 */

#include <AnalogPixy.h>

AnalogPixy::AnalogPixy(int analog, int digital, double offset) {
	input = new AnalogInput(analog);
	isConnected = new DigitalInput(digital);
	this->offset = offset;
}

AnalogPixy::~AnalogPixy() {
	delete input;
	delete isConnected;
}

bool AnalogPixy::IsConnected(){
	return isConnected->Get();
}

double AnalogPixy::Get(){
if(IsConnected()){
return (1-input->GetVoltage()/3.3*2-offset);
}
return 5;
}

double AnalogPixy::PIDGet(){
	return Get();
}
