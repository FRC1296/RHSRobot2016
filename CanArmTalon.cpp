/*
 * CanArmTalon.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include <CanArmTalon.h>
#include <RobotParams.h>

CanArmTalon::CanArmTalon(int canid) : CANTalon(canid){
}

CanArmTalon::~CanArmTalon() {
}

void CanArmTalon::PIDWrite(float p){ // -1, 1
	//double additive = cos((this->GetEncPosition()+zeroRadian)*countsToRadians)*maxAdditive;
	//double power = p+additive;
	double power = p;
	ABLIMIT(power,0.4);
	this->Set(power);
}

double CanArmTalon::PIDGet(){
	return (this->GetEncPosition());
}
