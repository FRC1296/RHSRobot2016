/*
 * CanArmTalon.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include <CanArmTalon.h>

CanArmTalon::CanArmTalon(int canid) : CANTalon(canid){
}

CanArmTalon::~CanArmTalon() {
}

void CanArmTalon::PIDWrite(float p){ // -1, 1
	double additive = cos((this->GetPulseWidthPosition()+zeroRadian)*countsToRadians)*maxAdditive;
	printf("writing %f \n", p+additive);

this->Set(p+additive);
}

double CanArmTalon::PIDGet(){
	printf("getting %d \n", this->GetPulseWidthPosition() );
	return (this->GetPulseWidthPosition())*1.0;
}
