/*
 * ShooterSolenoid.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#include <ShooterSolenoid.h>
#include <Arm.h>

ShooterSolenoid::ShooterSolenoid(int canid) : SolenoidBase(canid){
this->canid = canid;
}

ShooterSolenoid::~ShooterSolenoid() {
}

void ShooterSolenoid::Close(){
	this->Set(0, farMask, canid);
}

void ShooterSolenoid::Open(){
	if(Arm::GetEncTarget()== farEncoderPos){
		Set(shootValue, farMask, canid);
	}else if(Arm::GetEncTarget() == closeEncoderPos){
		Set(shootValue, closeMask, canid);
	}else
	{
		printf("Error: invalid shooting position\n");
	}
}
