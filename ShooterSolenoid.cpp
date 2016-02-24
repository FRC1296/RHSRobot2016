/*
 * ShooterSolenoid.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#include <ShooterSolenoid.h>
#include <Arm.h>

ShooterSolenoid::ShooterSolenoid(int canid) : SolenoidBase(canid){

}

ShooterSolenoid::~ShooterSolenoid() {
}

void ShooterSolenoid::Close(){
	this->Set(0, farMask, 0);
}

void ShooterSolenoid::Open(){
if(Arm::GetEncTarget() == farEncoderPos){
	this->Set(shootValue, farMask, 0);
}else if(Arm::GetEncTarget() == closeEncoderPos){
	this->Set(shootValue, closeMask, 0);
}

}
