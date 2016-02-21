/*
 * ShooterSolenoid.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#include <ShooterSolenoid.h>

ShooterSolenoid::ShooterSolenoid(int canid) : SolenoidBase(canid){

}

ShooterSolenoid::~ShooterSolenoid() {
}

void ShooterSolenoid::Close(){
	this->Set(0, mask, 0);
}

void ShooterSolenoid::Open(){
	this->Set(value, mask, 0);
}
