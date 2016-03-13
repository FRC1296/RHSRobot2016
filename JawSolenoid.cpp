/*
 * JawSolenoid.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#include <JawSolenoid.h>

JawSolenoid::JawSolenoid(int canid) : SolenoidBase(canid){

}

JawSolenoid::~JawSolenoid() {
}

void JawSolenoid::Close(){
	this->Set(uCloseValue, uMask, 0);
}

void JawSolenoid::Open(){
	this->Set(uOpenValue, uMask, 0);
}
