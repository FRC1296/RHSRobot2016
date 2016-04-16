/*
 * JawSolenoid.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: Jacob
 */

#include <JawSolenoid.h>

JawSolenoid::JawSolenoid(int canid) : SolenoidBase(canid){
this->canid = canid;
}

JawSolenoid::~JawSolenoid() {
}

void JawSolenoid::Close(){
	Set(uCloseValue, uMask, canid);
}

void JawSolenoid::Open(){
	Set(uOpenValue, uMask, canid);
}
