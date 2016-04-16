/*
 * CanArmTalon.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: Jacob
 */

#include <CanArmTalon.h>
#include <RobotParams.h>

CanArmTalon::CanArmTalon(int canid) : CANTalon(canid){
	pBurnTimer = new Timer();
}

CanArmTalon::~CanArmTalon() {
}

void CanArmTalon::PIDWrite(float p){ // -1, 1
	//double additive = cos((this->GetEncPosition()+zeroRadian)*countsToRadians)*maxAdditive;
	//double power = p+additive;
	double power = p;
	ABLIMIT(power,0.6);
	this->Set(power);

	if(this->GetOutputCurrent()>maxArmCurrent){
		if(pBurnTimer->Get()>timeout){
			SmartDashboard::PutBoolean("ARM CURRENT", false);
			this->Disable();
			pBurnTimer->Stop();
			pBurnTimer->Reset();
		}else{
			pBurnTimer->Start();
		}
	}else{
		pBurnTimer->Stop();
		pBurnTimer->Reset();
	}
}

double CanArmTalon::PIDGet(){
	return (this->GetPulseWidthPosition());
}
