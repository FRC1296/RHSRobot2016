/*
 * DriveTalon.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: Jacob
 */

#include <DriveTalon.h>
#include <iostream>
#include <fstream>

DriveTalon::DriveTalon(int canid) : CANTalon(canid){
	cand = canid;
	pCurrentTimer = new Timer();
	pTask = new Task("drive"+canid, &DriveTalon::StartTask, this);
	wpi_assert(pTask);
}

DriveTalon::~DriveTalon() {
	// TODO Auto-generated destructor stub
}

void DriveTalon::Run(){

	for(;;){
		float currentCurrent = this->GetOutputCurrent();
		if(currentCurrent != 0){
			if(lastCurrent == 0){
				std::ofstream file;
				file.open("/home/lvuser/drive"+cand);
				file << currentCurrent<< std::endl;
				file.close();
			}else{
				std::ofstream file;
				file.open("/home/lvuser/drive"+cand, std::ios::out | std::ios::app);
				file << currentCurrent<< std::endl;
				file.close();
			}
		}

		if(currentCurrent>maxCurrent){
			if(pCurrentTimer->Get() == 0){
				pCurrentTimer->Start();
			}else if(pCurrentTimer->Get() > maxCurrentTime){
				this->Disable();
				pCurrentTimer->Stop();
				pCurrentTimer->Reset();
			}
		}else{
			pCurrentTimer->Stop();
			pCurrentTimer->Reset();
		}
		Wait(.005);
		lastCurrent = currentCurrent;
	}
}

void DriveTalon::ResetCurrentTimeout(){
	pCurrentTimer->Stop();
	pCurrentTimer->Reset();
	this->Enable();
}

