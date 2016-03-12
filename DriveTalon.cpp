/*
 * DriveTalon.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: Jacob
 */

#include <DriveTalon.h>
#include <iostream>
#include <fstream>
#include <RobotParams.h>

DriveTalon::DriveTalon(int canid) : CANTalon(canid){
	cand = canid;
	path += std::to_string(cand)+".txt";
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
				file.open(path.c_str());
				file << currentCurrent<< std::endl;
				file.close();
			}else{
				std::ofstream file;
				file.open(path.c_str(), std::ios::out | std::ios::app);
				file << currentCurrent<< std::endl;
				file.close();
			}
		}

		if((currentCurrent>maxCurrentTele && ISTELEOPERATED) || (currentCurrent>maxCurrentAuto && ISAUTO) ){
			if(pCurrentTimer->Get() == 0){
				pCurrentTimer->Start();
			}else if(pCurrentTimer->Get() > maxCurrentTime){
				printf("talon %d has been shutdown", cand);
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

