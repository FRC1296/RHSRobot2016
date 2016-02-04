/*
 * PixyCam.cpp
 *
 *  Created on: Jan 14, 2016
 *      Author: fmj
 */

#include <PixyCam.h>

PixyCam::PixyCam() {

	bBlockFound = false;
	fCentroid = 0.0;

	pTask = new Task("tPixy", &PixyCam::Run, this);
}

void PixyCam::Run(PixyCam *pInstance)
{
	 uint8_t uPixiData[2];
	 uint16_t uPixiWord;
	 uint16_t uBlockByteCount;
	 PIXICOM_STATES ePixyComState = PIXYCOM_UNSYNCHED;
	 SPI* pCamera;

	pCamera = new SPI(SPI::kOnboardCS1);
	pCamera->SetMSBFirst();
	pCamera->SetSampleDataOnRising();
	pCamera->SetClockActiveHigh();
	//TODO do the math, is this fast enough?
	pCamera->SetClockRate(100000);

	 while(true){
		// TODO this is a lot of data, do we need it?  fewer max blocks?
     	//TODO do the math, is this fast enough?

		 Wait(0.0005);
		 pCamera->Read(false, uPixiData, 2);
		 uPixiWord = (((uint16_t)uPixiData[0] << 8) & 0xFF00) | ((uint16_t)uPixiData[1] & 0x00FF);   // convert to big endian

		 switch (ePixyComState)
		 {
			 case  PIXYCOM_UNSYNCHED:
				 // look for synch word

				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 ePixyComState = PIXYCOM_LOOK4SECONDSYNCWORD;
				 }
				 else if(uPixiWord == 0)
				 {
					 {
						std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						pInstance->bBlockFound = false;
						pInstance->fCentroid = 0.0;
					 }
				 }
				 break;

			 case  PIXYCOM_LOOK4SECONDSYNCWORD:
				 // look for second consecutive synch word

				 //TODO look for color sync word as well?

				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 uBlockByteCount = 0;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 }
				 else if(uPixiWord == 0)
				 {
					 {
						std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						pInstance->bBlockFound = false;
						pInstance->fCentroid = 0.0;
					 }
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 else
				 {
					 // keep looking for a new frame

					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 break;

			 case  PIXYCOM_GETBLOCKDATA:
				 pInstance->uCurrentBlock[uBlockByteCount++] = uPixiWord;

				 if(uBlockByteCount >= 6)
				 {
					 {
						// centroid of the largest block (the first signature)
						std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);

						pInstance->fCentroid = (float)((int)pInstance->uCurrentBlock[2] - 160) / 160.0;

						if((pInstance->fCentroid < 1.0) && (pInstance->fCentroid > -1.0))
						{
							pInstance->bBlockFound = true;
						}
						else
						{
							pInstance->bBlockFound = false;
						}
					 }

				    // ignore the other blocks
					ePixyComState = PIXYCOM_UNSYNCHED;
					Wait(0.02);
				 }
				 break;

			 default:
				 // should never get here!
				 ePixyComState = PIXYCOM_UNSYNCHED;
				 break;
		 }
	 }
}

bool PixyCam::GetCentroid(float &fNewCentroid)
{
	std::lock_guard<priority_recursive_mutex> sync(mutexData);

	fNewCentroid = fCentroid;
	return(bBlockFound);
}

PixyCam::~PixyCam(){
	delete pTask;
}

