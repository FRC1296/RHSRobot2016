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
	 //led->Set(Relay::kOn);

	pTask = new Task("tPixy", &PixyCam::Run, this);
}

void PixyCam::Run(PixyCam *pInstance)
{
	 uint8_t uPixiData[2];
	 uint16_t uPixiWord;
	 uint16_t uBlockByteCount;
	 PIXICOM_STATES ePixyComState = PIXYCOM_UNSYNCHED;
	 SPI* pCamera;


	pCamera = new SPI(SPI::kOnboardCS0);
	pCamera->SetMSBFirst();
	pCamera->SetSampleDataOnRising();
	pCamera->SetClockActiveHigh();
	//TODO do the math, is this fast enough?
	pCamera->SetClockRate(2000000);

	 while(true){
		// TODO this is a lot of data, do we need it?  fewer max blocks?
     	//TODO do the math, is this fast enough?

		 Wait(0.00);
		 pCamera->Read(false, uPixiData, 2);
		 uPixiWord = (((uint16_t)uPixiData[0] << 8) & 0xFF00) | ((uint16_t)uPixiData[1] & 0x00FF);   // convert to big endian

		 //printf("data = %04X state %d\n", uPixiWord, ePixyComState);

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
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
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
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
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

			 case  PIXYCOM_STARTOFRAME:
				 ePixyComState = PIXYCOM_STARTOFBLOCK;
				 uBlockByteCount = 0;
				 break;

			 case  PIXYCOM_STARTOFBLOCK:
				 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 uBlockByteCount = 0;
				 break;

			 case  PIXYCOM_GETBLOCKDATA:
				 pInstance->uCurrentBlock[uBlockByteCount++] = uPixiWord;

				 if(uBlockByteCount >= 6)
				 {
					 ePixyComState = PIXYCOM_ENDOFBLOCK;
				 }
				 break;

			 case  PIXYCOM_ENDOFBLOCK:
				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 // is this the last block?
					 uBlockByteCount = 0;
					 ePixyComState = PIXYCOM_ENDOFFRAME;

					 // calibrate on known image, verify x,y,w,h make sense
					 // TODO verify checksum
					 // TODO should we only follow the largest match?
					 // TODO convert x and y to % of full view? - what would help servo math the best
					 // TODO is the first signature labeled 0 or 1?

					 {
						    //std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);

						    if(pInstance->uCurrentBlock[1] == 1)
						    {
						    	// centroid of the largest block (the first signature)

						    	pInstance->bBlockFound = true;
						    	pInstance->fCentroid = (float)((int)pInstance->uCurrentBlock[2] - 159.5) / 159.5; // 0-319 0-199y
						    }
					 }

					 //printf("%d: x = %u, y = %u, w = %u, h = %u\n",
					 //		 pInstance->uCurrentBlock[1],
					 //		 pInstance->uCurrentBlock[2],
					 //		 pInstance->uCurrentBlock[3],
					 //		 pInstance->uCurrentBlock[4],
					 //		 pInstance->uCurrentBlock[5]);
				 }
				 else
				 {
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 break;

			 case  PIXYCOM_ENDOFFRAME:
				 if(uPixiWord == PIXICOM_FRAMESYNCWORD)
				 {
					 // new frame and new block

					 uBlockByteCount = 0;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
				 }
				 else if(uPixiWord == 0)
				 {
					 // new frame but there are no queued objects

					 {
						//std::lock_guard<priority_recursive_mutex> sync(pInstance->mutexData);
						pInstance->bBlockFound = false;
						pInstance->fCentroid = 0.0;
					 }
					 ePixyComState = PIXYCOM_UNSYNCHED;
				 }
				 else
				 {
					 // new block, eat the first word

					 pInstance->uCurrentBlock[0] = uPixiWord;
					 uBlockByteCount = 1;
					 ePixyComState = PIXYCOM_GETBLOCKDATA;
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
	//std::lock_guard<priority_recursive_mutex> sync(mutexData);
	fNewCentroid = fCentroid;
	return(bBlockFound);
}
double PixyCam::PIDGet(){
	//std::lock_guard<priority_recursive_mutex> sync(mutexData);
	SmartDashboard::PutNumber("fCentroid", fCentroid);
	return fCentroid;
}


PixyCam::~PixyCam(){
	delete pTask;
}

