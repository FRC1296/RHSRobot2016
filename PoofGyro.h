/*
 * PoofGyro.h
 *
 *  Created on: Apr 13, 2016
 *      Author: Jacob
 */

#ifndef SRC_POOFGYRO_H_
#define SRC_POOFGYRO_H_
#include "WPILib.h"

class PoofGyro : public PIDSource{
public:
	//PoofGyro();
	PoofGyro(SPI::Port);
	virtual ~PoofGyro();
	static constexpr double kCalibrationSampleTime = 5.0;
	void Calibrate();
	void StartCalibrate();
	void EndCalibrate();
	void CancelCalibrate();
	double GetCenter();
	void Reset();
	void Free();
	double GetAngle();
	double GetRate();
	double PIDGet();
	void Zero();
	void SetAngle(float an);
private:
	static constexpr double kSamplePeriod = 0.001;
	static constexpr double kDegreePerSecondPerLSB = -0.0125;
	static constexpr int kPIDRegister = 0x0C;
	SPI* m_spi;
	bool m_is_calibrating;
	double m_last_center;
	double offset;

	bool CalcParity(int v);
	int ReadRegister(int reg);
};

#endif /* SRC_POOFGYRO_H_ */
