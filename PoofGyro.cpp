/*
----------------------------------------------------------------------------
 Copyright (c) FIRST 2015-2016. All Rights Reserved.
 Open Source Software - may be modified and shared by FRC teams. The code
 must be accompanied by the FIRST BSD license file in the root directory of
 the project.
 MODIFIED BY TEAM 254
 MODIFIED BY TEAM 1296
----------------------------------------------------------------------------


 * Use a rate gyro to return the robots heading relative to a starting position.
 * The Gyro class tracks the robots heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a
 * short calibration routine where it samples the gyro while at rest to
 * determine the default offset. This is subtracted from each sample to
 * determine the heading.
 *
 * This class is for the digital ADXRS453 gyro sensor that connects via SPI.
 */
#include "PoofGyro.h"

PoofGyro::PoofGyro(SPI::Port port){
	offset = 0;
    m_spi = new SPI(port);
    m_spi->SetClockRate(3000000);
    m_spi->SetMSBFirst();
    m_spi->SetSampleDataOnRising();
    m_spi->SetClockActiveHigh();
    m_spi->SetChipSelectActiveLow();

    // Validate the part ID
    /* This part doesnt work quite yet in c++ - Jacob
int r = (ReadRegister(kPIDRegister) & 0xff00);
    if (r != 0x5200) {
        Free();
        DriverStation::ReportError("Could not find ADXRS453 gyro");
        return;
    }
*/
    // http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf
    m_spi->InitAccumulator(kSamplePeriod, 0x20000000, 4, 0x0c00000E, 0x04000000, 10, 16, true, true);
    // m_spi.initAccumulator(kSamplePeriod, 0x80000000, 4, 0xEFE00000,
    // 0x4E000000, 5, 16, true, true);

    Calibrate();
}

/**
 * This is a blocking calibration call. There are also non-blocking options
 * available in this class!
 *
 */

double PoofGyro::PIDGet() {
	return GetAngle()/45;
}

void PoofGyro::SetAngle(float ang){
offset = -m_spi->GetAccumulatorValue() * kDegreePerSecondPerLSB * kSamplePeriod;
offset += ang;
}

void PoofGyro::Zero(){
offset = -m_spi->GetAccumulatorValue() * kDegreePerSecondPerLSB * kSamplePeriod;
}

 void PoofGyro::Calibrate() {
        Wait(0.1); // Wait for things to settle down
        StartCalibrate();
        Wait(kCalibrationSampleTime);
        EndCalibrate();
}

void PoofGyro::StartCalibrate() {
        if (m_spi == NULL)
            return;

        if (!m_is_calibrating) {
            m_is_calibrating = true;
            m_spi->SetAccumulatorCenter(0);
            m_spi->ResetAccumulator();
        }
    }

void PoofGyro::EndCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false;
            m_last_center = m_spi->GetAccumulatorAverage();
            m_spi->SetAccumulatorCenter((int) round(m_last_center));
            m_spi->ResetAccumulator();
        }
    }

void PoofGyro::CancelCalibrate() {
        if (m_is_calibrating) {
            m_is_calibrating = false;
            m_spi->SetAccumulatorCenter((int) round(m_last_center));
            m_spi->ResetAccumulator();
        }
    }

double PoofGyro::GetCenter() {
        return m_last_center;
    }

bool PoofGyro::CalcParity(int v) {
        bool parity = false;
        while (v != 0) {
            parity = !parity;
            v = v & (v - 1);
        }
        return parity;
    }

int PoofGyro::ReadRegister(int reg) {
        int cmdhi = 0x8000 | (reg << 1);
        bool parity = CalcParity(cmdhi);

        uint8_t buf[4];
        buf[0] = (uint8_t)(cmdhi >> 8);
        buf[1] = (uint8_t)(cmdhi & 0xff);
        buf[2] = (uint8_t)(0);
        buf[3] = (uint8_t)(parity ? 0 : 1);


        m_spi->Write(buf, 4);
        m_spi->Read(false, buf, 4);

        if ((buf[0] & 0xe0) == 0) {
            return 0; // error, return 0
        }
        return (buf[0] >> 5) & 0xffff;
    }

void PoofGyro::Reset() {
        if (m_is_calibrating) {
            CancelCalibrate();
        }
        m_spi->ResetAccumulator();
    }

void PoofGyro::Free() {
        if (m_spi != NULL) {
            m_spi->FreeAccumulator();
            m_spi = NULL;
        }
    }

double PoofGyro::GetAngle() {
        if (m_spi == NULL)
            return 0.0;
        if (m_is_calibrating) {
            return 0.0;
        }
        return m_spi->GetAccumulatorValue() * kDegreePerSecondPerLSB * kSamplePeriod+offset;
    }

double PoofGyro::GetRate() {
        if (m_spi == NULL)
            return 0.0;
        if (m_is_calibrating) {
            return 0.0;
        }
        return m_spi->GetAccumulatorLastValue() * kDegreePerSecondPerLSB;
    }


PoofGyro::~PoofGyro(){
	delete m_spi;
}
