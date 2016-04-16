/** \file
 * Gyro classes borrowed from the Rat Pack!
 * The gyro can take up to 15 seconds to become usable.
 */

#include <ADXRS453Z.h>
#include <cstdarg>

ADXRS453Z::ADXRS453Z() {
	spi = new SPI(SPI::kOnboardCS3);
	spi->SetClockRate(4000000); //4 MHz (rRIO max, gyro can go high)
	spi->SetClockActiveHigh();
	spi->SetChipSelectActiveLow();
	spi->SetMSBFirst();

	command[0] = READ_COMMAND;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	iLoop = 0;

	accumulated_angle = 0.0;
	current_rate = 0.0;
	accumulated_offset = 0.0;
	rate_offset = 0.0;
	update_timer = new Timer();
	update_timer->Start();
	calibration_timer = new Timer();
	calibration_timer->Start();

	pTask = new Task("tADSRX543Z", &ADXRS453Z::StartTask, this);
}
ADXRS453Z::~ADXRS453Z() {

}

void ADXRS453Z::StartTask(ADXRS453Z *pThis)
{

	while (true)
	{
		pThis->Update();
		Wait(0.01);
	}
}

void ADXRS453Z::Update() {
	//calibration_timer->Start();
	check_parity(command);
	spi->Transaction(command, data, DATA_SIZE); //perform transaction, get error code

	if (calibration_timer->Get() < WARM_UP_PERIOD)
	{
		lastTime = thisTime = update_timer->Get();
		return;
	}
	else if (calibration_timer->Get() < CALIBRATE_PERIOD)
	{
		Calibrate();
	}
	else
	{
		UpdateData();
	}
}

void ADXRS453Z::UpdateData() {
	int sensor_data = assemble_sensor_data(data);
	float rate = ((float) sensor_data) / 80.0;

	current_rate = rate;
	current_rate -= rate_offset;
	thisTime = update_timer->Get();

	accumulated_offset += rate * (thisTime - lastTime);
	accumulated_angle += current_rate * (thisTime - lastTime);
	lastTime = thisTime;
	iLoop++;
}

void ADXRS453Z::Calibrate() {
	int sensor_data = assemble_sensor_data(data);
	float rate = ((float) sensor_data) / 80.0;

	thisTime = update_timer->Get();
	accumulated_offset += rate * (thisTime - lastTime);
	lastTime = thisTime;
	rate_offset = accumulated_offset
			/ (calibration_timer->Get() - WARM_UP_PERIOD);
	iLoop++;
}

float ADXRS453Z::GetRate() {
	return current_rate;
}

float ADXRS453Z::GetAngle() {
	return accumulated_angle;
}
void ADXRS453Z::SetAngle(float angle){
	accumulated_angle = angle;
}
double ADXRS453Z::PIDGet() {
	return GetAngle()/45;
}

float ADXRS453Z::Offset() {
	return rate_offset;
}

void ADXRS453Z::Reset() {
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	current_rate = 0.0;
	accumulated_angle = 0.0;
	rate_offset = 0.0;
	accumulated_offset = 0.0;

	//calibration_timer->Stop();
	calibration_timer->Reset();

	//update_timer->Stop();
	update_timer->Reset();
}

//a function to simply zero the gyro rather than reset & calibrate. Added by Taylor Smith
void ADXRS453Z::Zero()
{
	current_rate = 0.0;
	accumulated_angle = 0.0;
}

short ADXRS453Z::assemble_sensor_data(unsigned char * data) {
	//cast to short to make space for shifts
	//the 16 bits from the gyro are a 2's complement short
	//so we just cast it too a C++ short
	//the data is split across the output like this (MSB first): (D = data bit, X = not data)
	// X X X X X X D D | D D D D D D D D | D D D D D D X X | X X X X X X X X X
	return ((short) (data[0] & FIRST_BYTE_DATA)) << 14 | ((short) data[1]) << 6
			| ((short) (data[2] & THIRD_BYTE_DATA)) >> 2;
}

void ADXRS453Z::check_parity(unsigned char * command) {
	int num_bits = bits(command[0]) + bits(command[1]) + bits(command[2])
			+ bits(command[3]);

	if (num_bits % 2 == 0)
	{
		command[3] |= PARITY_BIT;
	}
}

int ADXRS453Z::bits(unsigned char val) {
	int n = 0;

	while (val)
	{
		val &= val - 1;
		n += 1;
	}

	return n;
}
