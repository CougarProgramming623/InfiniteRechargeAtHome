#pragma once

#include <frc/SpeedController.h>

class SpeedFalcon : frc::SpeedController {

	void Set(double speed) {};
	
	void SetVoltage(units::volt_t output) {};
	
	double Get() const {return 0;};
	
	void SetInverted(bool isInverted) {};
	
	bool GetInverted() const {};
	
	void Disable() {};
	
	void StopMotor() {};

	void PIDWrite(double output) {};
};