#pragma once

#include <frc/SpeedController.h>
#include <ctre/Phoenix.h>
#include <Util.h>

class SpeedFalcon : public frc::SpeedController {

	public:

		SpeedFalcon(BaseTalon& talonParameter) : talon {talonParameter} {/*why*/};

		void Set(double speed) { 
			talon.Set(ControlMode::PercentOutput, speed);
		};
		
		void SetVoltage(units::volt_t output) {
			DebugOutF("We cannot set the voltage you clown");
		};
		
		double Get() const {return talon.GetSelectedSensorVelocity();};
		
		void SetInverted(bool isInverted) {talon.SetInverted(isInverted);};
		
		bool GetInverted() const {return talon.GetInverted();};
		
		void Disable() {StopMotor();};
		
		void StopMotor() {talon.Set(ControlMode::PercentOutput, 0);};

		void PIDWrite(double output) {DebugOutF("What does this mean you clown");};

	private:

		ctre::phoenix::motorcontrol::can::BaseTalon& talon;
};