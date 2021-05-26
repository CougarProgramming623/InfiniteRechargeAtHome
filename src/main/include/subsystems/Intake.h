#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/button/Button.h>

#include "Robot.h"


namespace ohs2020 {

	class Intake {
		public:
		Intake();

		void Init();
		void MoveIntake();
		void Tick();

		private:
		frc2::Button m_IntakeDown;
		frc2::Button m_IntakeUp;

		frc2::Button m_IntakeIn;
		frc2::Button m_IntakeOut;

		WPI_TalonSRX m_IntakeMover;
		WPI_TalonSRX m_BallMover;
	};
}//namespace