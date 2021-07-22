#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/button/Button.h>

#include "Cob.h"

// #include "Robot.h"


namespace ohs2020 {

	class Intake {
		public:
		Intake();

		void Init();
		void MoveIntakeUpDown();
		void MoveIntakeRoller();
		void Tick();

		frc2::Button m_IntakeDown;
		frc2::Button m_IntakeUp;

		frc2::Button m_IntakeIn;
		frc2::Button m_IntakeOut;

		CANCoder m_IntakeEncoder;

		WPI_TalonSRX m_IntakeMover;
		WPI_TalonSRX m_BallMover;
	};
}//namespace