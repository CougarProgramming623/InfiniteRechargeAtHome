#include "subsystems/Intake.h"
#include "ohs/RobotID.h"
#include "Robot.h"

using namespace ohs623;

namespace ohs2020 {

Intake::Intake() :

	m_IntakeMover(RobotID::GetID(INTAKE_MOVER)),
	m_BallMover(RobotID::GetID(BALL_MOVER)),

	m_IntakeDown([&] 	{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(10);	}),
	m_IntakeUp([&] 		{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(9);	}),

	m_IntakeIn([&] 		{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(0);	}),
	m_IntakeOut([&] 	{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(0);	})
{



}//constructor

void Intake::Init() {
	MoveIntake();
}//Init

void Intake::MoveIntake() {
	m_IntakeDown.WhileHeld(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0.1);
		DebugOutF("Intake Moving Down");
	}, {}));
	m_IntakeDown.WhenReleased(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("Down Stopped");
	}, {}));

	m_IntakeUp.WhileHeld(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, -0.1);
		DebugOutF("Intake Moving Up");
	}, {}));
	m_IntakeUp.WhenReleased(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("Up Stopped");
	}, {}));

}//MoveIntake

}//namespace