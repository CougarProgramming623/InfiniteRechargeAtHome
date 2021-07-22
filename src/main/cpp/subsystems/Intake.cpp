#include "subsystems/Intake.h"
#include "ohs/RobotID.h"
#include "Robot.h"

#include <string>

using namespace ohs623;

namespace ohs2020 {

Intake::Intake() :

	m_IntakeEncoder(RobotID::GetID(INTAKE_MOVER)),
	m_IntakeMover(RobotID::GetID(INTAKE_MOVER)),
	m_BallMover(RobotID::GetID(BALL_MOVER)),

	m_IntakeDown([&] 	{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(10);	}),
	m_IntakeUp([&] 		{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(9);	}),

	m_IntakeIn([&] 		{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(6);	}),
	m_IntakeOut([&] 	{return Robot::Get().GetOI().GetButtonBoard().GetRawButton(11);	})

{
	ctre::phoenix::ErrorCode error = m_IntakeMover.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 100);
	
	std::stringstream buffer;
	buffer << "error: " << error;
	DebugOutF(buffer.str());
}//constructor


void Intake::Init() {
	MoveIntakeUpDown();
	MoveIntakeRoller();
}//Init

void Intake::MoveIntakeUpDown() {
	m_IntakeDown.WhileHeld(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0.25);
		DebugOutF("Intake Moving Down");
	}, {}));
	m_IntakeDown.WhenReleased(frc2::InstantCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("Down Stopped");
	}, {}));

	m_IntakeUp.WhileHeld(frc2::RunCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, -0.25);
		DebugOutF("Intake Moving Up");
	}, {}));
	m_IntakeUp.WhenReleased(frc2::InstantCommand([&] {
		m_IntakeMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("Up Stopped");
	}, {}));
}//MoveIntakeUpDown

void Intake::MoveIntakeRoller() {
	m_IntakeIn.WhileHeld(frc2::RunCommand([&] {
		m_BallMover.Set(ControlMode::PercentOutput, 0.75);
		DebugOutF("Roller Moving In");
	}, {}));
	m_IntakeIn.WhenReleased(frc2::InstantCommand([&] {
		m_BallMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("In Stopped");
	}, {}));

	m_IntakeOut.WhileHeld(frc2::RunCommand([&] {
		m_BallMover.Set(ControlMode::PercentOutput, -1);
		DebugOutF("Roller Moving Out");
	}, {}));
	m_IntakeOut.WhenReleased(frc2::InstantCommand([&] {
		m_BallMover.Set(ControlMode::PercentOutput, 0);
		DebugOutF("Out Stopped");
	}, {}));
}//MoveIntakeRoller

void Intake::Tick() {
	double position = m_IntakeMover.GetSelectedSensorPosition();
	Cob::PushValue(CobKey::INTAKE_POSITION, position);
	// OHS_DEBUG([&] (auto &f) { f << "position: " << position;});

	std::stringstream buffer;
	buffer << "position: " << position;
	DebugOutF(buffer.str());
}

}//namespace