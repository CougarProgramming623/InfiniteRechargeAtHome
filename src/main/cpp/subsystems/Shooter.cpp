#include "subsystems/Shooter.h"
#include "Robot.h"
#include "ohs/RobotID.h"
#include "ohs/Log.h"
#include "Util.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableRegistry.h>

#include "frc2/command/FunctionalCommand.h"

using namespace ohs623;

namespace ohs2020{

const double DefaultShooterPower = 1;

Shooter::Shooter() : 

m_FrontFlywheel(RobotID::GetID(FRONT_FLYWHEEL)),
m_BackFlywheel(RobotID::GetID(BACK_FLYWHEEL)),
m_Feeder(RobotID::GetID(FEEDER)),
m_FlyWheelEncoder(-1),

m_Launcher( [&] 		{ return Robot::Get().GetOI().GetButtonBoard().GetRawButton(2); }),
m_FlyWheelToggle([&] 	{ return Robot::Get().GetOI().GetButtonBoard().GetRawButton(4);	}), 
//m_ConveyorToggle( [&] { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(15); }),
m_ReverseFeeder( [&] { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(14); 	})
{}

void Shooter::Init() {

	SetupShooterButtons();
	m_Feeder.SetSafetyEnabled(false);
	m_BackFlywheel.SetInverted(InvertType::OpposeMaster);
	m_BackFlywheel.Set(ControlMode::Follower, RobotID::GetID(FRONT_FLYWHEEL));
}

inline bool GetFlyWheelToggleState() {
	return Robot::Get().GetOI().GetButtonBoard().GetRawButton(4);
}

void Shooter::SetupShooterButtons() {

	m_FlyWheelToggle.WhenPressed(frc2::FunctionalCommand([this]{}, [this] { //on execute

		m_IsFlywheelOn = true;
		m_FlywheelWU = (int)((double)m_FrontFlywheel.GetSelectedSensorVelocity() / 2048 * 600);
		// DebugOutF(std::to_string(m_FrontFlywheelWU));
		frc::SmartDashboard::PutNumber("Flywheel Speed", m_FlywheelWU);

    // double speed = 3500 + 3500 * Robot::Get().GetOI().GetButtonBoard().GetRawAxis(0);
    double speed = (Robot::Get().GetOI().GetButtonBoard().GetRawAxis(0) + 1) / 2;
    Robot::Get().GetOI().GetButtonBoard().SetOutput(2, m_FlywheelWU > 4400);

		// m_Flywheel.Set(ControlMode::Velocity, speed);
		m_FrontFlywheel.Set(ControlMode::PercentOutput, speed);
		DebugOutF(std::to_string(speed));
		Cob::PushValue(CobKey::FLYWHEEL_SPEED, speed );

	}, [this] (bool f){//on end
		DebugOutF("shooter end");

		m_IsFlywheelOn = false;

		m_FrontFlywheel.Set(ControlMode::Velocity, 0);

	}, [this] { return !GetFlyWheelToggleState(); }, {}));

	m_FlyWheelToggle.WhenReleased(frc2::RunCommand([&] {

		m_FlywheelWU = (int)((double)m_FrontFlywheel.GetSelectedSensorVelocity() / 2048 * 600);
		frc::SmartDashboard::PutNumber("Flywheel Speed", m_FlywheelWU);
        Robot::Get().GetOI().GetButtonBoard().SetOutput(2, false);

		// DebugOutF(std::to_string(m_FlywheelWU));

	}, {}));

  
	m_Launcher.WhenHeld(frc2::RunCommand([&] {

		m_Feeder.Set(ControlMode::PercentOutput, 1);

	}, {} ));

	m_Launcher.WhenReleased(frc2::InstantCommand([&] {

		m_Feeder.Set(ControlMode::PercentOutput, 0);

	}, {} ));
}

frc2::SequentialCommandGroup Shooter::Shoot() {
	frc2::SequentialCommandGroup group = frc2::SequentialCommandGroup();

	frc2::InstantCommand startFlywheel = frc2::InstantCommand( [&] {
		m_FrontFlywheel.Set(ControlMode::Velocity, 6000);
	}, {});


	frc2::InstantCommand startFeeder = frc2::InstantCommand( [&] {
		m_Feeder.Set(ControlMode::PercentOutput, 1);
	}, {});

	frc2::InstantCommand stopShoot = frc2::InstantCommand( [&] {
		m_FrontFlywheel.Set(ControlMode::Velocity, 0);
		m_Feeder.Set(ControlMode::PercentOutput, 0);
	}, {});

	group.AddCommands(
		startFlywheel, 
		frc2::WaitCommand(units::second_t(1.0)), 
		startFeeder, 
		frc2::WaitCommand(units::second_t(4.0)), 
		stopShoot);

	return std::move(group);
}

}//namespace