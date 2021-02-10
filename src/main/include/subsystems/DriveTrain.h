#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/PIDCommand.h>
#include <ctre/Phoenix.h>
#include <memory>

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>

#include "SpeedFalcon.h"
#include <frc/SpeedController.h>

namespace ohs2020 {	
 
class DriveTrain : public frc2::SubsystemBase {

public:
	DriveTrain();


	void Init();
	//frc2::Command* GetDefaultCommand() const override;

	void CartesianDrive(double x, double y, double rotation, double angle);

	//virtual void InitDefaultCommand() override;  

	frc2::PIDCommand* TurnToPos(double angle);

	void SetBrakeMode();
	void UseVelocityPID();
	void UsePositionPID();
	void UseMagicPID(double max);
	void SetPID(double, double, double, double, double);
	void SetBrakeMode(bool);
	
	//getters for direct access to motors
	BaseTalon* GetLFront() { return &m_FrontLeft; }
	BaseTalon* GetRFront() { return &m_FrontRight; }
	BaseTalon* GetLBack() { return &m_BackLeft; }
	BaseTalon* GetRBack() { return &m_BackRight; }

protected:
	
	//virtual std::unique_ptr<frc2::Command> TransferOwnership() && override;

private:
	WPI_TalonFX m_FrontLeft{31};
	WPI_TalonFX m_FrontRight{32};
	WPI_TalonFX m_BackLeft{34};
	WPI_TalonFX m_BackRight{33};

	frc2::PIDController* m_TurnController;

	SpeedFalcon m_left1{m_FrontLeft};
	SpeedFalcon m_left2{m_BackLeft};
	SpeedFalcon m_right1{m_FrontRight};
	SpeedFalcon m_right2{m_BackRight};
	/*
	frc2::PIDController m_XController;
	frc2::PIDController m_YController;
	frc::ProfiledPIDController<units::radians> m_ThetaController;

	frc::MecanumDriveKinematics kinematics;
	*/

	 // The motors on the left side of the drive
  	frc::SpeedControllerGroup m_leftMotors{m_left1, m_left2};
 	 // The motors on the right side of the drive
	frc::SpeedControllerGroup m_rightMotors{m_right1, m_right2};
	// The robot's drive
	frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};
	// The left-side drive encoder
	// frc::Encoder m_leftEncoder;
	// // The right-side drive encoder
	// frc::Encoder m_rightEncoder;
};

}//namespace
