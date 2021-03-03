#include "Trajectory.h"
#include "Robot.h"


#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

namespace ohs2020 {

frc2::Command* CreatePathFollow() {
	frc::DifferentialDriveVoltageConstraint autoVoltageContraint(
		frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
		kDriveKinematics, 10_V
	);

	frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);
	config.SetKinematics(kDriveKinematics);
	config.AddConstraint(autoVoltageContraint);

	auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
		// Start at the origin facing the +X direction
		frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
		// Pass through these two interior waypoints, making an 's' curve path
		{frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
		// End 3 meters straight ahead of where we started, facing forward
		frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
		// Pass the config
		config);

		Robot::Get().GetDriveTrain().ResetOdometry(exampleTrajectory.InitialPose());

		frc2::RamseteCommand ramseteCommand(
		exampleTrajectory, []() { return Robot::Get().GetDriveTrain().GetPose(); },
		frc::RamseteController(kRamseteB, kRamseteZeta),
		frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
		kDriveKinematics,
		[] { return Robot::Get().GetDriveTrain().GetWheelSpeeds(); },
		frc2::PIDController(kPDriveVel, 0, 0),
		frc2::PIDController(kPDriveVel, 0, 0),
		[](auto left, auto right) { Robot::Get().GetDriveTrain().TankDriveVolts(left, right); },
		{});

		return new frc2::SequentialCommandGroup(
			std::move(ramseteCommand),
			frc2::InstantCommand([] {Robot::Get().GetDriveTrain().TankDriveVolts(0_V, 0_V); }, {}));
}

}//namespace