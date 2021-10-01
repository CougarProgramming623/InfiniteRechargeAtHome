#pragma once

#include <frc/DriverStation.h>

#include <ctre/Phoenix.h>

namespace ohs623 {

enum Motor { 
	FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, FRONT_FLYWHEEL, BACK_FLYWHEEL, FEEDER, CLIMB_LEFT, CLIMB_RIGHT, INTAKE_MOVER, BALL_MOVER,
	MOTOR_COUNT
};

enum BotType {
	MARK, SABER,
	BOT_TYPE_COUNT
};


enum class MotorControllerType {
	TALON_SRX, TALON_FX
};

struct MotorData {
	int ID;
	MotorControllerType Controller;
};

class RobotID {    

public:
	RobotID() = delete;

	static const MotorData& Get(Motor);
	static int GetID(Motor);
	static BaseTalon* InitMotor(Motor);

	inline static BotType GetCurrentBot() { return s_CurrentBot; }

private:
	static BotType s_CurrentBot;

};

} // namepsace