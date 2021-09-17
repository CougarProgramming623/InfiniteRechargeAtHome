#include "subsystems/Shooter.h"
#include "Robot.h"
#include "RobotID.h"

namespace ohs2020{

    Shooter::Shooter() : 

    m_FlyWheel(RobotID::GetID(FLYWHEEL)),
    m_Feeder(RobotID::GetID(FEEDER)),
    m_FlyWheelEncoder(RobotID::GetID(FLYWHEEL)),

    m_Launcher( [&]         { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(2); }),
    m_FlyWheelToggle( [&]   { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(4); }),
    m_ReverseFeeder( [&]    { return Robot::Get().GetIO().GetButtonBoard().GetRawButton(14); })


    void Shooter::Init(){
        
        SetupShooterButtons();
        m_Feeder.setSafetyEnabled(false);
        DebugOutF("Shooter Init");
    }
}