#include "subsystems/Shooter.h"
#include "Robot.h"
#include "RobotID.h"

namespace ohs2020{

    const double DefaultShooterPower = 1;

    Shooter::Shooter() : 

    m_FlyWheel(RobotID::GetID(FLYWHEEL)),
    m_Feeder(RobotID::GetID(FEEDER)),
    m_FlyWheelEncoder(RobotID::GetID(FLYWHEEL)),

    m_Launcher( [&]         { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(2); }),
    m_FlyWheelToggle( [&]   { return Robot::Get().GetOI().GetButtonBoard().GetRawButton(4); }),
    m_ReverseFeeder( [&]    { return Robot::Get().GetIO().GetButtonBoard().GetRawButton(14); })
    {}

    void Shooter::Init(){
        
        SetupShooterButtons();
        m_Feeder.setSafetyEnabled(true);
        DebugOutF("Shooter Init");
    }

    inline bool GetFlyWheelToggleState() {
        return Robot::Get().GetIO().GetButtonBoard().GetRawButton(4);
    }

    void Shooter::SetupShooterButtons(){

        m_FlyWheelToggle.WhenPressed(frc2::FunctionalCommand([this]{}, [this] { //on execute

            m_IsFlywheelOn = true;
            m_FlywheelWU = (int)((double)m_Flywheel.GetSelectedSensorVelocity() / 2048 * 600 );
            frc::SmartDashboard::PutNumber("Flywheel Speed", m_FlywheelWU);

        }))
    }
}