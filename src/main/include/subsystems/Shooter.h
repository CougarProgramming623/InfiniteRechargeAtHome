#pragma once

#include <frc2/command/SequentialCommandGroup.h>

namespace ohs2020{

class Shooter{

public:
    Shooter(){}

	void Init();

	void SetupShooterButtons(){}

	bool GetFlywheelState() { return false; }

	double GetFlywheelWU() { return 0; }


private:
	bool m_isLoaded = false;
};
    
}