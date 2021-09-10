#pragma once

namespace ohs2020 {

class Climb{

    public:
    void Init();
    Climb();
	void RunDeploy();
	void Deploy();
	void VerticalClimb();
	void SideClimb();
	void LEDCanClimb();

	bool IsEndgame();
	bool CanClimb();
};

}