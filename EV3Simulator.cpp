#include "DrawThread_EV3.h"
#include "EV3SimulatorObj.h"

int main()
{
	EV3SimulatorObj ev3;
	ev3.destroyRobot();
	//ev3.makePlane(1, 1, 0.5);
	//ev3.makeParam(0.5);
	ev3.makeParam(0);
	ev3.makeRobot();
	
	ev3.loadBlocksData("test.csv");

	//ev3.makeBlock(0.3, 0, 0.05, 0.1, 0.2, 0.1, 0);
	
	ev3.ev3.setTargetVelocity(0.0,1.6);
	ev3.ev3.target_mangle = 1.6;
	DrawThread_EV3 ds(&ev3, 0.05);
	//ds.activate();
	ds.setRCPFlag();
	while (true)
	{
		if (ev3.ev3.getRightTouch(0.003))
		{
			ev3.ev3.setTargetVelocity(-0.01, 0);
		}
		ev3.update();
		Sleep(10);
	}
	return 0;
}