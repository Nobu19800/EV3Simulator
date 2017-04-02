#include "DrawThread.h"
#include "EV3SimulatorObj.h"

int main()
{
	EV3SimulatorObj ev3;
	DrawThread ds(&ev3, 0.01);
	ds.activate();
	while (true)
	{
		ev3.update();
		Sleep(10);
	}
	return 0;
}