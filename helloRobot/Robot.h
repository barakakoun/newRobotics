#pragma once

#include "Structs.h"
#include "Map.h"
#include "LocalizationManager.h"
#include <libplayerc++/playerc++.h>
#include <vector>
using namespace std;
using namespace PlayerCc;

namespace MapUtilities {
    class Map;
}

class Robot {
public:
	//---Members---
	PlayerClient* pc;
	Position2dProxy* pp;
	LaserProxy* lp;
	LocalizationManager* lm;
	Location PreviousLocation;
	Location CurrentLocation;
	//For checking in the simulator
	float maxDevation;

	//---Methods---
	Robot(string ip, int port);
	void SetSpeed(float speed, float Angular);
	float GetLaser(int index);
	float* GetLaserScan();
	void Refresh();
	void PrintRealPosition();
	Location GetRealLocation();
	void PrintEstimatePosition();
//	void SaveEstimatePositionToPng(Map& grid);
	Location GetEstimateLocation();
	Location GetRealLocationForGrid();
	Location GetEstimateLocationForGrid();
	virtual ~Robot();

private:
	float GetXPos();
	float GetYPos();
	float GetYaw();
	void Read();
};

