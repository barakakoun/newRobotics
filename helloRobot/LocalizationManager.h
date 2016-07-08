#pragma once
#include "Particle.h"
#include <vector>
#include "Particle.h"
#include "Structs.h"
#include <libplayerc++/playerc++.h>
#include "Map.h"
#include "lodepng.h"

typedef vector<Particle> particlesVec;

using namespace PlayerCc;
using MapUtilities::Map;
namespace MapUtilities {
    class Map;
}

//Class which controls all aspects following the slam system control
class LocalizationManager {
public:
	//---Methods---
	//Constructor of objects type of LocalizationManager
	LocalizationManager(float xRobot,float yRobot, float yawRobot);

	//Method which gives the ability to update all particles which are stored in the vector
	void UpdateParticles(Location deltaLocation,float laserScan[]);

	//Print all the particles, for diagnostic Purpose
	void PrintParticles();

	//Get the particles by the best average belief
	Location GetEstimateLocation();

	void RefreshBestLocation();

	//Destructor of objects type of LocalizationManager
	virtual ~LocalizationManager();

private:
	//Method which gives the ability to initialize all particles by radius that build by distance and degree deviation
	void InitParticles();

	//---Members---
	particlesVec particles; //Vector which hold all particles
	Location bestLocation;
	float bestBelief;

};
