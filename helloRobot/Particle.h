#pragma once

//#define MAX_ACCURATE_ANGLE_TO_MOVE 1.0
//#define NORMAL_ACCURATE_ANGLE_TO_MOVE 0.5
//#define SAFE_DISTANCE_TO_MOVE 0.3
//#define NORMALIZEDFACTOR 2
//#define LASER_MAX_RANGE 4
//#define LASER_ANGLE_RANGE 240
//#define OPEN_PATH_RANGE 0.75
//#define RTOD(r)   ((r) * 180 / M_PI)
//#define DTOR(d)   ((d) * M_PI / 180)
//#define NORMALIZE(z)   atan2(sin(z), cos(z))
//#define M_TO_CM(m)   (m * 100)
//#define ROBOT_DIM 2
//#define SENSOR_FROM_END 15
//#define SENSOR_DETECTION_RANGE 60
//#define CELL_DIM 5
//#define INDEX_PER_DEGREE 6
//#define MAX_DEGREE 360
#include "MathUtils.h"
#include "Structs.h"
#include "Map.h"
#include <math.h>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;
using namespace std;

namespace MapUtilities {
    class Map;
}

//Class which controls all aspects following the particle's system control
class Particle {
public:
    //Method which handles the particle position update
    Particle(float x, float y,float yaw, float belief);
    
	void Update(float delX, float delY, float delYaw, float laserScan[]);

	//Method which return the particle Belief value
    float GetBelief();
	float GetY();
	float GetX();
	float GetYaw();
   //Method that return the location by the biggest belief
	Location GetLocation();
	void Print();
	bool operator < (Particle& other);
	//Destructor of objects type of Particle
	~Particle();

private:
	float pX;
	float pY;
	float pYaw;
	float pBelief;
	float* GetLaserScan();

    void ApproxParticle(float particleX, float particleY, float particleYaw);
	float probByLaser(float laserScan[]);
	float ProbByMovement(float delX, float delY, float delYaw);
};
