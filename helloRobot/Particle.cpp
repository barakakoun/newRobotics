#include "Particle.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include "Map.h"
#include "ConfigurationManager.h"

using MapUtilities::Map;

//For new Particles
 Particle::Particle(float x, float y,float yaw, float belief)
{
	pBelief = belief;
	// Add randomized particle
	ApproxParticle(x, y, yaw);
}

// Method which handles the particle with Radius
void Particle::ApproxParticle(float particleX, float particleY, float particleYaw) {
	// Randomize the distance from the given location
    float xRandom =  ((float) rand() / (RAND_MAX)) * (2 * ConfigurationManager::GetMovementDevation()) - ConfigurationManager::GetMovementDevation();
    float yRandom = ((float) rand() / (RAND_MAX)) * (2 * ConfigurationManager::GetMovementDevation()) - ConfigurationManager::GetMovementDevation();

	pX = MathUtils::FixByBoundaries( xRandom + particleX, 0, ConfigurationManager::GetRealWidth() - 1);
	pY = MathUtils::FixByBoundaries( yRandom + particleY,-(ConfigurationManager::GetRealHeight() - 1) ,0);
	//int r = ((float) rand() / (RAND_MAX)) * (2 * DEGREE_DEVATION) - DEGREE_DEVATION;
	//int yaw = MathUtils::RadianToDegree(particleYaw);
	//pYaw = MathUtils::DegreeToRadian(((yaw + r) % MAX_DEGREE));

	float r = ((float) rand() / (RAND_MAX)) * (2 * ConfigurationManager::GetDegreeDevation() * M_PI / 180) - (ConfigurationManager::GetDegreeDevation() * M_PI / 180);

	float yaw = particleYaw + r;
	pYaw = (yaw > 0 ? yaw : M_PI + M_PI + yaw);
}

//Method which handles the particle position update
void Particle::Update(float delX, float delY, float delYaw, float laserScan[]) {
	pX = MathUtils::FixByBoundaries(pX + delX, 0, (ConfigurationManager::GetRealWidth() - 1));
	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << "x ";
	}
	pY = MathUtils::FixByBoundaries(pY + delY, -(ConfigurationManager::GetRealHeight() - 1),0);
	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << "y ";
	}
	//int deltayaw = MathUtils::RadianToDegree(delYaw);
	//int yaw = MathUtils::RadianToDegree(pYaw);
	//pYaw =  MathUtils::DegreeToRadian(((yaw + deltayaw) % MAX_DEGREE));
	float yaw = pYaw + delYaw;
	pYaw = (yaw > 0 ? yaw : M_PI + M_PI + yaw);
	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << "yaw ";
	}

	float preMov = (float)ProbByMovement(delX, delY, delYaw);

	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << "mov " << preMov;
	}

	float predLaser = probByLaser(laserScan);
	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << ", laser " << predLaser;
	}
	// Predict belief (how much i believe i arrived to t-1 + "delta") * movement believe (how much i believe to my movement model) * predict by laser
	// * normalization factor (so the value will not fade away)
	pBelief = MathUtils::FixByBoundaries(ConfigurationManager::GetNormalizatioFactor() * preMov * predLaser * pBelief,
											ConfigurationManager::GetMinBelief(),
											ConfigurationManager::GetMaxBelief());
	if(ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << ", belief :" << pBelief << endl;
	}
	//		if(DEBUG_PARTICLES)
	//		{
	//                cout << "mov: " << preMov << ",laser: " << predLaser << ", ";
	//                Print();
	//		}

}

//Method which calculate the particle's probability by the laser and the map
float Particle::probByLaser(float laserScan[]) {
	float xObj,yObj;
	int xObjByMap,yObjByMap;
	int countMiss = 0;
	int countHit = 0;
	const char* fileName = ConfigurationManager::GetMapFilename();
	Map& pMap = Map::InitializeMap(fileName);
	float distance;

//	for (int i=0; i < ConfigurationManager::GetLaserSamples(); i+=10) {
//			for (int j=0; j < laserArr[i] ; j ++) {
//
//				currLocation = map->getObstacleLocation(_belPos.x,_belPos.y,_belPos.yaw,laserArr[i],i);
//				widthIndex = map->xPosToIndex(currLocation.x * 100);
//				heightIndex = map->yPosToIndex(currLocation.y * 100);
//				currCell = grid[heightIndex][widthIndex];
//
//				// Calculates the obstacle location
//				xObj = MathUtils::FixByBoundaries((j * cos(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pX,0.0, ConfigurationManager::GetGridHeight() - 1);
//				yObj = MathUtils::FixByBoundaries((j * sin(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pY, 0.0, ConfigurationManager::GetGridWidth() - 1);
//				Location location = Location(xObj, yObj,0.0);
//				// Convert the rael location to grid location
//				Point point = MathUtils::RealToGrid(location);
//
//				if (pMap[point.y_][point.x_].cell_state != GridCellState::OBSTACLE) {
//					hit++;
//				} else {
//					miss++;
//				}
//			}
//
//			if (laserArr[i] < LASER_RANGE_MAX) {
//				obsLocation = map->getObstacleLocation(_belPos.x,_belPos.y,_belPos.yaw,laserArr[i],i);
//				widthIndex = map->xPosToIndex(obsLocation.x * 100);
//				heightIndex = map->yPosToIndex(obsLocation.y * 100);
//				currCell = grid[heightIndex][widthIndex];
//
//				if (currCell.color == C_BLACK) {
//					hit++;
//				} else {
//					miss++;
//				}
//			}
//		}
//
//		return (hit / (hit + miss));

	for (int i = 0; i < ConfigurationManager::GetLaserSamples(); i+= 33) {
		distance = MathUtils::FixByBoundaries(laserScan[i], ConfigurationManager::GetLaserRangeMinMeter(), ConfigurationManager::GetLaserRangeMaxMeter());
		// Checks if the current laser ray there's no obstacle ahead
		if (distance > ConfigurationManager::GetOpenPathRange()) {
			for (float j = ConfigurationManager::GetDistanceToTurn(); j <= ConfigurationManager::GetOpenPathRange(); j += ConfigurationManager::GetGridResolution()) {
				// Calculates the obstacle location
				// Calculate the x of the obstacle
				xObj = MathUtils::FixByBoundaries((j * cos(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pX,0.0, ConfigurationManager::GetGridHeight() - 1);
				// Calculate the y of the obstacle
				yObj = MathUtils::FixByBoundaries((j * sin(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pY, 0.0, ConfigurationManager::GetGridWidth() - 1);
				Location location = Location(xObj, yObj,0.0);
				// Convert the rael location to grid location
				Point point = MathUtils::RealToGrid(location);
				// If there's obstacle due to the given grid map
				if (pMap[point.y_][point.x_].cell_state != GridCellState::OBSTACLE)
				{
					countHit++;
				}
				else
				{
					countMiss++;
				}
			}
		}
		// If there's obstacle ahead - Do the exact opposite thing
		else {
			xObj = MathUtils::FixByBoundaries((distance * cos(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pX,0.0, ConfigurationManager::GetGridHeight() - 1);
			yObj = MathUtils::FixByBoundaries((distance * sin(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pY, 0.0, ConfigurationManager::GetGridWidth() - 1);
			Location location = Location(xObj, yObj,0.0);
			Point point = MathUtils::RealToGrid(location);

			if (pMap[point.y_][point.x_].cell_state == GridCellState::OBSTACLE)
			{
				countHit++;
			}
			else
			{
				countMiss++;
			}

			for(float j = ConfigurationManager::GetDistanceToTurn(); j < distance ; j += ConfigurationManager::GetGridResolution())
			{
				xObj = MathUtils::FixByBoundaries((j * cos(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pX,0.0, ConfigurationManager::GetGridHeight() - 1);
				yObj = MathUtils::FixByBoundaries((j * sin(MathUtils::DegreeToRadian(MathUtils::SampleToDegree((float)i)) + pYaw)) + pY, 0.0, ConfigurationManager::GetGridWidth() - 1);
				Location location = Location(xObj, yObj,0.0);
				Point point = MathUtils::RealToGrid(location);

				if (pMap[point.y_][point.x_].cell_state != GridCellState::OBSTACLE)
				{
					countHit++;
				}
				else
				{
					countMiss++;
				}
			}
		}
	}

	return ((float)countHit) / ((float)countMiss + (float)countHit);
}

//Version A of ProbByMovement
//    float Particle::ProbByMovement(float delX, float delY, float delYaw) {
//	float distance = MathUtils::distance(delX, delY);
//        float MaxRadian = MathUtils::DegreeToRadian(MAX_ACCURATE_ROTATION_DEGREE);
//        float NormalRadian = MathUtils::DegreeToRadian(NORMAL_ACCURATE_ROTATION_DEGREE);
//	if ((delYaw < NormalRadian) && (delYaw > -NormalRadian)) {
//		if (distance <= MAX_ACCURATE_MOVEMENT_M)
//                {
//			//return 1- distance/MAX_ACCURATE_MOVEMENT_M;
//                    return 1.0;
//                }
//		else
//                {
//			return (MAX_ACCURATE_MOVEMENT_M + distance + 0.2);
//                    //return 0;
//                }
//	}
//	else if ((delYaw < MaxRadian) && (delYaw > -MaxRadian))
//	{
//		if (distance <= MAX_ACCURATE_MOVEMENT_M)
//                {
//                    return 0.8;
//			//return 0.8 * (1- distance/MAX_ACCURATE_MOVEMENT_M);
//                }
//		else
//                {
//			//return (MAX_ACCURATE_MOVEMENT_M + distance - 0.2);
//                    return (MAX_ACCURATE_MOVEMENT_M + distance - 0.2);
//                   // return 0;
//                }
//	}
//	return 1.0;
//}

// Get the probability to be right by movements done
float Particle::ProbByMovement(float delX, float delY, float delYaw) {
	double distance;
		distance = MathUtils::distance(delX, delY);
		float posiblity;

		// Get positive val of delYaw
		delYaw = abs(delYaw) ;
		if (distance <= 0.5 && delYaw <= 0.5)
			posiblity = 1.0;
		else if (distance <= 2 && delYaw <= 3.0)
			posiblity = 0.7;
		else if (distance <= 2)
			posiblity = 0.4;
		else if (delYaw <= 3.0)
			posiblity = 0.5;
		 else
			 posiblity = 0.3;

		return posiblity;

	//return 1.0;
}

//Method which return the particle Belief value
float Particle::GetBelief() {
	return pBelief;
}

float Particle::GetYaw() {
	return pYaw;
}

float Particle::GetX() {
	return pX;
}

float Particle::GetY() {
	return pY;
}

void Particle::Print(){
	Location l = Location(GetX(),GetY(),GetYaw());
	l.Print();
}

Location Particle::GetLocation() {
	Location loc;
	loc.x_ = GetX();
	loc.y_ = GetY();
	loc.yaw_= GetYaw();
	return loc;
}

bool Particle::operator < (Particle& other)
{
    return (this-> GetBelief() < other.GetBelief());
}


//Destructor of objects type of Particle
Particle::~Particle() {
}


