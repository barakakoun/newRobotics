#include <iostream>
#include "Robot.h"
#include <libplayerc++/playerc++.h>
#include "MathUtils.h"
#include "Structs.h"
#include "Map.h"
#include "ConfigurationManager.h"

using namespace std;
using namespace PlayerCc;

using MapUtilities::Map;

Robot::Robot(string ip, int port) {
	this->pc = new PlayerClient(ip, port);
	this->lp = new LaserProxy(pc);
	this->pp = new Position2dProxy(pc);
	maxDevation = 0.0;

	if (ConfigurationManager::REAL_WORLD) {
		pp -> SetMotorEnable(true);
	}

	SetSpeed(0, 0);

	//Set the start position
	CurrentLocation.x_ = ConfigurationManager::GetDefaultX();
	CurrentLocation.y_ = ConfigurationManager::GetDefaultY();
	CurrentLocation.yaw_ = MathUtils::DegreeToRadian(
			ConfigurationManager::GetDefaultYaw());
	//In real world there is need to call odemtry number of times
	pp -> SetOdometry(CurrentLocation.x_, CurrentLocation.y_,
			CurrentLocation.yaw_);
	pc -> Read();

	double tempo = pp->GetXPos();

	// Until the robot is in the start position
	while (tempo != CurrentLocation.x_) {
		pp -> SetOdometry(CurrentLocation.x_, CurrentLocation.y_,
					CurrentLocation.yaw_);
		tempo = pp->GetXPos();
		pc -> Read();
	}

	PreviousLocation = Location(CurrentLocation.x_, CurrentLocation.y_,
			CurrentLocation.yaw_);

	//To clean the junk
	for (int i = 0; i < 15; i++) {
		pc -> Read();
	}

	// To randomize numbers before initialize particles
	srand(time(NULL));
	// Initialize the localization manager
	this-> lm = new LocalizationManager(CurrentLocation.x_, CurrentLocation.y_,
			CurrentLocation.yaw_);
}

float Robot::GetXPos() {
	if (ConfigurationManager::REAL_WORLD) {
		return pp ->GetXPos();
	} else {
		//If not in the real world add noise by configure devation
		float r = ((float) rand() / (RAND_MAX)) * (2
				* ConfigurationManager::GetMovementDevation())
				- ConfigurationManager::GetMovementDevation();
		return MathUtils::FixByBoundaries(pp ->GetXPos() + r, 0.0,
				ConfigurationManager::GetRealWidth() - 1);
	}

}

float Robot::GetYPos() {
	if (ConfigurationManager::REAL_WORLD) {
		return pp ->GetYPos();
	} else {
		//If not in the real world add noise by configure devation
		float r = ((float) rand() / (RAND_MAX)) * (2
				* ConfigurationManager::GetMovementDevation())
				- ConfigurationManager::GetMovementDevation();
		return MathUtils::FixByBoundaries(pp ->GetYPos() + r,
				-(ConfigurationManager::GetRealHeight() - 1), 0.0);
	}

}

float Robot::GetYaw() {
	if (ConfigurationManager::GetRealWidth()) {
		float yaw = pp->GetYaw();
		return (yaw > 0 ? yaw : M_PI + M_PI + yaw);
	} else {
		//If not in the real world add noise by configure devation
		//int r = ((float) rand() / (RAND_MAX)) * (2 * DEGREE_DEVATION) - DEGREE_DEVATION;
		//int yaw = MathUtils::RadianToDegree(pp ->GetYaw());
		//return MathUtils::DegreeToRadian(((yaw + r) % MAX_DEGREE));

		float r = ((float) rand() / (RAND_MAX)) * (2
				* ConfigurationManager::GetDegreeDevation() * M_PI / 180)
				- (ConfigurationManager::GetDegreeDevation() * M_PI / 180);
		float yaw = pp->GetYaw() + r;
		return (yaw > 0 ? yaw : M_PI + M_PI + yaw);
	}
}

void Robot::SetSpeed(float speed, float Angular) {
	pp -> SetSpeed(speed, Angular);
}

void Robot::Read() {
	(*pc).Read();
	(*pc).Read();
}

//Read and update the particles, print the estimate as equivalent to real location (only in simulator)
void Robot::Refresh() {
	Robot::Read();
	// To randomize numbers
	srand(time(NULL));

	//Calculate the delta position that the robot passed
	CurrentLocation = Location(GetXPos(), GetYPos(), GetYaw());
	Location delta;
	delta.x_ = GetRealLocation().x_ - PreviousLocation.x_;
	delta.y_ = GetRealLocation().y_ - PreviousLocation.y_;
	delta.yaw_ = GetRealLocation().yaw_ - PreviousLocation.yaw_;
	//for testing
	//cout << "Del Yaw: " << delta.yaw_ << endl;
	PreviousLocation = GetRealLocation();

	// Get laser scan
	float* laserScan = GetLaserScan();
	// Update the particles
	lm -> UpdateParticles(delta, laserScan);
	delete[] laserScan;

	//------- indication for the simulator
	if (ConfigurationManager::DEBUG_PARTICLES) {
		cout << "Estimate position of the robot in the real world: " << endl;
		this->lm->GetEstimateLocation().Print();

		cout << "Position of the robot in the real world: " << endl;
		PrintRealPosition();

		float maxWrong;
		if (abs(GetRealLocation().x_ - this->lm->GetEstimateLocation().x_)
				> abs(GetRealLocation().y_ - this->lm->GetEstimateLocation().y_)) {
			maxWrong = abs(GetRealLocation().x_
					- this->lm->GetEstimateLocation().x_);
		} else {
			maxWrong = abs(GetRealLocation().y_
					- this->lm->GetEstimateLocation().y_);
		}

		if (maxDevation < maxWrong) {
			maxDevation = maxWrong;
		}

		cout << "Wrong: " << maxWrong << " ,Max: " << maxDevation << endl;
	}
}

float* Robot::GetLaserScan() {
	float* laserScan = new float[ConfigurationManager::GetLaserSamples()];
	for (int i = 0; i < ConfigurationManager::GetLaserSamples(); i++) {
		laserScan[i] = GetLaser(i);
	}

	return laserScan;
}

float Robot::GetLaser(int index) {
	return (MathUtils::FixByBoundaries(lp -> GetRange(index),
			ConfigurationManager::GetLaserRangeMinMeter(),
			ConfigurationManager::GetLaserRangeMaxMeter()));
}

void Robot::PrintRealPosition() {
	GetRealLocation().Print();
}
Location Robot::GetRealLocation() {
	return CurrentLocation;
}

Location Robot::GetRealLocationForGrid() {
	Location l1 = Location(GetRealLocation().x_, GetRealLocation().y_,
			GetRealLocation().yaw_);
	Point p = MathUtils::RealToGrid(l1);
	Location l2 = Location(p.x_, p.y_, GetRealLocation().yaw_);
	return l2;
}

Location Robot::GetEstimateLocationForGrid() {
	Location l1 = GetEstimateLocation();
	Point p = MathUtils::RealToGrid(l1);
	Location l2 = Location(p.x_, p.y_, l1.yaw_);
	return l2;
}

Location Robot::GetEstimateLocation() {
	Location l = lm->GetEstimateLocation();//{lm->GetEstimateLocation().x_, lm->GetEstimateLocation().y_, lm->GetEstimateLocation().yaw_};
	return l;
}

void Robot::PrintEstimatePosition() {
	Location l = Location(GetEstimateLocation().x_, (-1)
			* GetEstimateLocation().y_, GetEstimateLocation().yaw_);
	l.Print();
}

Robot::~Robot() {
	delete lp;
	delete pp;
	delete lm;
	delete pc;
}

