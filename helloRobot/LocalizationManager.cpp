#include "LocalizationManager.h"
#include "Particle.h"
#include <libplayerc++/playerc++.h>
#include "ConfigurationManager.h"


using MapUtilities::Map;

//Constructor of objects type of LocalizationManager
LocalizationManager::LocalizationManager(float xRobot,float yRobot, float yawRobot) {
	particles = particlesVec();
	particles.reserve(ConfigurationManager::GetParticleCount());
	// The first location is the start location
	bestLocation = Location(xRobot,yRobot,yawRobot);
	InitParticles();
	if (ConfigurationManager::DEBUG_PARTICLES)
	{
		cout << "---initialize---" << endl;
		PrintParticles();
		cout << "-----------------" << endl;
	}
}


//Method which gives the ability to initialize all particles
void LocalizationManager::InitParticles() {
            srand(time(NULL));
            // At the beginning all the particles are in the radius of the start location
            for (int i=0; i < ConfigurationManager::GetParticleCount(); i++) {
               Particle newP(bestLocation.x_,bestLocation.y_, bestLocation.yaw_,1);
               particles.push_back(newP);
//               cout << newP.GetX() << " " << newP.GetY() << endl;
            }
}

//Method which gives the ability to update all particles which are stored in the vector
void LocalizationManager::UpdateParticles(Location deltaLocation,float laserScan[]) {
	srand(time(NULL));
	particlesVec::iterator pd = particles.begin();
	particlesVec::iterator pEnd = particles.end();
	list<Particle> potentialDecendents;
	bestBelief = ConfigurationManager::GetMinBelief();

	const char* fileName = ConfigurationManager::GetMapFilename();
	Map& grid = Map::InitializeMap(fileName);
	Location particle_location;
	Point particle_point;

	//Update all particles belief and drop the ones who below the thresh variable
	int i = 0;
	if (pd != pEnd) {
		for (; pd!=pEnd; pd++ ) {
//			ConfigurationManager::GetPrintParticles();
			if(ConfigurationManager::DEBUG_PARTICLES)
			{
				cout << "before update" << endl;
			}
			// Update the particle
			pd->Particle::Update(deltaLocation.x_,deltaLocation.y_, deltaLocation.yaw_,laserScan);
			if(ConfigurationManager::DEBUG_PARTICLES)
			{
				cout << "after update" << endl;
			}
			// Print thing
			if (ConfigurationManager::GetPrintParticles() != -1) {
				particle_point = Point(pd->GetX()/ConfigurationManager::GetGridResolution(), (-1)* pd->GetY()/ConfigurationManager::GetGridResolution());
				grid[particle_point.y_][particle_point.x_].cell_state = GridCellState::PARTICALE;
			}

			if (pd->GetBelief() < ConfigurationManager::GetParticleBeliefThresh())
			{
				particles.erase(pd);
			}
			else if (pd->GetBelief() > ConfigurationManager::GetParticleBeliefThresh()) {
				if(particles.size() < ConfigurationManager::GetParticleCount())
				{
					// Add identical instead
					Particle newP(pd->GetX(), pd->GetY(),pd-> GetYaw(),pd->GetBelief());
					particles.push_back(newP);
					//potentialDecendents.push_back(newP);
					i++;
				}
				if (pd -> GetBelief() >= bestBelief)
				{
					bestLocation = pd -> GetLocation();
					bestBelief = pd->GetBelief();
				}
			}
		}
	}
	ConfigurationManager::SetPrintParticles(-1);
	//Resampling all the partices
	if(bestBelief != ConfigurationManager::GetMinBelief())
	{
		potentialDecendents.sort();

		while (particles.size() < ConfigurationManager::GetParticleCount() && !potentialDecendents.empty()) {
			particles.push_back(potentialDecendents.front());
			potentialDecendents.pop_front();
		}
	}

	RefreshBestLocation();
}

void LocalizationManager::PrintParticles() {
	particlesVec::iterator pd = particles.begin();
	particlesVec::iterator pEnd = particles.end();
	int i = 0;
	if (pd != pEnd) {
		cout<<"-----------------------------------"<<endl;
		for (; pd!=pEnd; pd++ ) {
		pd -> Print();
		i++;
		}
		cout<<"-----------------------------------"<<endl;
	}
}

Location LocalizationManager::GetEstimateLocation()
{
    return bestLocation;
}

//Get the average of all the particles with the highest belief
void LocalizationManager::RefreshBestLocation()
{
	particlesVec::iterator pd = particles.begin();
	particlesVec::iterator pEnd = particles.end();
	bestLocation = Location(0.0,0.0,0.0);
	int j = 0 ;

	// Calculate the average location between all the particles that contain the best belief value
	if (pd != pEnd) {
		for (; pd!=pEnd; pd++ ) {
			if(pd -> GetBelief() == bestBelief)
			{
				bestLocation = bestLocation + pd->GetLocation();
				j++;
			}
		}
		bestLocation = bestLocation/j;
	}
}


//Destructor of objects type of LocalizationManager
LocalizationManager::~LocalizationManager() {
}
