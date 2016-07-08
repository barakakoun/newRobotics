/*
 * ConfigurationManager.cpp
 */

#include "ConfigurationManager.h"

/* GENERL*/
// Globals determining logic and debug
const bool  ConfigurationManager::REAL_WORLD = true;
const bool  ConfigurationManager::DEBUG_PARTICLES = false;
const bool  ConfigurationManager::DEBUG_BEHAIVOURS = false;
const bool  ConfigurationManager::USE_PARTICLES = true;

/* MAP */
//divide to data  by pixel , real, map, grid
//Picture that given
float ConfigurationManager::pixel_width_;
float ConfigurationManager::pixel_height_;
float ConfigurationManager::pixel_resolution_;
float ConfigurationManager::grid_resolution_;
float ConfigurationManager::real_width_;
float ConfigurationManager::real_height_;
float ConfigurationManager::grid_width_;
float ConfigurationManager::grid_height_;
float ConfigurationManager::map_resolution_;
float ConfigurationManager::map_width_;
float ConfigurationManager::map_height_;

/* Map an Waypoint classes */
char* ConfigurationManager::map_filename_;
Point ConfigurationManager::start_location_;
Point ConfigurationManager::end_location_;

/* PARTICLE FILTERING */
float ConfigurationManager::max_belief_;
float ConfigurationManager::default_belief_;
float ConfigurationManager::min_belief_;
float ConfigurationManager::degree_devation_;
float ConfigurationManager::movement_devation_;
float ConfigurationManager::particle_belief_thresh_;  // Tune
unsigned ConfigurationManager::particles_count_;
float ConfigurationManager::normalization_factor_; //Tune, for particle update - belief normlization
float ConfigurationManager::distance_to_turn_;
float ConfigurationManager::max_accurate_movement_m_; // Tune, probBYMOvment (0.5))
float ConfigurationManager::max_accurate_rotation_degree_;  // Tune , probBYMOvment (wrote 57))
float ConfigurationManager::normal_accurate_rotation_degree_; //Tune , wrote(25))
float ConfigurationManager::open_path_range_; // Tune, ditance I want to check
Location ConfigurationManager::wrong_location_;
int ConfigurationManager::print_paticles_;

/* LASER */
double ConfigurationManager::laser_range_min_meter_;
double ConfigurationManager::laser_range_max_meter_;

int ConfigurationManager::laser_samples_;
unsigned ConfigurationManager::laser_degrees_;
float ConfigurationManager::laser_space_;
float ConfigurationManager::laser_degree_; // Giving
unsigned ConfigurationManager::sensor_from_start_; // Giving
unsigned ConfigurationManager::laser_scan_step_; // Tune, to see idf its good
float ConfigurationManager::laser_steps_;
float ConfigurationManager::laser_range_step_;

/* ROBOT */
//BY REAL WORLD PARAMETERS
float ConfigurationManager::default_x_;
float ConfigurationManager::default_y_;
float ConfigurationManager::default_yaw_;
unsigned ConfigurationManager::max_degree_;
double ConfigurationManager::robot_size;

/* Behaviour */
float ConfigurationManager::obstacle_ahead_distace_;
unsigned int ConfigurationManager::obstacle_start_index_;
unsigned int ConfigurationManager::obstacle_end_index_;
unsigned int ConfigurationManager::middle_laser_index_;
unsigned int ConfigurationManager::last_laser_index_;
float ConfigurationManager::move_forawrd_speed_;
float ConfigurationManager::turn_left_speed_;
float ConfigurationManager::turn_right_speed_;
unsigned int ConfigurationManager::minor_movement_loops_;

ConfigurationManager::ConfigurationManager() {

}

void ConfigurationManager::InitConfigurationManager() {
	char file_data[10][100];
	char attribute_name[200];
	ifstream inputFile;
	inputFile.open("parameters.txt", ios::in);
	int counter = 0;

	while (!inputFile.eof()) {
		inputFile.getline(attribute_name, 100, ' ');
		inputFile.getline(file_data[counter], 100, '\n');
		counter++;
	}

	inputFile.close();

	string mapString = file_data[0];
	if (!mapString.empty() && mapString[mapString.size() - 1] == '\r') {
		mapString.erase(mapString.size() - 1);
	}
	map_filename_ = new char[1024];
	strcpy(map_filename_, mapString.c_str());
	string startLocation = file_data[1];
	float xStart = atoi(startLocation.substr(0,startLocation.find_first_of(' ')).c_str());
	startLocation = startLocation.substr(startLocation.find_first_of(' ') + 1);
	float yStart = atoi(startLocation.substr(0,startLocation.find_first_of(' ')).c_str());
	startLocation = startLocation.substr(startLocation.find_first_of(' ') + 1);
	default_yaw_ = atoi(startLocation.c_str());
	default_x_ = xStart/40;
	default_y_ = yStart/-40;
	start_location_ = Point((xStart / 4), (yStart / 4));

	string goal = file_data[2];
	float xTarget = atoi(goal.substr(0, goal.find_first_of(' ')).c_str());
	goal = goal.substr(goal.find_first_of(' ') + 1);
	float yTarget = atoi(goal.c_str());
	end_location_ = Point((xTarget / 4), (yTarget / 4));

	string robotSize = file_data[3];
	double robotLength = atoi(robotSize.substr(0, robotSize.find_first_of(' ')).c_str());
	robotSize = robotSize.substr(robotSize.find_first_of(' ') + 1);
	double robotWidth = atoi(robotSize.c_str());
	robot_size = fmax(robotLength, robotWidth);

	string MapResolutionCM = file_data[4];
	pixel_resolution_ = atof(MapResolutionCM.c_str())/100;
	string GridResolutionCM = file_data[5];
	grid_resolution_ = atof(GridResolutionCM.c_str())/100;

	pixel_width_ = 550;
	pixel_height_ = 380;

	real_width_ = pixel_width_ * pixel_resolution_;
	real_height_ = pixel_height_ * pixel_resolution_;

	grid_width_ = real_width_ / grid_resolution_; // now 550, need to be 137.5
	grid_height_ = real_height_ / grid_resolution_;// now 380, need to be 95
	map_resolution_ = 1;
	map_width_ = real_width_/ map_resolution_;
	map_height_ = real_height_/ map_resolution_ ;

	/* PARTICLE FILTERING */
	max_belief_ = 1.0;
	default_belief_ = max_belief_;
	min_belief_ = 0.0;
	degree_devation_ = 2.0;
	movement_devation_ = 0.05;
	particle_belief_thresh_ = 0.5;  // Tune
	particles_count_ = 100;
	normalization_factor_ = 1.15; //Tune, for particle update - belief normlization
	distance_to_turn_ = 0.5;
	max_accurate_movement_m_ = 0.5; // Tune, probBYMOvment (0.5))
	max_accurate_rotation_degree_ = 57.0;  // Tune , probBYMOvment (wrote 57))
	normal_accurate_rotation_degree_ = max_accurate_rotation_degree_ / 2; //Tune , wrote(25))
	open_path_range_ = 3.0; // Tune, ditance I want to check
	wrong_location_ = Location(-1.0,-1.0,-1.0);
	print_paticles_ = -1;

	/* LASER */
	laser_range_min_meter_ = 0.060;
	laser_range_max_meter_ = 4.095;

	laser_samples_ = 666;
	laser_degrees_ = 240;
	laser_space_ = (float)laser_degrees_/(float)laser_samples_;
	laser_degree_ = 240.0; // Giving
	sensor_from_start_ = 0; // Giving
	laser_scan_step_ = 13; // Tune, to see idf its good
	laser_steps_ = 10 ;
	laser_range_step_ = (open_path_range_ - laser_range_min_meter_)/laser_steps_;

	/* ROBOT */
	//BY REAL WORLD PARAMETERS
	max_degree_ = 360.0;

	/* Behaviour */
	obstacle_ahead_distace_ = 0.3;
	obstacle_start_index_ = 208;
	obstacle_end_index_ = 458;
	middle_laser_index_ = 333;
	last_laser_index_ = 666;
	move_forawrd_speed_ = 0.2;
	turn_left_speed_ = 0.2;
	turn_right_speed_ = -0.2;
	minor_movement_loops_ = 2;
}

ConfigurationManager::~ConfigurationManager() {
	// TODO Auto-generated destructor stub
}
