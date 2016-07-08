/*
 * ConfigurationManager.h
 */

#ifndef CONFIGURATIONMANAGER_H_
#define CONFIGURATIONMANAGER_H_

#include <fstream>
#include <math.h>
#include "Structs.h"

class ConfigurationManager {
public:
	virtual ~ConfigurationManager();

	static void InitConfigurationManager();

	/* GENERL*/
	// Globals determining logic and debug
	static const bool  REAL_WORLD;
	static const bool  DEBUG_PARTICLES;
	static const bool  DEBUG_BEHAIVOURS;
	static const bool  USE_PARTICLES;

	/* MAP */
		static float GetPixelWidth();
		static float GetPixelHeight();
		static float GetPixelResolution();
		static float GetGridResolution();
		static float GetRealWidth();
		static float GetRealHeight();
		static float GetGridWidth();
		static float GetGridHeight();
		static float GetMapResolution();
		static float GetMapWidth();
		static float GetMapHeight();

		/* Map and Waypoint classes */
		static char* GetMapFilename();
		static double GetCentimetersPerPixel();
		static int GetGridCentimetersSize();
		static Point GetStartLocation();
		static Point GetEndLocation();
		static double GetRobotSize();
		static int GetCellsPerWaypoint();

		/* PARTICLE FILTERING */
		static float GetMaxBelief();
		static float GetDefaultBelief();
		static float GetMinBelief();
		static float GetDegreeDevation();
		static float GetMovementDevation();
		static float GetParticleBeliefThresh();
		static unsigned GetParticleCount();
		static float GetNormalizatioFactor();
		static float GetDistanceToTurn();
		static float GetMaxAccurateMovementM();
		static float GetMaxAccurateRotationDegree();
		static float GetNormalAccurateRotationDegree();
		static float GetOpenPathRange();
		static Location GetWrongLocation();
		static int GetPrintParticles();
		static void SetPrintParticles(int newValue);

		/* LASER */
		static double GetLaserRangeMinMeter();
		static double GetLaserRangeMaxMeter();

		static int GetLaserSamples();
		static unsigned GetLaserDegrees();
		static float GetLaserSpace();
		static float GetLaserDegree();
		static unsigned GetSensorFromStart();
		static unsigned GetLaserScanStep();
		static float GetLaserSteps();
		static float GetLaserRangeStep();


	/* ROBOT */
	//BY REAL WORLD PARAMETERS
	static float GetDefaultX();
	static float GetDefaultY();
	static float GetDefaultYaw();
	static unsigned GetMaxDegree();

		/* Behaviour */
		static float GetObstacleAheadDistace();
		static unsigned int GetObstacleStartIndex();
		static unsigned int GetObstacleEndIndex();
		static unsigned int GetMiddleLaserIndex();
		static unsigned int GetLastLaserIndex();
		static float GetMoveForawrdSpeed();
		static float GetTurnLeftSpeed();
		static float GetTurnRightSpeed();
		static unsigned int GetMinorMovementLoops();


//	char* mapPath;
//	int xStartLocation;
//	int yStartLocation;
//	int yawStartLocation;
//	int xTarget;
//	int yTarget;
//	int robotLength;
//	int robotWidth;
//	double mapResolution;
	double gridResolution;

private:
	ConfigurationManager();

	/* MAP */
	//divide to data  by pixel , real, map, grid
	//Picture that given
	static float pixel_width_;
	static float pixel_height_;
	static float pixel_resolution_;
	static float grid_resolution_;
	static float real_width_;
	static float real_height_;
	static float grid_width_;
	static float grid_height_;
	static float map_resolution_;
	static float map_width_;
	static float map_height_;

	/* Map and Waypoint classes */
	static char* map_filename_;
	static const double centimeters_per_pixel_ = 2.5;
	static const int grid_centimeters_size_ = 10;
	static Point start_location_;
	static Point end_location_;
	static double robot_size;
	static const int cells_per_waypoint_ = 5;

	/* PARTICLE FILTERING */
		static float max_belief_;
		static float default_belief_;
		static float min_belief_;
		static float degree_devation_;
		static float movement_devation_;
		static float particle_belief_thresh_;
		static unsigned particles_count_;
		static float normalization_factor_;
		static float distance_to_turn_;
		static float max_accurate_movement_m_;
		static float max_accurate_rotation_degree_;
		static float normal_accurate_rotation_degree_;
		static float open_path_range_;
		static Location wrong_location_;
		static int print_paticles_;


		/* LASER */
		static double laser_range_min_meter_;
		static double laser_range_max_meter_;

		static int laser_samples_;
		static unsigned laser_degrees_;
		static float laser_space_;
		static float laser_degree_;
		static unsigned sensor_from_start_;
		static unsigned laser_scan_step_;
		static float laser_steps_;
		static float laser_range_step_;


		/* ROBOT */
		//BY REAL WORLD PARAMETERS
		static float default_x_;
		static float default_y_;
		static float default_yaw_;
		static unsigned max_degree_;

		/* Behaviour */
		static float obstacle_ahead_distace_;
		static unsigned int obstacle_start_index_;
		static unsigned int obstacle_end_index_;
		static unsigned int middle_laser_index_;
		static unsigned int last_laser_index_;
		static float move_forawrd_speed_;
		static float turn_left_speed_;
		static float turn_right_speed_;
		static unsigned int minor_movement_loops_;


	ConfigurationManager(const ConfigurationManager& configuration_manager);
	const ConfigurationManager& operator=(const ConfigurationManager& configuration_manager);
};

/* MAP */
inline float ConfigurationManager::GetPixelWidth() {
	return pixel_width_;
}

inline float ConfigurationManager::GetPixelHeight() {
	return pixel_height_;
}

inline float ConfigurationManager::GetPixelResolution() {
	return pixel_resolution_;
}

inline float ConfigurationManager::GetGridResolution() {
	return grid_resolution_; //0.1
}

inline float ConfigurationManager::GetRealWidth() {
	return real_width_;
}

inline float ConfigurationManager::GetRealHeight() {
	return real_height_;
}

inline float ConfigurationManager::GetGridWidth() {
	return grid_width_;
}

inline float ConfigurationManager::GetGridHeight() {
	return grid_height_;
}

inline float ConfigurationManager::GetMapResolution() {
	return map_resolution_;
}

inline float ConfigurationManager::GetMapWidth() {
	return map_width_;
}

inline float ConfigurationManager::GetMapHeight() {
	return map_height_;
}

inline char* ConfigurationManager::GetMapFilename() {
	return map_filename_;
}

inline double ConfigurationManager::GetCentimetersPerPixel() {
	return centimeters_per_pixel_;
}

inline int ConfigurationManager::GetGridCentimetersSize() {
	return grid_centimeters_size_;
}

inline Point ConfigurationManager::GetStartLocation() {
	return start_location_;
}

inline Point ConfigurationManager::GetEndLocation() {
	return end_location_;
}

inline double ConfigurationManager::GetRobotSize() {
	return robot_size;
}

inline int ConfigurationManager::GetCellsPerWaypoint() {
	return cells_per_waypoint_;
}

/* PARTICLE FILTERING */
inline float ConfigurationManager::GetMaxBelief() {
	return max_belief_;
}

inline float ConfigurationManager::GetDefaultBelief() {
	return default_belief_;
}

inline float ConfigurationManager::GetMinBelief() {
	return min_belief_;
}

inline float ConfigurationManager::GetDegreeDevation() {
	return degree_devation_; // 2.0
}

inline float ConfigurationManager::GetMovementDevation() {
	return movement_devation_; // 0.05
}

inline float ConfigurationManager::GetParticleBeliefThresh() {
	return particle_belief_thresh_; // 0.5
}

inline unsigned ConfigurationManager::GetParticleCount() {
	return particles_count_;
}

inline float ConfigurationManager::GetNormalizatioFactor() {
	return normalization_factor_; //1.15
}

inline float ConfigurationManager::GetDistanceToTurn() {
	return distance_to_turn_; //0.5
}

inline float ConfigurationManager::GetMaxAccurateMovementM() {
	return max_accurate_movement_m_;
}

inline float ConfigurationManager::GetMaxAccurateRotationDegree() {
	return max_accurate_rotation_degree_;
}

inline float ConfigurationManager::GetNormalAccurateRotationDegree() {
	return normal_accurate_rotation_degree_;
}

inline float ConfigurationManager::GetOpenPathRange() {
	return open_path_range_; //3.0
}

inline Location ConfigurationManager::GetWrongLocation() {
	return wrong_location_;
}

inline int ConfigurationManager::GetPrintParticles() {
	return print_paticles_;
}
inline void ConfigurationManager::SetPrintParticles(int newValue){
	print_paticles_ = newValue;
}

/* LASER */
inline double ConfigurationManager::GetLaserRangeMinMeter() {
	return laser_range_min_meter_; //0.06
}

inline double ConfigurationManager::GetLaserRangeMaxMeter() {
	return laser_range_max_meter_; //4.095
}

inline int ConfigurationManager::GetLaserSamples() {
	return laser_samples_;
}

inline unsigned ConfigurationManager::GetLaserDegrees() {
	return laser_degrees_;
}

inline float ConfigurationManager::GetLaserSpace() {
	return laser_space_;
}

inline float ConfigurationManager::GetLaserDegree() {
	return laser_degree_;
}

inline unsigned ConfigurationManager::GetSensorFromStart() {
	return sensor_from_start_;
}

inline unsigned ConfigurationManager::GetLaserScanStep() {
	return laser_scan_step_;
}

inline float ConfigurationManager::GetLaserSteps() {
	return laser_steps_;
}

inline float ConfigurationManager::GetLaserRangeStep() {
	return laser_range_step_;
}

/* ROBOT */
//BY REAL WORLD PARAMETERS
inline float ConfigurationManager::GetDefaultX() {
	return default_x_;
}

inline float ConfigurationManager::GetDefaultY() {
	return default_y_;
}

inline float ConfigurationManager::GetDefaultYaw() {
	return default_yaw_;
}

inline unsigned ConfigurationManager::GetMaxDegree() {
	return max_degree_;
}

/* Behaviour */
inline float ConfigurationManager::GetObstacleAheadDistace() {
	return obstacle_ahead_distace_; // 0.3
}

inline unsigned int ConfigurationManager::GetObstacleStartIndex() {
	return obstacle_start_index_; // 208
}

inline unsigned int ConfigurationManager::GetObstacleEndIndex() {
	return obstacle_end_index_; // 458
}

inline unsigned int ConfigurationManager::GetMiddleLaserIndex() {
	return middle_laser_index_;
}

inline unsigned int ConfigurationManager::GetLastLaserIndex() {
	return last_laser_index_;
}

inline float ConfigurationManager::GetMoveForawrdSpeed() {
	return move_forawrd_speed_;
}

inline float ConfigurationManager::GetTurnLeftSpeed() {
	return turn_left_speed_;
}

inline float ConfigurationManager::GetTurnRightSpeed() {
	return turn_right_speed_;
}

inline unsigned int ConfigurationManager::GetMinorMovementLoops() {
	return minor_movement_loops_; //2
}

#endif /* CONFIGURATIONMANAGER_H_ */
