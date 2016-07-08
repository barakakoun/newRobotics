/*
 * main.cpp
 *
 */

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include "Map.h"
#include "WaypointManager.h"
#include "ConfigurationManager.h"
#include "lodepng.h"
#include "Robot.h"
#include "MathUtils.h"

#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

using namespace std;
using MapUtilities::Map;
using MapUtilities::WaypointManager;

//static void debug_print(Map& grid); or daniel
static void debug_print(Map& grid, const char* debug_filename); // or daniel
static double GetAngleForWaypoint(const Point to, const Point from);
static void TurnRobotToWaypoint(Robot& r, const double angle);
static bool WalkToWaypoint(Robot& r, Map& grid);
static void WalkFromObstacle(Robot& r);

//Act as the manager, the path plan is determined in the map and changing throw the running
int main(int argc, char** argv)
{
	cout << "Started Running" << endl;

	// Init configuration
	ConfigurationManager::InitConfigurationManager();


	// Perfrom required calculation on the map and find A* start path to goal
	const char* fileName = ConfigurationManager::GetMapFilename();
	const Point start_location = ConfigurationManager::GetStartLocation();
	const Point end_location = ConfigurationManager::GetEndLocation();
	Map& grid = Map::InitializeMap(fileName);
	WaypointManager& wp_manager = *(WaypointManager::getInstance());
	debug_print(grid, "debug.png");

	// Find the A* path
	wp_manager.FindPath(grid, start_location, end_location);

	debug_print(grid, "FinalGrid.png");

	char* robotPort;

	if (ConfigurationManager::REAL_WORLD) {
		robotPort = "10.10.245.63";
	} else {
		robotPort = "localhost";
	}

	Robot r(robotPort, 6665);

	//For testing
	//r.GetRealLocation().Print();

	// Print all chosen way point
	Point p;
	if (ConfigurationManager::DEBUG_BEHAIVOURS) {

		for (int waypoint = 1; waypoint < wp_manager.WaypointCount(); waypoint++) {
			p = Point(wp_manager[waypoint].x_, wp_manager[waypoint].y_);

			cout << "Way Point: " << waypoint << " (" << p.x_ << "," << p.y_ << ")" << endl;

		}
	}

	Point current_location;
	Location l;
	// Run over the waypoints
	// Init the waypoint counter
	int waypoint = 1;
	// As long as we didn't reach the last waypoint
	while (waypoint < wp_manager.WaypointCount()) {
//		ConfigurationManager::SetPrintParticles(-1);
		ConfigurationManager::SetPrintParticles(waypoint);

		//Print the robot estimate position in the real resolution
		cout << "Barak: " << endl;
		r.PrintEstimatePosition();

		stringstream ss;
		ss << "waypointPrint/waypoint_" << waypoint << ".png";
		debug_print(grid, ss.str().c_str());

		p = Point(wp_manager[waypoint - 1].x_, wp_manager[waypoint - 1].y_);
		grid[p.y_][p.x_].cell_state = GridCellState::A_STAR_PATH;

		bool arrivedToWaypoint = false;
		// As long as didn't arrived to a waypoint
		while (!arrivedToWaypoint) {
			if (ConfigurationManager::USE_PARTICLES) {
				// Get the estimate location and set the robot position properly
				Location estimated = r.GetEstimateLocationForGrid();
				current_location.x_ = estimated.x_;
				current_location.y_ = estimated.y_;
			} else {
				// If we assume that the robot is exactly where its supposed to be
				current_location.x_ = r.GetRealLocationForGrid().x_;
				current_location.y_ = r.GetRealLocationForGrid().y_;
			}
			// In debug run, print the current location and the next way point to arrive
			if (ConfigurationManager::DEBUG_BEHAIVOURS) {
				cout << "From: (" << current_location.x_ << ","
						<< current_location.y_ << ") To: ("
						<< wp_manager[waypoint].x_ << ","
						<< wp_manager[waypoint].y_ << ")" << endl;
			}

			// Get the angle require for the robot to spin in order to arrive to the waypoint
			// Calculate the moving angle between two points
			double angle = GetAngleForWaypoint(wp_manager[waypoint], current_location);
			// Turn the robot with the found angle
			TurnRobotToWaypoint(r, angle);

			arrivedToWaypoint = WalkToWaypoint(r, grid);

			// Initialize the estimate location by the best location found with particles strategy
			if (ConfigurationManager::USE_PARTICLES) {
				Location estimated = r.GetEstimateLocationForGrid();
				current_location = Point(floor((estimated.x_)), floor(estimated.y_));
			} else {
				current_location = Point(
						floor((r.GetRealLocationForGrid().x_)), floor(
								r.GetRealLocationForGrid().y_));
			}

			// Advance required waypoint to closest not yet reached waypoint
			int advanced_waypoint = 1;

			// Calculate distance to current wanted waypoint
			int previous_waypoint_distance = MathUtils::distance(current_location.x_,
																 current_location.y_,
																 wp_manager[waypoint].x_,
																 wp_manager[waypoint].y_);
			// Calculate distance to the next!! wanted waypoint
			int waypoint_distance = MathUtils::distance(current_location.x_,
														current_location.y_,
														wp_manager[waypoint + advanced_waypoint].x_,
														wp_manager[waypoint + advanced_waypoint].y_);

			// כל עוד המרחק מהרובוט לנקודה הבאה קטן מהמרחק של הרובוט לנקודה הנוכחית (אליה צריך הרובוט להגיע), קבע את הנקודה הבאה כנקודה הנוכחית
			// מה שבעצם עושים זה מוצאים את הנקודה בהמשך המסלול אליה הרובוט הכי קרוב (אבל אם הנקודה הנוכחית יותר קרובה מהנקודה אחריה, זה מספיק לנו ונעצור)
			while (waypoint_distance <= previous_waypoint_distance) {
				p = Point(wp_manager[waypoint + advanced_waypoint - 1].x_, wp_manager[waypoint + advanced_waypoint - 1].y_);
				grid[p.y_][p.x_].cell_state = GridCellState::A_STAR_PATH;
				previous_waypoint_distance = waypoint_distance;
				advanced_waypoint++;
				waypoint_distance = MathUtils::distance(current_location.x_,
														current_location.y_,
														wp_manager[waypoint + advanced_waypoint].x_,
														wp_manager[waypoint + advanced_waypoint].y_);
			}

			// עדכן את הנקודה הנוכחית אליה אנו רוצים להגיע לנקודה הקרובה ביותר, כפי שחושב בלולאה לעיל
			waypoint += advanced_waypoint - 1;
		}
		// Increase the waypoint
		waypoint++;
	}

	// Last print
	stringstream ss;
	ss << "waypointPrint/waypoint_" << waypoint << ".png";
	debug_print(grid, ss.str().c_str());

	// Free allocated memory
	Map::Finalize();
	WaypointManager::Finalize();
	cout << "Finished Running" << endl;

	return 0;
}

void debug_print(Map& grid, const char* debug_filename) {
	//const char* debug_filename = "FinalGrid.png";
	const int h = grid.GridHeight();
	const int w = grid.GridWidth();
	std::vector<unsigned char> debug_image;

	debug_image.resize(h*w*4);

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			switch (grid[i][j].cell_state) {
				case (GridCellState::FREE): {
					debug_image[i * w * 4 + j * 4] = 255;
					debug_image[i * w * 4 + j * 4 + 1] = 255;
					debug_image[i * w * 4 + j * 4 + 2] = 255;
					break;
				}
				case (GridCellState::OBSTACLE): {
					debug_image[i * w * 4 + j * 4] = 0;
					debug_image[i * w * 4 + j * 4 + 1] = 0;
					debug_image[i * w * 4 + j * 4 + 2] = 0;
					break;
				}
				case (GridCellState::BLOWED): {
					debug_image[i * w * 4 + j * 4] = 145;
					debug_image[i * w * 4 + j * 4 + 1] = 145;
					debug_image[i * w * 4 + j * 4 + 2] = 145;
					break;
				}
				case (GridCellState::A_STAR_PATH): {
					//								debug_image[i * w * 4 + j * 4] = 255;
					//								debug_image[i * w * 4 + j * 4 + 1] = 36;
					//								debug_image[i * w * 4 + j * 4 + 2] = 233;

					debug_image[i * w * 4 + j * 4] = 255;
					debug_image[i * w * 4 + j * 4 + 1] = 255;
					debug_image[i * w * 4 + j * 4 + 2] = 255;
					break;
				}
				case (GridCellState::WAY_POINT): {
					debug_image[i * w * 4 + j * 4] = 89;
					debug_image[i * w * 4 + j * 4 + 1] = 202;
					debug_image[i * w * 4 + j * 4 + 2] = 171;
					break;
				}
					//or daniel
				case (GridCellState::PARTICALE): {
					debug_image[i * w * 4 + j * 4] = 255;
					debug_image[i * w * 4 + j * 4 + 1] = 0;
					debug_image[i * w * 4 + j * 4 + 2] = 0;
					break;
				}
				default: {
					break;
				}
			}

			// Alpha
			debug_image[i * w * 4 + j * 4 + 3] = 255;
		}
	}

	unsigned error = lodepng::encode(debug_filename, debug_image, w, h);

	if (error)
			std::cout << "encoder error " << error << ": "
					<< lodepng_error_text(error) << std::endl;
}



double GetAngleForWaypoint(const Point to, const Point from) {
	// Get the distance to the next waypoint
	int del_x = to.x_ - from.x_;
	int del_y = to.y_ - from.y_;
	// Calculate arc tangent with two deltas of the triangle
	double angle = atan2(-del_y, del_x);

	// If the angle if negative, add 360 degrees
	if (angle < 0)
	{
		angle += (2 * M_PI);
	}

	return angle;
}

void TurnRobotToWaypoint(Robot& r, const double angle) {
	// The mistake in the robot
	static const double deg_miss = (2.0 * M_PI / 180.0); // 0.0349
//	static const double deg_miss = 0.0;
	double robot_angle;

	// Get the robot's yaw
	if (ConfigurationManager::USE_PARTICLES) {
		robot_angle = r.GetEstimateLocationForGrid().yaw_;
	}
	else {
		robot_angle = r.GetRealLocationForGrid().yaw_;
		robot_angle = ((robot_angle > 0) ? robot_angle : M_PI + M_PI + robot_angle);
	}

	// If the robot need to move with clockwise
	// Means, the current angle is bigger than the walking angle
	if (((robot_angle > angle) && (robot_angle - angle < M_PI)) ||
			((robot_angle < angle) && (angle - robot_angle > M_PI)))  {
		// While we havn't reach the desired walking angel yet
		// Keep turn the robot clockwise
		while (angle < robot_angle - deg_miss) {
			// Turn the robot
			r.SetSpeed(0.0, ConfigurationManager::GetTurnRightSpeed());
			// Refresh the particles
			r.Refresh();

			if (ConfigurationManager::USE_PARTICLES) {
				robot_angle = r.GetEstimateLocationForGrid().yaw_;
			}
			else {
				robot_angle = r.GetRealLocationForGrid().yaw_;
				robot_angle = ((robot_angle > 0) ? robot_angle : M_PI + M_PI + robot_angle);
			}
		}
	}
	else  {// turn against clockwise
		while (angle > robot_angle + deg_miss) {
			// Turn the robot
			r.SetSpeed(0.0, ConfigurationManager::GetTurnLeftSpeed());
			// Refresh the particles
			r.Refresh();

			if (ConfigurationManager::USE_PARTICLES) {
				robot_angle = r.GetEstimateLocationForGrid().yaw_;
			}
			else {
				robot_angle = r.GetRealLocationForGrid().yaw_;
				robot_angle = ((robot_angle > 0) ? robot_angle : M_PI + M_PI + robot_angle);
			}
		}
	}

	r.SetSpeed(0.0, 0.0);
	// Refresh the particles
	r.Refresh();
}

bool WalkToWaypoint(Robot& r, Map& grid) {
	bool arrivedToWayoint = false;
	bool on_waypoint = false;
	bool on_a_star = false;
	bool obstacle_found = false;

	r.SetSpeed(ConfigurationManager::GetMoveForawrdSpeed(), 0);
	Point current_location;

	// Some minor movements in new selected path
	for (int  i = 0; i < ConfigurationManager::GetMinorMovementLoops(); i++) {
		r.Refresh();
	}

	Point p;
	while (true) {
		r.Refresh();
		if (ConfigurationManager::USE_PARTICLES) {
			Location estimated = r.GetEstimateLocationForGrid();
			current_location = Point(floor((estimated.x_)), floor(estimated.y_));
		}
		else {
			current_location = Point(floor((r.GetRealLocationForGrid().x_)), floor(r.GetRealLocationForGrid().y_ ));
		}
		if(ConfigurationManager::DEBUG_BEHAIVOURS)
		{
		cout << "(" << current_location.x_
			 << "," << current_location.y_
			 << ")" << endl;
		}

		// Check if arrived to the waypoint in radius of 1
		for (int  i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				p = Point(current_location.x_ + j, current_location.y_ + i);
				on_waypoint |=
						(grid[p.y_][p.x_].cell_state ==
								GridCellState::WAY_POINT);
			}
		}

		// If arrived, break
		if (on_waypoint) {
			arrivedToWayoint = true;
			break;
		}

		// If didn't arrive check if obstacle found ahead - If in the radius ahead the laser ray is smaller then 0.3m
		for (int i = ConfigurationManager::GetObstacleStartIndex(); i < ConfigurationManager::GetObstacleEndIndex(); i++) {
			obstacle_found |= (r.GetLaser(i) < ConfigurationManager::GetObstacleAheadDistace());
		}

		// If obstacle found, avoid it
		if (obstacle_found) {
//			if(ConfigurationManager::DEBUG_BEHAIVOURS)
//			{
			cout << "Obtstacle found ahead" << endl;
//			}
			WalkFromObstacle(r);
			break;
		}

		// Check if in the A* path
		on_a_star = false;
		for (int  i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				p = Point(current_location.x_ + j, current_location.y_ + i);
				on_a_star |=
						(grid[p.y_][p.x_].cell_state ==
								GridCellState::A_STAR_PATH);
			}
		}

		if (!on_a_star) {
//			if(ConfigurationManager::DEBUG_BEHAIVOURS)
//			{
			cout << "lost A* path" << endl;
//			}
			break;
		}
	}

	r.SetSpeed(0.0, 0.0);
	return arrivedToWayoint;
}

void WalkFromObstacle(Robot& r) {

	r.SetSpeed(0.0, 0.0);
	r.Refresh();
	int density_right = 0;
	int density_left = 0;

	// Check what is more clear - right or left - for every obstacle in ray the direction gets a point to the counter
	for (int i = 0; i < ConfigurationManager::GetMiddleLaserIndex(); i++) {
		// Left side
		if (r.GetLaser(i) < ConfigurationManager::GetObstacleAheadDistace()) {
			density_right++;
		}
		// Right side
		if (r.GetLaser(ConfigurationManager::GetLaserSamples() - i) < ConfigurationManager::GetObstacleAheadDistace()) {
			density_left++;
		}
	}

	// Turn to the cleanest side
	if (density_right > density_left) {
		r.SetSpeed(0.0, ConfigurationManager::GetTurnRightSpeed());
	}
	else {
		r.SetSpeed(0.0, ConfigurationManager::GetTurnLeftSpeed());
	}

	// Keep turn in place until there is no obstacle ahead
	while (true) {
		bool obstacle_found = false;
		r.Refresh();
		for (int i = 0; i < ConfigurationManager::GetLastLaserIndex(); i++) {
			obstacle_found |= (r.GetLaser(i) < ConfigurationManager::GetObstacleAheadDistace());
		}

		if (!obstacle_found) {
			break;
		}
	}

	// If there's no more obstacle ahead, stop turning and continue moving forward for a bit
	r.SetSpeed(ConfigurationManager::GetMoveForawrdSpeed(), 0);

	// Some minor movements in new selected path
	for (int  i = 0; i < ConfigurationManager::GetMinorMovementLoops(); i++) {
		r.Refresh();
	}
	// Stop walking!
	r.SetSpeed(0.0, 0.0);
}

