/*
 * WaypointManager.cpp
 */

#include "WaypointManager.h"
#include <cmath>
#include <vector>
#include <queue>
#include <map>
#include "Map.h"
#include "ConfigurationManager.h"

namespace MapUtilities {

namespace {

struct AStarCell {
	Point current_location_;
	double priority;
};

class CompareAStarCells {
public:
	bool operator()(const AStarCell& lhs, const AStarCell& rhs) {
		// Return true if lhs is ordered before rhs
		return (lhs.priority > rhs.priority);
	}
};
}

static double heruistic_function(const Point current_location);
static int GetCellNeighbours(std::vector<AStarCell>& cell_neighbours,
							  AStarCell& current_cell,
							  Map& grid_map);

WaypointManager* WaypointManager::waypoint_manager_ = 0;

WaypointManager::WaypointManager() {
}

WaypointManager::~WaypointManager() {
	waypoint_locations_.clear();
}

WaypointManager* WaypointManager::getInstance() {
	if (!waypoint_manager_) {
		waypoint_manager_ = new WaypointManager();
	}

	return waypoint_manager_;
}

void WaypointManager::Finalize() {
	if (waypoint_manager_) {
		delete waypoint_manager_;
		waypoint_manager_ = 0;
	}
}

void WaypointManager::FindPath(Map& grid_map, const Point start_point, const Point end_point) {

	std::priority_queue<AStarCell, std::vector<AStarCell>, CompareAStarCells> a_star_priority_queue;
	// Vector of aStar cell : contain location point and priority
	std::vector<AStarCell> cell_neighbours;
	int neightbours_count;
	std::map<const Point, Point> a_star_came_from_found_path;
	//
	std::map<const Point, int> cost_so_far;

	AStarCell start_cell;
	start_cell.current_location_ = Point(start_point.x_, start_point.y_);
	start_cell.priority = 0;

	// Put the start location in the priority queue
	a_star_priority_queue.push(start_cell);
	a_star_came_from_found_path[start_cell.current_location_] = start_cell.current_location_;
	cost_so_far[start_cell.current_location_] = 0;

	// As long as the priority queue isn't empty
	while (!a_star_priority_queue.empty()) {
		// Get the cell on the top of the queue
		AStarCell current_cell = a_star_priority_queue.top();
		a_star_priority_queue.pop();

		if (current_cell.current_location_ == end_point) {
			break;
		}

		cell_neighbours.clear();
		// Get the number of neighbors and the neighbors to the vector
		neightbours_count = GetCellNeighbours(cell_neighbours, current_cell, grid_map);
		// Go over all the neighbors
		for (int current_neighbour = 0; current_neighbour < neightbours_count; current_neighbour++) {
			// Calculate the current cost: the cost untill this location + the cost of the cell
			int new_cost_so_far =
					cost_so_far[current_cell.current_location_] +
					grid_map[cell_neighbours[current_neighbour].current_location_.y_]
					        [cell_neighbours[current_neighbour].current_location_.x_].cell_cost;
			if (!cost_so_far.count(cell_neighbours[current_neighbour].current_location_) ||
				 new_cost_so_far < cost_so_far[cell_neighbours[current_neighbour].current_location_]) {
				a_star_came_from_found_path[cell_neighbours[current_neighbour].current_location_] =
						current_cell.current_location_;
				cost_so_far[cell_neighbours[current_neighbour].current_location_] = new_cost_so_far;
				// Set the new priority as the new cost + the airline distance from the current location to the end location
				double new_priority =
						new_cost_so_far +
						heruistic_function(cell_neighbours[current_neighbour].current_location_);
				cell_neighbours[current_neighbour].priority = new_priority;
				a_star_priority_queue.push(cell_neighbours[current_neighbour]);
			}
		}
	}

	std::vector<Point> a_star_cells;
	Point current_location_in_path = end_point;
	int a_star_path_cell_count = 0;
	// Set the end location as the last waypoint
	grid_map[current_location_in_path.y_][current_location_in_path.x_].cell_state = GridCellState::WAY_POINT;
	// Add the end location to the a star vector
	a_star_cells.insert(a_star_cells.begin(), current_location_in_path);
	current_location_in_path = a_star_came_from_found_path[current_location_in_path];
	a_star_path_cell_count++;

	while (current_location_in_path != start_point) {
		grid_map[current_location_in_path.y_][current_location_in_path.x_].cell_state =
				GridCellState::A_STAR_PATH;
		a_star_cells.insert(a_star_cells.begin(), current_location_in_path);
		current_location_in_path = a_star_came_from_found_path[current_location_in_path];
		a_star_path_cell_count++;
	}

	// Add the start point itself as waypoint
	grid_map[current_location_in_path.y_][current_location_in_path.x_].cell_state =
					GridCellState::WAY_POINT;
	a_star_cells.insert(a_star_cells.begin(), current_location_in_path);
			current_location_in_path = a_star_came_from_found_path[current_location_in_path];
			a_star_path_cell_count++;

	const int minimum_cells_per_waypoint = ConfigurationManager::GetCellsPerWaypoint();
	int cells_from_last_waypoint = 0;
	int delta_x = 0;
	int delta_y = 0;
	int previous_delta_x = 0;
	int previous_delta_y = 0;
	for (int i = 1; i < a_star_path_cell_count; i++) {
		delta_x = a_star_cells[i].x_ - a_star_cells[i - 1].x_;
		delta_y = a_star_cells[i].y_ - a_star_cells[i - 1].y_;
		cells_from_last_waypoint++;

		if ((delta_x != previous_delta_x) || (delta_y != previous_delta_y)) {
			grid_map[a_star_cells[i - 1].y_][a_star_cells[i - 1].x_].cell_state =
					GridCellState::WAY_POINT;
			waypoint_locations_.push_back(a_star_cells[i - 1]);
			previous_delta_x = delta_x;
			previous_delta_y = delta_y;
			cells_from_last_waypoint = 0;
		}
		else if (cells_from_last_waypoint == minimum_cells_per_waypoint) {
			grid_map[a_star_cells[i - 1].y_][a_star_cells[i - 1].x_].cell_state =
					GridCellState::WAY_POINT;
			waypoint_locations_.push_back(a_star_cells[i - 1]);
			cells_from_last_waypoint = 0;
		}
	}

	waypoint_locations_.push_back(end_point);
}

// Calculates the distance to the end
double heruistic_function(const Point current_location) {
	int delta_x = current_location.x_ - ConfigurationManager::GetEndLocation().x_;
	int delta_y = current_location.y_ - ConfigurationManager::GetEndLocation().y_;

	return std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

int GetCellNeighbours(std::vector<AStarCell>& cell_neighbours,
					   AStarCell& current_cell,
					   Map& grid_map) {
	// Does not check for boundaries! could get index out of bound error!
	// Since we move inside the grid, it should be okay tho
	AStarCell neighbour_cell;
	int neighbours_count = 0;
	// Get all the neighbors
	for (int row_adjacent = -1; row_adjacent <= 1; row_adjacent++) {
		for (int col_adjacent = -1; col_adjacent <= 1; col_adjacent++) {
			// If the neighbor is free and it's not the current location
			if ((grid_map[current_cell.current_location_.y_ + row_adjacent]
			             [current_cell.current_location_.x_ + col_adjacent].cell_state == GridCellState::FREE) &&
					(row_adjacent != 0 || col_adjacent != 0)) {
				// Add it to the neighbors and increase the counter
				neighbour_cell.current_location_.x_ = current_cell.current_location_.x_ + col_adjacent;
				neighbour_cell.current_location_.y_ = current_cell.current_location_.y_ + row_adjacent;
				cell_neighbours.push_back(neighbour_cell);
				neighbours_count++;
			}
		}
	}

	return neighbours_count;
}
}
