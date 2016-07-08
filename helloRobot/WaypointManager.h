/*
 * WaypointManager.h
 *
 */

#ifndef WAYPOINTMANAGER_H_
#define WAYPOINTMANAGER_H_

#include "Structs.h"
#include <vector>

namespace MapUtilities {
class Map;

class WaypointManager {
public:
	virtual ~WaypointManager();
	static WaypointManager* getInstance();
	static void Finalize();
	void FindPath(Map& grid_map, const Point start_point, const Point end_point);
	const Point operator[](const int index);
	const int WaypointCount() const;

private:
	WaypointManager();
	std::vector<Point> waypoint_locations_;
	static WaypointManager* waypoint_manager_;

	WaypointManager(const WaypointManager&);
	WaypointManager& operator=(const WaypointManager&);
};

inline const Point WaypointManager::operator[](const int index) {
	return waypoint_locations_[index];
}

inline const int WaypointManager::WaypointCount() const {
	return waypoint_locations_.size();
}
}
#endif /* WAYPOINTMANAGER_H_ */
