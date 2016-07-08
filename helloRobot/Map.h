/*
 * Map.h
 */

#ifndef MAP_H_
#define MAP_H_

struct GridCell;

namespace MapUtilities {

class Map {
public:
	// Virtual public Ctor
	virtual ~Map();

	// Initialization and finalize for singleton pattern
	static Map& InitializeMap(const char* filename);
	static void Finalize();

	// Easy accessor to the grid cell
	GridCell* operator[](const int index) const;
	unsigned int GridHeight() const;
	unsigned int GridWidth() const;

private:
	// Using encapsulation, only the Ctor and decode will be shown
	// All other functions and methods will be done by static local functions.
	Map();
	bool DecodePngFileAndMakeGrid(const char* filename);

	// Data Members
	unsigned int height_;
	unsigned int width_;
	GridCell** grid_map_;
	static Map* map_instance_;

	// As part of the singleton, disable copy and assignment
	Map(const Map&);
	Map& operator=(const Map&);
};

inline GridCell* Map::operator[](const int index) const {
	return grid_map_[index];
}

inline unsigned int Map::GridHeight() const {
	return height_;
}

inline unsigned int Map::GridWidth() const {
	return width_;
}

}
#endif /* MAP_H_ */
