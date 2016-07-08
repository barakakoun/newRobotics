/*
 * Map.cpp
 */

#include <cstdlib>
#include <cmath>
#include <vector>
#include "Map.h"
#include "lodepng.h"
#include "Structs.h"
#include "ConfigurationManager.h"

using std::vector;

namespace MapUtilities {

static bool* ConvertPngToRawPixels(vector<unsigned char>& png_image,
								  const unsigned int image_height,
								  const unsigned int image_width);
static bool IsRgbaPixelWhite(vector<unsigned char>& png_image,
							 const unsigned int pixel_index);
static GridCell** CreateCompressedGridMap(bool* raw_pixels_image,
		  	  	  	  	  	  	  	  	  const unsigned int image_real_height,
		  	  	  	  	  	  	  	  	  const unsigned int image_real_width,
										  unsigned int& grid_height,
										  unsigned int& grid_width);
static GridCell** InitializeGrid(const unsigned int grid_height,
								 const unsigned int grid_width);
static void DeleteGrid(GridCell** grid_map,
		 	 	 	   const unsigned int grid_height);
static void CopyGrid(GridCell** destination_grid,
					 GridCell** source_grid,
					 const unsigned int grid_height,
					 const unsigned int grid_width,
					 const unsigned int blow_grid_size);
static bool AreaHasBlackPixel(bool* raw_pixels_image,
							  const unsigned int row,
							  const unsigned int col,
							  const unsigned int grid_width,
							  const unsigned int adjancent_height_ceiling,
							  const unsigned int adjancent_width_ceiling);
static void CreateBlowedWeightedGridMap(GridCell** grid_map,
										const unsigned int grid_height,
										const unsigned int grid_width);
static void BlowCurrentCellNeighbours(GridCell** grid_map,
									  const unsigned int current_cell_row,
									  const unsigned int current_cell_col,
									  const int blow_grid_size);
static void WeightCurrentCellBasedOnNeighbours(GridCell** grid_map,
									  	  	   const int current_cell_row,
									  	  	   const int current_cell_col,
									  	  	   const int blow_grid_size);
Map* Map::map_instance_ = 0;

Map::Map() : height_(0), width_(0), grid_map_(0) {
}

Map::~Map() {
	if (grid_map_) {
		DeleteGrid(grid_map_, height_);
	}
}

Map& Map::InitializeMap(const char* filename) {
	if (!map_instance_) {
		map_instance_ = new Map();
		if (!map_instance_->DecodePngFileAndMakeGrid(filename)) {
			map_instance_ = 0;
			exit(1);
		}
	}

	return *map_instance_;
}

void Map::Finalize() {
	if (map_instance_) {
		delete map_instance_;
		map_instance_ = 0;
	}
}

bool Map::DecodePngFileAndMakeGrid(const char* filename) {
	vector<unsigned char>original_png_image;
	unsigned int original_image_width;
	unsigned int original_image_height;
	bool error_status =
			lodepng::decode(original_png_image, original_image_width,
							original_image_height, filename);

	if (!error_status) {
		bool* image_pixels_raw_data =
				ConvertPngToRawPixels(original_png_image,
							  	  	  original_image_height,
							  	  	  original_image_width);
		grid_map_ =
				CreateCompressedGridMap(image_pixels_raw_data,
									original_image_height,
									original_image_width,
									height_,
									width_);

		delete[] image_pixels_raw_data;

		CreateBlowedWeightedGridMap(grid_map_, height_, width_);
	}

	return (!error_status);
}

bool* ConvertPngToRawPixels(vector<unsigned char>& png_image,
								   const unsigned int image_height,
								   const unsigned int image_width) {
	bool* raw_pixels = new bool[image_width * image_height];

	for (unsigned int row = 0; row < image_height; row++) {
		for (unsigned int col = 0; col < image_width; col++) {
			unsigned int currentPixel = ((row * image_width) + col) * 4;
			if (IsRgbaPixelWhite(png_image, currentPixel)) {
				raw_pixels[(row * image_width) + col] = false;
			}
			else {
				raw_pixels[(row * image_width) + col] = true;
			}
		}
	}

	return raw_pixels;
}

bool IsRgbaPixelWhite(vector<unsigned char>& png_image,
							 const unsigned int pixel_index) {
	return ((png_image[pixel_index] != 0) ||  // R
			(png_image[pixel_index + 1] != 0) || // G
			(png_image[pixel_index + 2] != 0)); // B
}

GridCell** CreateCompressedGridMap(bool* raw_pixels_image,
										  const unsigned int image_real_height,
										  const unsigned int image_real_width,
										  unsigned int& grid_height,
										  unsigned int& grid_width) {
	const unsigned int resulotion_ratio =
			ConfigurationManager::GetGridCentimetersSize() / ConfigurationManager::GetCentimetersPerPixel();


	const bool height_has_reminder = ((image_real_height % resulotion_ratio) != 0);
	const bool width_has_reminder = ((image_real_width % resulotion_ratio) != 0);
	const int height_reminder = image_real_height % resulotion_ratio;
	const int width_reminder = image_real_width % resulotion_ratio;
	grid_height = (image_real_height / resulotion_ratio) + height_has_reminder;
	grid_width = (image_real_width / resulotion_ratio) + width_has_reminder;

	GridCell** grid_map = InitializeGrid(grid_height, grid_width);

	unsigned int row;
	unsigned int col;
	const unsigned height_ceiling = image_real_height - resulotion_ratio;
	const unsigned width_ceiling = image_real_width - resulotion_ratio;
	for (row = 0; row <= height_ceiling; row+=resulotion_ratio) {
		for (col = 0; col <= width_ceiling; col+=resulotion_ratio) {

			if (AreaHasBlackPixel(raw_pixels_image,		// Raw pixels image
								  row, col, image_real_width, // Current row, col and image width
								  resulotion_ratio, 	// Extra height area to check
								  resulotion_ratio)) { 	// Extra width area to check
				grid_map[row / resulotion_ratio][col / resulotion_ratio].cell_state =
						GridCellState::OBSTACLE;
			}
		}

		if (width_has_reminder &&
				AreaHasBlackPixel(raw_pixels_image,
								  row, col, image_real_width,
								  resulotion_ratio,
								  width_reminder)) {
			grid_map[row / resulotion_ratio][col / resulotion_ratio].cell_state =
									GridCellState::OBSTACLE;
		}
	}

	if (height_has_reminder) {
		for (col = 0; col <= width_ceiling; col+=resulotion_ratio) {
			if (AreaHasBlackPixel(raw_pixels_image,		// Raw pixels image
											  row, col, image_real_width, // Current row, col and image width
											  height_reminder, 	// Extra height area to check
											  resulotion_ratio)) { 	// Extra width area to check
				grid_map[row / resulotion_ratio][col / resulotion_ratio].cell_state =
									GridCellState::OBSTACLE;
			}

			if (width_has_reminder &&
					AreaHasBlackPixel(raw_pixels_image,
									  row, col, image_real_width,
									  height_reminder,
									  width_reminder)) {
				grid_map[row / resulotion_ratio][col / resulotion_ratio].cell_state =
										GridCellState::OBSTACLE;
			}
		}
	}

	return grid_map;
}

GridCell** InitializeGrid(const unsigned int grid_height,
								 const unsigned int grid_width) {
	GridCell** grid_map = new GridCell*[grid_height];
	for (unsigned int i = 0; i < grid_height; i++) {
		grid_map[i] = new GridCell[grid_width];
	}

	for (unsigned int row = 0; row < grid_height; row++) {
		for (unsigned int col = 0; col < grid_width; col++) {
			grid_map[row][col].cell_state = GridCellState::FREE;
			grid_map[row][col].cell_cost = 0;
		}
	}

	return grid_map;
}

void DeleteGrid(GridCell** grid_map, const unsigned int grid_height) {
	for (unsigned int i = 0; i < grid_height; i++) {
		delete[] grid_map[i];
	}

	delete[] grid_map;
	grid_map = 0;
}

void CopyGrid(GridCell** destination_grid,
					 GridCell** source_grid,
					 const unsigned int grid_height,
					 const unsigned int grid_width,
					 const unsigned int blow_grid_size) {
	for (unsigned int row = 0; row < grid_height; row++) {
		for (unsigned int col = 0; col < grid_width; col++) {
			destination_grid[row][col] = source_grid[row + blow_grid_size][col + blow_grid_size];
		}
	}
}

bool AreaHasBlackPixel(bool* raw_pixels_image,
							  const unsigned int row,
							  const unsigned int col,
							  const unsigned int grid_width,
							  const unsigned int adjancent_height_ceiling,
							  const unsigned int adjancent_width_ceiling) {
	bool obstacle_found = false;
	for (unsigned int adj_row = 0; adj_row < adjancent_height_ceiling; adj_row++) {
		for (unsigned int adj_col = 0; adj_col < adjancent_width_ceiling; adj_col++) {
			obstacle_found |=  raw_pixels_image[((row + adj_row)* grid_width) + col + (adj_col)];
		}
	}

	return obstacle_found;
}

void CreateBlowedWeightedGridMap(GridCell** grid_map,
						const unsigned int grid_height,
						const unsigned int grid_width) {
	const double robot_size = ConfigurationManager::GetRobotSize();
	const unsigned int grid_cell_cm_size = ConfigurationManager::GetGridCentimetersSize();
	int blow_grid_size = std::ceil((robot_size / (grid_cell_cm_size)) / 2);

	const unsigned int walled_grid_height = grid_height + (2 * blow_grid_size);
	const unsigned int walled_grid_width = grid_width + (2 * blow_grid_size);
	GridCell** temporary_blown_walled_grid = InitializeGrid(walled_grid_height, walled_grid_width);

	for (unsigned int row = 0; row < grid_height; row++) {
		for (unsigned int col = 0; col < grid_width; col++) {
			if (grid_map[row][col].cell_state == GridCellState::OBSTACLE) {
					temporary_blown_walled_grid[row + blow_grid_size]
					                            [col + blow_grid_size].cell_state =
					                            		GridCellState::OBSTACLE;
					BlowCurrentCellNeighbours(temporary_blown_walled_grid, row, col, blow_grid_size);
			}
		}
	}

	for (unsigned int row = 0; row < grid_height; row++) {
		for (unsigned int col = 0; col < grid_width; col++) {
			WeightCurrentCellBasedOnNeighbours(temporary_blown_walled_grid, row, col, blow_grid_size);
		}
	}

	CopyGrid(grid_map, temporary_blown_walled_grid, grid_height, grid_width, blow_grid_size);
	DeleteGrid(temporary_blown_walled_grid, walled_grid_height);
}

void BlowCurrentCellNeighbours(GridCell** grid_map,
									  const unsigned int current_cell_row,
									  const unsigned int current_cell_col,
									  const int blow_grid_size) {
	int current_checked_row;
	int current_checked_col;

	for (int blow_row = -blow_grid_size; blow_row <= blow_grid_size; blow_row++) {
		for (int blow_col = -blow_grid_size; blow_col <= blow_grid_size; blow_col++) {
			if (abs(blow_row) + abs(blow_col) < blow_grid_size)  {// Blow a Diamond, and not a square
				current_checked_row = current_cell_row + blow_grid_size + blow_row;
				current_checked_col = current_cell_col + blow_grid_size + blow_col;
				if (grid_map[current_checked_row][current_checked_col].cell_state !=
						GridCellState::OBSTACLE) {
					grid_map[current_checked_row][current_checked_col].cell_state = GridCellState::BLOWED;
				}
			}
		}
	}

}

void WeightCurrentCellBasedOnNeighbours(GridCell** grid_map,
									  	  	   const int current_cell_row,
									  	  	   const int current_cell_col,
									  	  	   const int blow_grid_size) {
	int current_checked_row;
	int current_checked_col;

	for (int blow_row = -blow_grid_size; blow_row <= blow_grid_size; blow_row++) {
		for (int blow_col = -blow_grid_size; blow_col <= blow_grid_size; blow_col++) {
			current_checked_row = current_cell_row + blow_grid_size + blow_row;
			current_checked_col = current_cell_col + blow_grid_size + blow_col;
			if (grid_map[current_checked_row][current_checked_col].cell_state == GridCellState::OBSTACLE) {
				grid_map[current_cell_row][current_cell_col].cell_cost += 1;
			}
			else if (grid_map[current_checked_row][current_checked_col].cell_state == GridCellState::BLOWED) {
				grid_map[current_cell_row + blow_grid_size][current_cell_col + blow_grid_size].cell_cost += 1;
			}
		}
	}
}
}
