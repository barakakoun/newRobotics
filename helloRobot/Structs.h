/*
 * Structs.h
 *
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <iostream>

using namespace std;

struct Point {
	unsigned int x_; // Rows = Heights
	unsigned int y_; // Cols = Width

	Point(){
		x_ = 0;
		y_ = 0;
	}

	Point(unsigned int x, unsigned int y) {
		x_ = x;
		y_ = y;
	}

	bool operator==(const Point& rhs) const {
		return ((this->x_ == rhs.x_) && (this->y_ == rhs.y_));
	}

	bool operator!=(const Point& rhs) const {
		return (!(this->operator ==(rhs)));
	}


	bool operator<(const Point& rhs) const {
		return ((this->x_ < rhs.x_) || ((this->x_ == rhs.x_) && (this->y_ < rhs.y_)));
	}

  void Print()
  {
      cout << "X:" << this->x_ << " Y:" << this->y_ << endl;
  }
};

struct Location {
	float x_;
	float y_;
	float yaw_;

	Location(){
		x_ = 0;
		y_ = 0;
		yaw_ = 0;
	}

	Location(double dX, double dY, double dYaw) {
		x_ = dX;
		y_ = dY;
		yaw_ = dYaw;
	}

         void Print()
        {
            cout << "X:" << this->x_ << " Y:" << this->y_ << " YAW:" << this->yaw_ << endl;
        }


    inline Location operator+(Location a) {
        return Location(a.x_+x_,a.y_+y_, a.yaw_ + yaw_);
    }

    inline Location operator-(Location a) {
            return Location(a.x_-x_,a.y_-y_, a.yaw_-yaw_);
        }


    inline Location operator/(double a) {
        return Location(x_/a,y_/a,yaw_/a);
    }

    inline bool operator!=(Location a) {
           return (!((x_ == a.x_)&&(y_ == a.y_)&&(yaw_ == a.yaw_)));
       }

    inline void operator=(Location a) {
               x_ = a.x_;
               y_ = a.y_;
               yaw_ = a.yaw_;
           }

};

namespace GridCellState {
enum CellState {
	FREE,
	OBSTACLE,
	A_STAR_PATH,
	WAY_POINT,
	BLOWED,
	PARTICALE
};
}

#pragma pack(1)
struct GridCell {
	GridCellState::CellState cell_state : 3;
	unsigned int cell_cost : 5;
};
#pragma pack(0)

#endif /* STRUCTS_H_ */
