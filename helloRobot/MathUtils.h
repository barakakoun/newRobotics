#pragma once

#include <math.h>
#include <float.h>
#include "Structs.h"

class MathUtils {
public:
			static float  distance(float  deltaX, float  deltaY);
			static float  distance(float  x1, float  y1, float  x2, float  y2);
			static float  IndexToAngle(int i, int r);
			static float  RadianToDegree(float  r);
			static float  DegreeToRadian(float  d);
			static float  SampleToDegree(float s);
			static float  DegreeToSample(float d);
			static float FixByBoundaries(float number, float minValue, float maxValue);
			static Point RealToGrid(Location& p);
			static void GridToMap(Point& p);
			static void GridToReal(Point& p);
			static void RealToMap(Point& p);

};
