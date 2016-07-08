#include "MathUtils.h"
#include "ConfigurationManager.h"
#include "Structs.h"
#include <math.h>

	float  MathUtils::distance(float  deltaX, float  deltaY) {
		return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
	}

	float  MathUtils::distance(float  x1, float  y1, float  x2, float  y2) {
		return distance(x2 - x1, y2 - y1);
	}

	float  MathUtils::IndexToAngle(int i, int r) {
		return ((float )i*((float )r/ ConfigurationManager::GetLaserSamples()))-r/2.0;
	}

	float  MathUtils::RadianToDegree(float  r) {
		return  ((r) * 180.0 / M_PI);
	}

	float  MathUtils::DegreeToRadian(float  d) {
		return ((d) * M_PI / 180.0);
	}

	float  MathUtils::SampleToDegree(float s)
	{
		return ((s  - (ConfigurationManager::GetLaserSamples() / 2.0))* ConfigurationManager::GetLaserSpace());
	}

    float  MathUtils::DegreeToSample(float d)
	{
    	return ((d / ConfigurationManager::GetLaserSpace()) + (ConfigurationManager::GetLaserSamples() / 2.0));
	}

	float MathUtils::FixByBoundaries(float number, float minValue, float maxValue) {
		if ((number >= minValue) && (number <= maxValue)) {
			return number;
		} else if (number < minValue) {
			return minValue;
		} else if (number > maxValue || number != number) {
			return maxValue;
		}
		return NAN;
	}

	Point MathUtils::RealToGrid(Location& l)
	{
		Point point = Point(l.x_/ConfigurationManager::GetGridResolution(), (-1)* l.y_/ConfigurationManager::GetGridResolution());
		return point;
	}

	// For show the position in the real world (estimate))
	void MathUtils::GridToReal(Point& p)
	{
		p.x_  = p.x_ * ConfigurationManager::GetGridResolution();
		p.y_ = (-1) * p.y_ * ConfigurationManager::GetGridResolution();
	}

	// just for use in GridToMAp
	void MathUtils::RealToMap(Point& p)
	{
		p.x_ = p.x_/ConfigurationManager::GetMapResolution() - ConfigurationManager::GetMapWidth() / 2;
		p.y_ = (p.y_ * (-1)) /ConfigurationManager::GetMapResolution() - ConfigurationManager::GetMapHeight() / 2;
	}

	// For show the position in the map (estimate))
	void MathUtils::GridToMap(Point& p)
	{
		GridToReal(p);
		RealToMap(p);
	}
