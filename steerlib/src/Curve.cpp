//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;



Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	if(path.size()>2)
	{
		for(int i=window; i<path.size(); i+=window)
		{
			DrawLib::drawLine(path.at(i-window), path.at(i), curveColor, curveThickness);
		}
	}
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	// After 10 years I decided to implement again the BUBBLESORT!!!! :)
	int n = controlPoints.size();
	for (int i = 0 ; i < n-1; i++)
	{
		for (int j = 0 ; j < n - i - 1; j++)
		{
			if (controlPoints.at(j).time > controlPoints.at(j+1).time) /* For decreasing order use < */
			{
			CurvePoint swap = controlPoints.at(j);
			controlPoints.at(j)   = controlPoints.at(j+1);
			controlPoints.at(j+1) = swap;
			}
		}
	}

	//we have to remove points with the same timestamp in order to avoid teleportation issues
	// removeControlPoints();
	//Alternatively we can  increase the time of the second point with identical time
	adjustControlPoints();
	



	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}
	
	path.push_back(outputPoint);
	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if(controlPoints.size()<=1 )
	{
		std::cerr<<"Robustness check failed!"<<std::endl;
		return false;
	}
	else
	{
		return true;
	}
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	int n = controlPoints.size();
	for(int i = 0; i < n; i++)
	{
		if(time < controlPoints.at(i).time)
		{
			nextPoint = i;
			// std::cout<<"Current Interval: "<<time<< "  Next Point: "<<controlPoints.at(i).time<<std::endl;
			return true;
		}

	}

	std::cerr << "No more control points!!!"<<std::endl;
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	float t;

	unsigned int prev_point_index = nextPoint - 1;
	normalTime = controlPoints.at(prev_point_index).time;
	intervalTime = controlPoints.at(nextPoint).time;
	t = (time - normalTime)/(intervalTime - normalTime);
	// std::cout <<"prev: "<< prev_point_index<<"  next: "<<nextPoint<<std::endl;


	float h1 = 2*pow(t,3.0) - 3*pow(t,2.0) +1;
	float h2 = -2*pow(t,3.0) + 3*pow(t,2.0);
	float h3 = pow(t,3.0) - 2*pow(t,2.0) + t;
	float h4 = pow(t,3.0) - pow(t,2.0);

	// std::cout<<"h1 = "<<h1<<" h2 = "<<h2<<" h3 = "<<h3<<" h4 = "<<h4<<std::endl;

	newPosition = h1*controlPoints.at(prev_point_index).position + h2*controlPoints.at(nextPoint).position + (intervalTime - normalTime)*h3*controlPoints.at(prev_point_index).tangent + (intervalTime - normalTime)*h4*controlPoints.at(nextPoint).tangent;
	// std::cout<<"newPosition: "<< newPosition<<"  time:"<<t<<std::endl;
	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;
	float t;

	int i_minus_one = nextPoint - 2;
	if(i_minus_one < 0)
	{
		i_minus_one = controlPoints.size()-1;
	}
	int i = nextPoint - 1;
	int i_plus_one = nextPoint;
	int i_plus_two = (nextPoint+1)%controlPoints.size();

	// std::cout<<"i-1 = "<<i_minus_one<<" i = "<<i<<" i+1 = "<<i_plus_one<<" i+2 = "<<i_plus_two<<std::endl;
	
	normalTime = controlPoints.at(i).time;
	intervalTime = controlPoints.at(i_plus_one).time;
	t = (time - normalTime)/(intervalTime - normalTime);
	// std::cout<<t<<std::endl;

	Point P1 = Point( 1.0f*controlPoints.at(i).position);
	Point P2 = Point( 0.5f*controlPoints.at(i_plus_one).position + (-0.5f)*controlPoints.at(i_minus_one).position);
	Point P3 = Point( 1.0f*controlPoints.at(i_minus_one).position + (-5.0f*0.5f)*controlPoints.at(i).position + (2.0f) * controlPoints.at(i_plus_one).position + (-0.5f)*controlPoints.at(i_plus_two).position);
	Point P4 = Point( (-0.5f)*controlPoints.at(i_minus_one).position + (3.0f*0.5f) * controlPoints.at(i).position + (-3.0f*0.5f) * controlPoints.at(i_plus_one).position + (0.5f)*controlPoints.at(i_plus_two).position);

	newPosition = P1 + (P2 * t) + (P3 * t * t) + (P4 * t * t * t);

	return newPosition;
}

void Curve::removeControlPoints()
{
	std::vector<int> indices;
	for(int i=0; i<controlPoints.size()-2; i++)
	{
		std::cout<<controlPoints.at(i).time<<std::endl;
		if(controlPoints.at(i).time == controlPoints.at(i+1).time)
		{
			std::cout<<" i = "<<i<<" i+1 = "<<i+1<<" time_i = "<< controlPoints.at(i).time<< " time_i+1 = "<<controlPoints.at(i+1).time<<std::endl;
			indices.push_back(i);
		}
	}

	for(int i=indices.size()-1; i>=0; i--)
	{
		std::cout<<"removing control point with index: "<<indices.at(i)<<std::endl;
		controlPoints.erase(controlPoints.begin() + indices.at(i));
	}
	indices.clear();
}

void  Curve::adjustControlPoints()
{
	bool foundPoint = true;
	for(int i=0; i<controlPoints.size()-2; i++)
	{
		if(controlPoints.at(i).time == controlPoints.at(i+1).time && i+2<controlPoints.size())
		{
			controlPoints.at(i+1).time = (controlPoints.at(i).time + controlPoints.at(i+2).time)/2.0f;
		}
		else if(controlPoints.at(i).time == controlPoints.at(i+1).time && i+2>=controlPoints.size())
		{
			controlPoints.at(i+1).time += 25;
		}
	}



}