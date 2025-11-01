// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <Eigen/Core>

class Box
{
	//internal storage for lower and upper corner points of the box
	std::pair<Eigen::Vector3f, Eigen::Vector3f> bounds;

public:

	//creates an empty box like the method Clear
	Box();

	//construct a box with the gven lower and upper corner points
	Box(const Eigen::Vector3f& lbound, const Eigen::Vector3f& ubound);

	//returns the corner point which is the lower bound of the box in all dimensions
	Eigen::Vector3f& LowerBound();

	//returns the corner point which is the lower bound of the box in all dimensions
	const Eigen::Vector3f& LowerBound() const;

	//returns the corner point which is the upper bound of the box in all dimensions
	Eigen::Vector3f& UpperBound();

	//returns the corner point which is the upper bound of the box in all dimensions
	const Eigen::Vector3f& UpperBound() const;
	
	//returns a vector containing the extents of the box in all dimensions
	Eigen::Vector3f Extents() const;

	//returns a vector containing the extents of the box in all dimensions divided by 2
	Eigen::Vector3f HalfExtents() const;

	//returns the center of the box
	Eigen::Vector3f Center() const;

	//returns the surface area of the box
	float SurfaceArea() const;

	//returns the volume of the box
	float Volume() const;

	//returns the box radius for a given direction vector a 
	//if the box is centered at the origin 
	//then the projection of the box on direction a is contained within  the Interval [-r,r]
	float Radius(const Eigen::Vector3f& a) const;

	//returns true if the box b overlaps with the current one
	bool Overlaps(const Box& b) const;

	//returns true if the point p is inside this box
	bool IsInside(const Eigen::Vector3f& p) const;

	//returns  true if box b is inside this box
	bool IsInside(const Box& b) const;
	
	//creates a box which goes from [+infinity,- infinity] in al dimensions
	void Clear();
	
	//enlarges the box such that the point p is inside afterwards
	void Insert(const Eigen::Vector3f& p);

	//enlarges the box such that box b is inside afterwards
	void Insert(const Box& b);

	//returns the point on or inside the box with the smallest distance to p 
	Eigen::Vector3f ClosestPoint(const Eigen::Vector3f& p) const;

	//returns the squared distance between p and the box  
	float SqrDistance(const Eigen::Vector3f& p) const;

	//returns the euclidean distance between p and the box 
	float Distance(const Eigen::Vector3f& p) const;

};
