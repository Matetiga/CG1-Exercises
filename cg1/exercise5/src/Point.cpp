// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Point.h"
#include "GridUtils.h"

//default constructor
Point::Point(){}
	
//construct a point with given point position v0
Point::Point(const Eigen::Vector3f& v0):v0(v0)
{
}

//construct a point from vertex v of giben halfedge mesh m
Point::Point(const HEMesh &m, const OpenMesh::VertexHandle& v):h(v)
{
	v0 = ToEigenVector(m.point(v));
}
		
//returns axis aligned bounding box of point
Box Point::ComputeBounds() const
{
	Box b;	
	b.Insert(v0);
		
	return b;
}

//returns true if point overlap with box b
bool Point::Overlaps(const Box& b) const
{
	Eigen::Vector3f lb = b.LowerBound();
	Eigen::Vector3f ub = b.UpperBound();
	return 
		(v0[0] >= lb[0] && v0[0] <= ub[0]&&
		v0[1] >= lb[1] && v0[1] <= ub[1] &&
		v0[2] >= lb[2] && v0[2] <= ub[2]);		   
}

//returns the point position
Eigen::Vector3f Point::ClosestPoint(const Eigen::Vector3f& p) const
{
	return v0;
}

//returns the squared distance between the query point p and the current point
float Point::SqrDistance(const Eigen::Vector3f& p) const
{
	Eigen::Vector3f d = p-ClosestPoint(p);
	return d.squaredNorm();
}

//returns the euclidean distance between the query point p and the current point
float Point::Distance(const Eigen::Vector3f& p) const
{
	return sqrt(SqrDistance(p));
}

//returns a the position of the point as a reference point which is used to sort the primitive in the AABB tree construction
Eigen::Vector3f Point::ReferencePoint() const
{
	return v0;
}
