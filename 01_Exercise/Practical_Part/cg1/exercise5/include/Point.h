// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once
#include <util/OpenMeshUtils.h>
#include "Box.h"


class Point
{
	//internal storage of point position
	Eigen::Vector3f v0;
	//internal storage for a vertex handle
	//this vertex handle can be used to optionally identify the vertex in a halfedge mesh data structure instance
	OpenMesh::VertexHandle h;
	

public:
	
	//default constructor
	Point();
	
	//construct a point with given point position v0
	Point(const Eigen::Vector3f& v0);

	//construct a point from vertex v of giben halfedge mesh m
	Point(const HEMesh &m, const OpenMesh::VertexHandle& v);
		
	//returns axis aligned bounding box of point
	Box ComputeBounds() const;

	//returns true if point overlap with box b
	bool Overlaps(const Box& b) const;

	//returns the point position
	Eigen::Vector3f ClosestPoint(const Eigen::Vector3f& p) const;

	//returns the squared distance between the query point p and the current point
	float SqrDistance(const Eigen::Vector3f& p) const;

	//returns the euclidean distance between the query point p and the current point
	float Distance(const Eigen::Vector3f& p) const;

	//returns a the position of the point as a reference point which is used to sort the primitive in the AABB tree construction
	Eigen::Vector3f ReferencePoint() const;
};





