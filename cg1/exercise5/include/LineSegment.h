// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once
#include "Box.h"
#include "GridUtils.h"
#include <util/OpenMeshUtils.h>

/*
a 3d line segment primitive which can be used with the AABBTree and the HashGrid data structure
*/
class LineSegment
{
	//internal storage of start point of line segment
	Eigen::Vector3f v0;
	//internal storage of end point of line segment
	Eigen::Vector3f v1;
	//internal storage for an edge handle
	//this edge handle can be used to optionally identify the edge in a halfedge mesh data structure instance
	OpenMesh::EdgeHandle h;
	
public:
	//default constructor
	LineSegment();

	//constructs a line segment by the two end points v0, v1 without using the edge handle
	LineSegment(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1);

	//construct a line segment from the edge e of the halfedge mesh m
	LineSegment(const HEMesh& m,const OpenMesh::EdgeHandle& e);

	//returns an axis aligned bounding box of the line segment
	Box ComputeBounds() const;
	
	//returns true if the line segment overlaps the given box b
	bool Overlaps(const Box& b) const;

	//returns the point with smallest distance topoint p which lies on the line segment
	Eigen::Vector3f ClosestPoint(const Eigen::Vector3f& p) const;

	//returns the squared distance between point p and the line segment
	float SqrDistance(const Eigen::Vector3f& p) const;

	//returns the euclidean distance between point p and the line segment
	float Distance(const Eigen::Vector3f& p) const;
	
	//returns a reference point  which is on the line segment and is used to sort the primitive in the AABB tree construction
	Eigen::Vector3f ReferencePoint() const;

};

