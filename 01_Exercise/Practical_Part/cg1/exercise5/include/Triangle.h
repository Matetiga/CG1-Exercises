// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once
#include "Box.h"
#include "util/OpenMeshUtils.h"


/*
a triangle primitive which can be used with the AABBTree and the HashGrid data structure
*/
class Triangle
{
	//internal storage of the first vertex position of the triangle
	Eigen::Vector3f v0;
	//internal storage of the second vertex position of the triangle
	Eigen::Vector3f v1;
	//internal storage of the third vertex position of the triangle
	Eigen::Vector3f v2;
	//internal storage of the optional face handle to identify the originating face in a half edge mesh instance
	OpenMesh::FaceHandle h;

public:

	//default constructor
	Triangle();
	//constructs a triangle using the vertex positions v0,v1 and v2
	Triangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,const Eigen::Vector3f& v2);
	//constructs a triangle from  the face f of the given halfedge mesh m
	Triangle(const HEMesh&m, const OpenMesh::FaceHandle& f);
	//returns the axis aligned bounding box of the triangle
	Box ComputeBounds() const;
	//returns true if the triangle overlaps the given box b
	bool Overlaps(const Box& b) const;
	//returns the barycentric coordinates of the point with thesmallest distance to point p which lies on the triangle
	void ClosestPointBarycentric(const Eigen::Vector3f& p, float& l0, float& l1, float& l2) const;
	//returns the point with smallest distance to point p which lies on the triangle
	Eigen::Vector3f ClosestPoint(const Eigen::Vector3f& p) const;
	//returns the squared distance between point p and the triangle
	float SqrDistance(const Eigen::Vector3f& p) const;
	//returns the euclidean distance between point p and the triangle
	float Distance(const Eigen::Vector3f& p) const;
	//returns a reference point  which is on the triangle and is used to sort the primitive in the AABB tree construction
	Eigen::Vector3f ReferencePoint() const;

};

