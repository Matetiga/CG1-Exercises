// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "LineSegment.h"
#include "GridUtils.h"


//default constructor
LineSegment::LineSegment()
{
}

//constructs a line segment by the two end points v0, v1 without using the edge handle
LineSegment::LineSegment(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1):v0(v0),v1(v1)
{
}
	
//construct a line segment from the edge e of the halfedge mesh m
LineSegment::LineSegment(const HEMesh& m,const OpenMesh::EdgeHandle& e):h(e)
{
	auto h = m.halfedge_handle(e, 0);
	v0 = ToEigenVector(m.point(m.from_vertex_handle(h)));
	v1 = ToEigenVector(m.point(m.to_vertex_handle(h)));
}

//returns an axis aligned bounding box of the line segment
Box LineSegment::ComputeBounds() const
{
	Box b;	
	b.Insert(v0);
	b.Insert(v1);
	return b;
}
	
//returns true if the line segment overlaps the given box b
bool LineSegment::Overlaps(const Box& b) const
{

	Box aabb = ComputeBounds();

	if(!b.Overlaps(aabb))
		return false;
		
	Eigen::Vector3f o = b.Center();
	Eigen::Vector3f u1 = v0-o;
	Eigen::Vector3f u2 = v1-o;
		
	Eigen::Vector3f d1 = v1-v0;
	d1.normalize();
		
	float r = b.Radius(d1);
	float lb = u1.dot(d1);
	float ub = u2.dot(d1);
	if(lb > ub) 
		std::swap(lb,ub);
	if(lb > r || ub < -r)
		return false;

	Eigen::Vector3f e1(1,0,0);
	Eigen::Vector3f d2= d1.cross(e1);
	r = b.Radius(d2);
	lb = u1.dot(d2);
	ub = u2.dot(d2);
	if(lb > ub) 
		std::swap(lb,ub);
	if(!OverlapIntervals(-r,r,lb,ub))
		return false;

	Eigen::Vector3f e2(0,1,0);
	Eigen::Vector3f d3 = d1.cross(e2);
	r = b.Radius(d3);
	lb = u1.dot(d3);
	ub = u2.dot(d3);
	if(lb > ub) 
		std::swap(lb,ub);
	if(!OverlapIntervals(-r,r,lb,ub))
		return false;

	Eigen::Vector3f e3(0,0,1);
	Eigen::Vector3f d4 = d1.cross(e3);
	r = b.Radius(d4);
	lb = u1.dot(d4);
	ub = u2.dot(d4);
		
	if(!OverlapIntervals(-r,r,lb,ub))
		return false;


	return true;
		
				   
}

//returns the point with smallest distance topoint p which lies on the line segment
Eigen::Vector3f LineSegment::ClosestPoint(const Eigen::Vector3f& p) const
{
	//the two endpoints of the line segment are v0,v1
	/* Task 5.2.1 */
	return Eigen::Vector3f(0,0,0);
}

//returns the squared distance between point p and the line segment
float LineSegment::SqrDistance(const Eigen::Vector3f& p) const
{
	Eigen::Vector3f d = p-ClosestPoint(p);
	return d.squaredNorm();
}

//returns the euclidean distance between point p and the line segment
float LineSegment::Distance(const Eigen::Vector3f& p) const
{
	return sqrt(SqrDistance(p));
}
	
//returns a reference point  which is on the line segment and is used to sort the primitive in the AABB tree construction
Eigen::Vector3f LineSegment::ReferencePoint() const
{
	return 0.5f*(v0 + v1);
}



