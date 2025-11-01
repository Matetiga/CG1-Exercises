// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "AABBTree.h"
#include <iostream>

void BuildAABBTreeFromTriangles(const HEMesh& m, AABBTree<Triangle >& tree)
{
	std::cout << "Building AABB tree from triangles .." << std::endl;
	tree.Clear();
	auto fend = m.faces_end();
	for(auto fit = m.faces_begin(); fit != fend; ++fit) 
		tree.Insert(Triangle(m,*fit));
	
	tree.Complete();
	std::cout << "Done." << std::endl;
}

void BuildAABBTreeFromVertices(const HEMesh& m, AABBTree<Point>& tree)
{
	std::cout << "Building AABB tree from vertices .." << std::endl;
	tree.Clear();
	auto vend = m.vertices_end();
	for(auto vit = m.vertices_begin(); vit != vend; ++vit)
		tree.Insert(Point(m,*vit));
	
	tree.Complete();
	std::cout << "Done." << std::endl;
}

void BuildAABBTreeFromEdges(const HEMesh& m, AABBTree<LineSegment>& tree)
{
	std::cout << "Building AABB tree from edges .." << std::endl;
	tree.Clear();
	auto eend = m.edges_end();
	for(auto eit = m.edges_begin(); eit != eend; ++eit)	
		tree.Insert(LineSegment(m,*eit));
	
	tree.Complete();
	std::cout << "Done." << std::endl;
}
