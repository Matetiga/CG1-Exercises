// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "HashGrid.h"
#include <iostream>

void BuildHashGridFromTriangles(const HEMesh& m, HashGrid<Triangle>& grid, const Eigen::Vector3f& cellSize)
{
	std::cout << "Building hash grid from triangles .." << std::endl;
	grid = HashGrid<Triangle>(cellSize, 1);
	auto fend = m.faces_end();
	for(auto fit = m.faces_begin(); fit != fend; ++fit)
		grid.Insert(Triangle(m,*fit));
	std::cout << "Done (using " << grid.NumCells() << " cells)." << std::endl;
}

void BuildHashGridFromVertices(const HEMesh& m, HashGrid<Point>& grid, const Eigen::Vector3f& cellSize)
{
	std::cout << "Building hash grid from vertices .." << std::endl;
	grid = HashGrid<Point>(cellSize, 1);
	auto vend = m.vertices_end();
	for(auto vit = m.vertices_begin(); vit != vend; ++vit)
		grid.Insert(Point(m,*vit));
	std::cout << "Done (using " << grid.NumCells() << " cells)." << std::endl;
}

void BuildHashGridFromEdges(const HEMesh& m, HashGrid<LineSegment >& grid, const Eigen::Vector3f& cellSize)
{
	std::cout << "Building hash grid from edges .." << std::endl;
	grid = HashGrid<LineSegment>(cellSize, 1);
	auto eend = m.edges_end();
	for(auto eit = m.edges_begin(); eit != eend; ++eit)
		grid.Insert(LineSegment(m,*eit));
	std::cout << "Done (using " << grid.NumCells() << " cells)." << std::endl;
}
