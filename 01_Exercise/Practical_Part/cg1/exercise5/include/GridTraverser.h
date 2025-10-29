// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <array>
#include <Eigen/Core>

class GridTraverser
{
	//ray origin and direction
	Eigen::Vector3f orig,dir;
	//grid cell extents
	Eigen::Vector3f cellExtents;
	//current cell index
	Eigen::Vector3i current;

	/* you can additional attributes for incremental calculation here */

public:
	//default constructor
	GridTraverser();

	//constructs a grid traverser for a given ray with origin o, and ray direction d for a grid with cell extents ce
	GridTraverser(const Eigen::Vector3f& o, const Eigen::Vector3f&d, const Eigen::Vector3f ce);

	//accessor of ray origin
	Eigen::Vector3f& Origin();

	//const accessor of ray origin
	const Eigen::Vector3f& Origin() const;

	//accessor of ray direction
	Eigen::Vector3f& Direction();
	
	//const accessor of ray direction
	const Eigen::Vector3f& Direction() const;	

	//set cell extents
	void SetCellExtents(const Eigen::Vector3f& cellExtent);	

	//init at origin cell
	void Init();

	//step to next cell along ray direction
	void operator++(int);

	//return current cell index
	Eigen::Vector3i operator*();		
};
