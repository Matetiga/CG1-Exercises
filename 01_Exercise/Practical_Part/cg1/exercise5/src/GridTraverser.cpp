// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "GridTraverser.h"
#include "GridUtils.h"


GridTraverser::GridTraverser()
{ }

GridTraverser::GridTraverser(const Eigen::Vector3f& o, const Eigen::Vector3f&d, const Eigen::Vector3f cell_extents)
	: orig(o), dir(d), cellExtents(cell_extents)
{
	dir.normalize();
	Init();
}

Eigen::Vector3f& GridTraverser::Origin()
{
	return orig;
}
const Eigen::Vector3f& GridTraverser::Origin() const
{
	return orig;
}

Eigen::Vector3f& GridTraverser::Direction()
{
	return dir;
}

const Eigen::Vector3f& GridTraverser::Direction() const
{
	return dir;
}

void GridTraverser::SetCellExtents(const Eigen::Vector3f& cellExtent)
{
	this->cellExtents = cellExtent;
	Init();
}

void GridTraverser::Init()
{
	current = PositionToCellIndex(orig, cellExtents);
	/* Task 5.2.2 */
	//you can add some precalculation code here
}

void GridTraverser::operator++(int)
{
	/* Task 5.2.2 */
	//traverse one step along the ray
	//update the cell index stored in attribute "current"
}

Eigen::Vector3i GridTraverser::operator*()
{
	return current;
}

	
