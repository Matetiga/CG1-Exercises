// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "GridTraverser.h"
#include "GridUtils.h"
#include <limits>
#include <cmath>

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
	const float inf = std::numeric_limits<float>::infinity();

	//step direction
	for (int d = 0; d < 3; ++d)
	{
		if (dir[d] > 0.0f) step[d] = 1;
		else if (dir[d] < 0.0f) step[d] = -1;
		else step[d] = 0;
	}

	//tDelta: how far along the ray we must move (in t) to cross one whole cell in that axis
	for (int d = 0; d < 3; d++)
	{
		if (step[d] == 0) tDelta[d] = inf;
		else
			tDelta[d] = std::abs(cellExtents[d] / dir[d]);
	}

	//tMax: t value at which we cross the first boundary per axis
	for (int d = 0; d < 3; ++d)
	{
		if (step[d] == 0)
		{
			tMax[d] = inf;
			continue;
		}

		float nextBoundary;
		if (step[d] > 0) nextBoundary = (static_cast<float>(current[d]) + 1.0f) * cellExtents[d];
		else nextBoundary = static_cast<float>(current[d]) * cellExtents[d];

		tMax[d] = (nextBoundary - orig[d]) / dir[d];

		if (tMax[d] < 0.0f) tMax[d] = 0.0f;
	}
}


void GridTraverser::operator++(int)
{
	/* Task 5.2.2 */
	//traverse one step along the ray
	//update the cell index stored in attribute "current"
	//if(tMax.x() < tMax.y())
	//{
	//	if(tMax.x() < tMax.z())
	//	{
	//		// x next
	//		current.x() += step.x();
	//		tMax.x() += tDelta.x();
	//	}
	//	else
	//	{
	//		// z next
	//		current.z() += step.z();
	//		tMax.z() += tDelta.z();
	//	}
	//}
	//else
	//{
	//	if(tMax.y() < tMax.z())
	//	{
	//		// y next
	//		current.y() += step.y();
	//		tMax.y() += tDelta.y();
	//	}
	//	else
	//	{
	//		// z next
	//		current.z() += step.z();
	//		tMax.z() += tDelta.z();
	//	}
	//}

	int axis = 0;
	if (tMax.y() < tMax.x()) axis = 1;
	if (tMax.z() < tMax[axis]) axis = 2;

	current[axis] += step[axis];
	tMax[axis] += tDelta[axis];
}

Eigen::Vector3i GridTraverser::operator*()
{
	return current;
}

	
