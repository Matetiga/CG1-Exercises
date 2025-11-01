// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <unordered_map>
#include <array>
#include <vector>
#include "Box.h"
#include "GridUtils.h"
#include "Triangle.h"
#include "Point.h"
#include "LineSegment.h"

template <typename Primitive >
class HashGrid 
{
public:	

	//hash function
	struct GridHashFunc
	{
		size_t operator()(const Eigen::Vector3i &idx ) const
		{
			static const int p1 = 131071;
			static const int p2 = 524287;
			static const int p3 = 8191;
			return idx[0] * p1 + idx[1] * p2 + idx[2] * p3;
		}
	};
	//type of internal hash map
	typedef std::unordered_map<Eigen::Vector3i,std::vector<Primitive>,GridHashFunc> CellHashMapType;	
	
private:
	//internal hash map storing the data of each non empty grid cell
	//it is a map with a 3 dimensional cell index index as a key and a std::vector<Primitive> as value type
	CellHashMapType cellHashMap;
	//internal extents of a cell
	Eigen::Vector3f cellExtents;

public:
	//constructor for hash grid with uniform cell extent
	//initial size is used to preallocate memory for the internal unordered map
	HashGrid(const float cellExtent=0.01,const int initialSize=1): cellHashMap(initialSize)
	{		
		cellExtents[0] =cellExtents[1] =cellExtents[2] = cellExtent;
	}

	//constructor for  hash grid with non uniform cell extents
	//initial size is used to preallocate memory for the internal unordered map
	HashGrid(const Eigen::Vector3f& cellExtents,const int initialSize): cellHashMap(initialSize),cellExtents(cellExtents)
	{	
	}

	//resize hash map with at least count buckets
	void ReHash(const int count)
	{
		cellHashMap.rehash(count);
	}

	
	//converts a position to a grid index 
	Eigen::Vector3i PositionToIndex(const Eigen::Vector3f& pos) const
	{	
		return PositionToCellIndex(pos, cellExtents) ;
	}

	//return the center position of a cell specified by its cell key
	Eigen::Vector3f CellCenter(const Eigen::Vector3i& idx) const
	{
		Eigen::Vector3f p;
		for(int d = 0; d < 3; ++d)
			p[d] = (idx[d] + 0.5f)*cellExtents[d];
		
		return p;
	}

	//return the center position of a cell containing give position pos
	Eigen::Vector3f CellCenter(const Eigen::Vector3f& pos) const
	{
		return CellCenter(PositionToIndex(pos));
	}
	
	//return the min corner position of a cell specified by its cell key
	Eigen::Vector3f CellMinPosition(const Eigen::Vector3i& key) const
	{
		Eigen::Vector3f p;
		for(int d = 0; d < 3; ++d)
			p[d] = key[d]*cellExtents[d];
		
		return p;
	}

	//return the min corner position of a cell containing the point pos
	Eigen::Vector3f CellMinPosition(const Eigen::Vector3f& pos) const
	{
		return CellMinPosition(PositionToIndex(pos));
	}

	//return the max corner position of a cell specified by its cell key
	Eigen::Vector3f CellMaxPosition(const Eigen::Vector3i& idx) const 
	{
		Eigen::Vector3f p;
		for(int d = 0; d < 3; ++d)
			p[d] = (idx[d]+1)*cellExtents[d];
		
		return p;
	}

	//return the max corner position of a cell containing the point pos
	Eigen::Vector3f CellMaxPosition(const Eigen::Vector3f& pos) const
	{
		return CellMaxPosition(PositionToIndex(pos));
	}

	//returns bounding box of cell with index idx
	Box CellBounds(const Eigen::Vector3i& idx) const
	{
		return Box(CellMinPosition(idx),CellMaxPosition(idx));		
	}
	
	//returns the  bounding box of cell containing the point pos
	Box CellBounds(const Eigen::Vector3f& pos) const
	{
		Eigen::Vector3i idx = PositionToIndex(pos);
		return Box(CellMinPosition(idx),CellMaxPosition(idx));		
	}
	
	//returns the extents of a grid cell
	Eigen::Vector3f CellExtents() const
	{
		return cellExtents;
	}

	//returns volume of a grid cell
	float CellVolume() const
	{
		float vol = 0;
		for(int d = 0; d < 3; ++d)
			vol *= cellExtents[d];
		return vol;
	}
	
	//removes all non empty cells from the hash grid
	bool Empty(const Eigen::Vector3i& idx) const
	{
		auto it = cellHashMap.find(idx);
		if(it == cellHashMap.end())
			return true;
		return false;		
	}


	//inserts primitive p into all overlapping hash grid cells
	//the primitive must implement a method "box compute_bounds()" which returns an axis aligned bounding box
	//and a method "bool overlaps(const box& b)" which returns true if the primitive overlaps the given box b
	void Insert(const Primitive& p)
	{
		Box b = p.ComputeBounds();
		Eigen::Vector3f lb = b.LowerBound();
		Eigen::Vector3f ub = b.UpperBound();
		if(lb[0] > ub[0])
			return;
		if(lb[1] > ub[1])
			return;
		if(lb[2] > ub[2])
			return;
		Eigen::Vector3i lb_idx = PositionToIndex(lb);
		Eigen::Vector3i ub_idx = PositionToIndex(ub);

		
		
		Eigen::Vector3i idx;
		for(idx[0] = lb_idx[0]; idx[0] <=ub_idx[0]; ++idx[0])
			for(idx[1] = lb_idx[1]; idx[1] <=ub_idx[1]; ++idx[1])
				for(idx[2] = lb_idx[2]; idx[2] <=ub_idx[2]; ++idx[2])
					if(p.Overlaps(CellBounds(idx)))
						cellHashMap[idx].push_back(p);

	}
	

	//remove all cells from hash grid
	void Clear()
	{
		cellHashMap.clear();
	}
	
	//returns true if hashgrid contains no cells
	bool Empty() const
	{
		return cellHashMap.empty();
	}

	//returns the number of non empty cells
	size_t NumCells() const
	{
		return cellHashMap.size();
	}

	//iterator pointing to the  first cell within the hashgrid
	typename CellHashMapType::iterator NonEmptyCellsBegin()
	{
		return cellHashMap.begin();
	}

	//iterator pointing behind the last cell within the hashgrid
	typename CellHashMapType::iterator NonEmptyCellsEnd()
	{
		return cellHashMap.end();
	}

	//const iterator pointing to the first cell within the hashgrid
	typename CellHashMapType::const_iterator NonEmptyCellsBegin() const
	{
		return cellHashMap.begin();
	}

	//const iterator pointing behind the last cell within the hashgrid
	typename CellHashMapType::const_iterator NonEmptyCellsEnd() const
	{
		return cellHashMap.end();
	}

	//iterator pointing to the first primitive stored in the cell idx
	typename std::vector<Primitive>::iterator PrimitivesBegin(const Eigen::Vector3i& idx)
	{
		assert(!Empty(idx));
		return cellHashMap[idx].begin();
	}

	//iterator pointing after the last primitive stored in the cell idx
	typename std::vector<Primitive>::iterator PrimitivesEnd(const Eigen::Vector3i& idx)
	{
		assert(!Empty(idx));
		return cellHashMap[idx].end();
	}

	//const iterator pointing to the first primitive stored in the cell idx
	typename std::vector<Primitive>::const_iterator PrimitivesBegin(const Eigen::Vector3i& idx) const
	{
		assert(!Empty(idx));
		return cellHashMap[idx].cbegin();
	}

	//const iterator pointing after the last primitive stored in the cell idx
	typename std::vector<Primitive>::const_iterator PrimitivesEnd(const Eigen::Vector3i& idx) const
	{
		assert(!Empty(idx));
		return cellHashMap[idx].cend();
	}
};

//helper function to construct a hashgrid data structure from the triangle faces of the halfedge mesh m
void BuildHashGridFromTriangles(const HEMesh& m, HashGrid<Triangle>& grid, const Eigen::Vector3f& cellSize);
//helper function to construct a hashgrid data structure from the vertices of the halfedge mesh m
void BuildHashGridFromVertices(const HEMesh& m, HashGrid<Point>& grid, const Eigen::Vector3f& cellSize);
//helper function to construct a hashgrid data structure from the edges of the halfedge mesh m
void BuildHashGridFromEdges(const HEMesh& m, HashGrid<LineSegment >& grid, const Eigen::Vector3f& cellSize);




