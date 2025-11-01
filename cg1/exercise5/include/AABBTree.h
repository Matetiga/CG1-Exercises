// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <queue>
#include <utility>

#include <util/OpenMeshUtils.h>
#include "Box.h"
#include "Triangle.h"
#include "LineSegment.h"
#include "Point.h"


/**
* Axis aligned bounding volume hierachy data structure.
*/
template <typename Primitive>
class AABBTree
{
public:
	typedef std::vector<Primitive> primitive_list;
	//iterator type pointing inside the primitive list
	typedef typename primitive_list::iterator PrimitiveIterator;
	//const iterator type pointing inside the primitive list
	typedef typename primitive_list::const_iterator const_primitive_iterator;
	
	//abstract base class defining the common interface of all aabb tree node
	class AABBNode
	{
	protected:
		//storage of bounding box assosiated with aabb_node
		Box bounds;
	public:
		AABBNode() {
		}

		AABBNode(const Box& b): bounds(b) {
		}

		//returns the bounding box of the node
		Box GetBounds() const
		{
			return bounds;
		}

		virtual int NumPrimitives() const = 0;
		

		//this method must be implemented to return true for a leaf node and false for a non-leaf node
		virtual bool IsLeaf() const = 0;
		//virtual destructor
		virtual ~AABBNode() {}

	};
	///a class representing a leaf node of an aabb tree (non split node)
	class AABBLeafNode: public AABBNode
	{
		
		//internal storage to the range (begin and end pointer) of the primitives associated with the current leaf node
		PrimitiveIterator primitivesBegin, primitivesEnd;
	public:	
		
		
		//construct a leaf node from 
		AABBLeafNode(const PrimitiveIterator& primitivesBegin,
			const PrimitiveIterator& primitivesEnd,
			const Box& b):
				primitivesBegin(primitivesBegin),primitivesEnd(primitivesEnd), AABBNode(b)
		{
		}

		//return always true because its a leaf node
		bool IsLeaf() const
		{
			return true;
		}
		//returns the number primitives assosiated with the current leaf
		int NumPrimitives() const
		{
			return (int)(primitivesEnd-primitivesBegin);
		}

		const_primitive_iterator begin() const
		{
			return primitivesBegin;
		}

		const_primitive_iterator end() const
		{
			return primitivesEnd;
		}


	};

	///a class representing a split node of an aabb tree (non leaf node)
	class AABBSplitNode: public AABBNode
	{
		//child pointers
		AABBNode* children[2];
	public:	
		//default constructor
		AABBSplitNode()
		{
			children[0] = children[1] = nullptr;
		}
		//construct a split node from given Left and right child pointers and given bounding box b of the node
		AABBSplitNode(AABBNode* left_child, AABBNode* right_child,const Box& b): AABBNode(b)
		{
			children[0] = left_child;
			children[1] = right_child;
		}

		//destructor of node, recursively deleted whole subtree
		~AABBSplitNode()
		{
			if(Left() != nullptr)
				delete Left();
			if(Right() != nullptr)
				delete Right();
		}

		//returns always false because its a split node
		bool IsLeaf() const
		{
			return false;
		}
			
		//returns a pointer to the left child node
		AABBNode* Left()
		{
			return children[0];
		}
		//returns a pointer to the right child node
		AABBNode* Right() 
		{
			return children[1];
		}

		//returns a const pointer to the left child node 
		const AABBNode* Left() const
		{
			return children[0];
		}

		//returns a const pointer to the right child node 
		const AABBNode* Right() const
		{
			return children[1];
		}

		//counts the number of primitives of the subtree
		int NumPrimitives() const
		{
			return Left()->NumPrimitives() + Right()->NumPrimitives();	
		}	
	
	};

private:
	//search entry used internally for nearest and k nearest primitive queries
	struct SearchEntry
	{
		//squared distance to node from query point
		float sqrDistance;
		//node
		const AABBNode* node;
		
		//constructor
		SearchEntry(float sqrDistance, const AABBNode* node)
			: sqrDistance(sqrDistance), node(node)
		{ }
		
		//search entry a < b means a.sqr_distance > b. sqr_distance 
		bool operator<(const SearchEntry& e) const
		{
			return sqrDistance > e.sqrDistance;
		}
	};

	//result entry for nearest and k nearest primitive queries
	struct ResultEntry
	{
		//squared distance from query point to primitive
		float sqrDistance;
		//pointer to primitive
		const Primitive* prim;
		//default constructor
		ResultEntry()
			: sqrDistance(std::numeric_limits<float>::infinity()), prim(nullptr)
		{ }
		//constructor
		ResultEntry(float sqrDistance, const Primitive* p)
			: sqrDistance(sqrDistance), prim(p)
		{ }
		//result_entry are sorted by their sqr_distance using this less than operator 
		bool operator<(const ResultEntry& e) const
		{
			return sqrDistance < e.sqrDistance;
		}
	};

	//list of all primitives in the tree
	primitive_list primitives;
	//maximum allowed tree depth to stop tree construction
	int maxDepth;
	//minimal number of primitives to stop tree construction
	int minSize;
	//pointer to the root node of the tree
	AABBNode *root;
	//a flag indicating if the tree is constructed
	bool completed;


public:
	//returns a pointer to the root node of the tree
	AABBNode* Root() 
	{
		assert(IsCompleted());
		return root;
	}

	//returns a const pointer to the root node of the tree
	const AABBNode* Root() const
	{
		assert(IsCompleted());
		return root;
	}

	//constructor of aabb tree 
	//default  maximal tree depth is 20 
	//default minimal size of a node not to be further subdivided in the cnstruction process is two 
	AABBTree(int maxDepth=20, int minSize=2):
		maxDepth(maxDepth),minSize(minSize),root(nullptr),completed(false)
	{
		
	}

	//copy constructor
	AABBTree(const AABBTree& other)
	{
		primitives = other.primitives;
		maxDepth = other.maxDepth;
		minSize = other.minSize;
		root = CopyTree(other.primitives,other.root);
		completed = other.completed;
	}
	
	//move constructor
	AABBTree(AABBTree&& other):root(nullptr),completed(false)
	{
		*this = std::move(other);
	}

	//copy assignment operator
	AABBTree& operator=(const AABBTree& other)
	{
		if(this != &other)
		{
			if(root != nullptr)
				delete root;
			primitives = other.primitives;
			maxDepth = other.maxDepth;
			minSize = other.minSize;
			root = CopyTree(other.primitives,other.root);
			completed = other.completed;
		}
		return *this;
	}

	//move assign operator
	AABBTree& operator=(AABBTree&& other)
	{
		if(this != &other)
		{	
			std::swap(primitives,other.primitives);
			std::swap(maxDepth, other.maxDepth);
			std::swap(minSize, other.minSize);
			std::swap(root,other.root) ;
			std::swap(completed, other.completed);
		}
		return *this;
	}

	//remove all primitives from tree
	void Clear()
	{
		primitives.clear();
		if(root != nullptr)
		{
			delete root;
			root = nullptr;
		}
		completed = false;
	}

	//returns true if tree is empty
	bool Empty() const
	{
		return primitives.Empty();
	}
	
	//insert a primitive into internal primitive list 
	//this method do not construct the tree!
	//call the method Complete, after insertion of all primitives
	void Insert(const Primitive& p)
	{
		primitives.push_back(p);
		completed=false;
	}
	
	//construct the tree from all prior inserted primitives  
	void Complete()
	{
		//if tree already constructed -> delete tree
		if(root != nullptr)
			delete root;
		//compute bounding box over all primitives using helper function
		Box bounds = ComputeBounds(primitives.begin(),primitives.end());
		//initial call to the recursive tree construction method over the whole range of primitives
		root = Build(primitives.begin(),primitives.end(),bounds,0);
		//set completed flag to true
		completed=true;
	}

	//returns true if the tree can be used for queries
	//if the tree is not completed call the method complete()
	bool IsCompleted() const
	{
		return completed;
	}

	//closest primitive computation via linear search
	ResultEntry ClosestPrimitiveLinearSearch(const Eigen::Vector3f& q) const
	{
		ResultEntry best;
		
		auto pend = primitives.end();
		for(auto pit = primitives.begin(); pit != pend; ++pit)
		{
			float dist = pit->SqrDistance(q);
			if(dist < best.sqrDistance)
			{
				best.sqrDistance = dist;
				best.prim = &(*pit);
			}
		}
		return best;
	}

	//computes the k nearest neighbor primitives via linear search
	std::vector<ResultEntry> ClosestKPrimitivesLinearSearch(size_t k, const Eigen::Vector3f& q) const
	{
		std::priority_queue<ResultEntry> k_best;
		
		Primitive best_p;
		auto pend = primitives.end();
		for(auto pit = primitives.begin(); pit != pend; ++pit)
		{
			float dist = pit->SqrDistance(q);
			if(k_best.size() < k )
			{
				k_best.push(ResultEntry(dist,*pit));
				continue;
			}
			if(k_best.top().SqrDistance > dist)
			{
				k_best.pop();
				k_best.push(ResultEntry(dist,*pit));
			}				
		}
		std::vector<ResultEntry> result(k_best.size());
		auto rend = result.end();
		for(auto rit = result.begin(); rit != rend; ++rit)
		{
			*rit = k_best.top();
			k_best.pop();
		}
		return result;

	}
	
	//closest k primitive computation 
	std::vector<ResultEntry> ClosestKPrimitives(size_t k,const Eigen::Vector3f& q) const
	{
		//student begin
		return ClosestKPrimitivesLinearSearch(k,q);
		//student end
	}
	
	//returns the closest primitive and its squared distance to the point q
	ResultEntry ClosestPrimitive(const Eigen::Vector3f& q) const
	{
		assert(IsCompleted());
		if(root == nullptr)
			return ResultEntry();
		/* Task 5.2.1 */
		return ClosestPrimitiveLinearSearch(q);	
	}

	//return the closest point position on the closest primitive in the tree with respect to the query point q
	Eigen::Vector3f ClosestPoint(const Eigen::Vector3f& p) const
	{
		ResultEntry r = ClosestPrimitive(p);
		return  r.prim->ClosestPoint(p);
	}
	
	//return the squared distance between point p and the nearest primitive in the tree
	float SqrDistance(const Eigen::Vector3f& p) const
	{
		ResultEntry r = ClosestPrimitive(p);
		return r.SqrDistance;
	}

	//return the euclidean distance between point p and the nearest primitive in the tree
	float Distance(const Eigen::Vector3f& p) const
	{
		return sqrt(SqrDistance(p));
	}


protected:

	//helper function to copy a subtree
	AABBNode* CopyTree(const primitive_list& other_primitives,AABBNode* node)
	{
		if(node == nullptr)
			return nullptr;
		if(node->IsLeaf())
		{
			AABBLeafNode* leaf = (AABBLeafNode*)node;
			return new AABBLeafNode(primitives.begin()+(leaf->primitives.begin()-other_primitives.begin()),
				primitives.begin()+(leaf->primitives.end()-other_primitives.begin()));
		}
		else
		{
			AABBSplitNode* split = (AABBSplitNode*)node;
			return new AABBSplitNode(CopyTree(other_primitives,split->Left()), 
				CopyTree(other_primitives,split->Right()));
		}
	}

	//helper function to compute an axis aligned bounding box over the range of primitives [begin,end)
	Box ComputeBounds(const_primitive_iterator begin,
		const_primitive_iterator end)
	{
		Box bounds;
		for(auto pit = begin; pit != end; ++pit)
			bounds.Insert(pit->ComputeBounds());
		return bounds;
	}
	
	

	//recursive tree construction initially called from method complete()
	//build an aabb (sub)-tree over the range of primitives [begin,end), 
	//the current bounding box is given by bounds and the current tree depth is given by the parameter depth
	//if depth >= max_depth or the number of primitives (end-begin)  <= min_size a leaf node is constructed
	//otherwise split node is created 
	// to create a split node the range of primitives [begin,end) must be splitted and reordered into two 
	//sub ranges [begin,mid) and [mid,end),
	//therefore sort the range of primitives [begin,end) along the largest bounding box extent by its  reference 
	//point returned by the method ReferencePoint()
	//then choose the median element as mid 
	// the STL routine std::nth_element would be very useful here , you only have to provide a ordering predicate
	//compute the boundg boxed of the two resulting sub ranges and recursivly call build on the two subranges
	//the resulting subtree are used as children of the resulting split node.
	AABBNode* Build(PrimitiveIterator begin, PrimitiveIterator end, Box& bounds, int depth)
	{
		
		if(depth >= maxDepth || end-begin <= minSize)
		{	
			return new AABBLeafNode(begin,end,bounds);
		}

		Eigen::Vector3f e = bounds.Extents();
		
		int axis = 0;
		float max_extent = e[0];
		if(max_extent < e[1])
		{
			axis = 1;
			max_extent = e[1];
		}
		if(max_extent < e[2])
		{
			axis = 2;
			max_extent = e[2];
		}
		

		PrimitiveIterator mid= begin + (end-begin)/2;
		std::nth_element(begin,mid,end,[&axis](const Primitive& a, const Primitive& b)
			{ return a.ReferencePoint()[axis] < b.ReferencePoint()[axis];});
		
		Box lbounds = ComputeBounds(begin,mid);
		Box rbounds = ComputeBounds(mid,end);

		return new AABBSplitNode(Build(begin,mid,lbounds,depth+1),Build(mid,end,rbounds,depth+1),bounds);

	}
};

//helper function to construct an aabb tree data structure from the triangle faces of the halfedge mesh m
void BuildAABBTreeFromTriangles(const HEMesh& m, AABBTree<Triangle >& tree);
//helper function to construct an aabb tree data structure from the vertices of the halfedge mesh m
void BuildAABBTreeFromVertices(const HEMesh& m, AABBTree<Point>& tree);
//helper function to construct an aabb tree data structure from the edges of the halfedge mesh m
void BuildAABBTreeFromEdges(const HEMesh& m, AABBTree<LineSegment>& tree);

