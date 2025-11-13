//
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) 2015-2017 CGV TU Dresden - All Rights Reserved
//
#pragma once
#include <vector>
#include <unordered_map>
#include <random>


/*
Usage: 

//C++0x Random Engine Mersenne Twister
std::mt19937 eng;

//a set data structure with amortize constant time insertion, removal, and uniform random sampling from its elements
sample_set<int> my_set;
my_set.reserve(3);

my_set.insert(1);
my_set.insert(2);
my_set.insert(6);

int rand_elem1 = my_set.sample(eng);//1,2 or 6
my_set.remove(2);

int rand_elem2 = my_set.sample(eng); //1 or 6

*/


template <typename T>
struct sample_set
{
	
	typedef int element_index;
	std::vector<T> elements;
	std::unordered_map<T,element_index> index_lut;
		
	
	//create empty set
	sample_set(){}
	
	//reserve memory for n elements
	void reserve(size_t n)
	{
		elements.reserve(n);
		index_lut.rehash(n);
	}

	//insert element elem into set
	void insert(const T& elem)
	{	
		//guard against duplicates
		if (index_lut.find(elem) == index_lut.end())
		{
			element_index idx = (element_index)elements.size();
			elements.push_back(elem);
			index_lut[elem]=idx;
		}
	}

	//remove element elem from set
	bool remove(const T& elem)
	{
		auto it = index_lut.find(elem);
		if(it == index_lut.end())
		{
			return false;
		}

		int i = it->second;
		
		std::swap(elements[i],elements.back());
		elements.pop_back();
		
		index_lut.erase(it);
		if(unsigned(i) < elements.size())
		{		
			index_lut[elements[i]]=i;
		}
		return true;
	}

	//draw a sample from set
	// this makes the function be a template
	// Engine is a template parameter, which allows random number generators (Engines) to be passed in 
	template <typename Engine>
	// T is a class template parameter
	const T& sample(Engine& eng)
	{	
		int b = (element_index)(elements.size()-1);
		std::uniform_int_distribution<int> uniform_dist(0,b);
		element_index idx = uniform_dist(eng);
		return elements[idx];
	}
	
	//returns number of elements in set
	size_t size() const
	{
		return elements.size();
	}

	//returns true if set is empty
	bool empty() const
	{
		return elements.empty();
	}

};

namespace std
{
	template<>
	struct hash<OpenMesh::SmartFaceHandle>
	{
		typedef OpenMesh::SmartFaceHandle argument_type;
		typedef std::size_t result_type;

		result_type operator()(argument_type const& s) const
		{
			return s.idx();
		}
	};
}
