// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <memory>
#include <random>
#include "Smoothing.h"


void SmoothUniformLaplacian (HEMesh &m, float lambda)
{
	/*Task 2.2.3*/
	// Uniform discrete Laplacian smoothing
		// Uniform Laplacian means that the new position is based on the difference between the current position and the average position of the neighbors
		// so the new point will be centered relative to its neighbors

	// 𝒑𝑖 ← 𝒑𝑖 + Δt ∙ 𝛼 ∙ Δs * p𝑖
	// with : Δs *p𝑖 = -2H n 
		// (n as the surface normal) (H as the mean curvature)

	for(auto vertex : m.vertices()){
		OpenMesh::Vec3f average(0.f, 0.f, 0.f);
		unsigned int num_neightbors = 0;
		std::cout << "Vertex idx : " << vertex.idx() << std::endl;

		// iterate through neigbors 
		for (auto neighbor : m.vv_range(vertex)) {
			std::cout << "   Neighbor idx : "<< neighbor.idx()<< " with points : " << m.point(neighbor) << std::endl;
			average += m.point(neighbor);
			num_neightbors += 1;
		}
		std::cout << "current average before division : " << average << std::endl;
		average /= (float) num_neightbors;
		m.point(vertex) = m.point(vertex) + lambda * average;
	}

	
}

void SmoothCotanLaplacian (HEMesh &m, float lambda)
{
	/*Task 2.2.3*/
}

void AddNoise (HEMesh &m, OpenMesh::MPropHandleT<Viewer::BBoxType> bbox_prop)
{
	std::mt19937 rnd;
	std::normal_distribution<float> dist;

	for (auto v : m.vertices())
	{
		OpenMesh::Vec3f n;
		m.calc_vertex_normal_correct(v, n);
		const auto diag = m.property(bbox_prop).diagonal();
		const float base_diag = std::min(diag.x(), std::min(diag.y(), diag.z())) / 20.f;
		float base_nb=0, nb_num=0;
		for (auto vnb : m.vv_range(v))
		{
			base_nb += (m.point(v) - m.point(vnb)).norm();
			nb_num++;
		}
		base_nb /= 4.f * nb_num;

		m.point(v) += std::min(base_diag, base_nb) * dist(rnd) * n.normalized();
	}
}
