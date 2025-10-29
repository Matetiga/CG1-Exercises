// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <memory>
#include <random>
#include "Smoothing.h"


void SmoothUniformLaplacian (HEMesh &m, float lambda)
{
	/*Task 2.2.3*/
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
