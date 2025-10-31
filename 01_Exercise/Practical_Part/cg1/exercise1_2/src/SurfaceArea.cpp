// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "SurfaceArea.h"

#include <iostream>
#include <vector>

float ComputeSurfaceArea(const HEMesh& m)
{
	float area_a   = 0.0f;
	float area_b = 0.0f;

	for (const auto& f : m.faces())
	{
		std::vector<OpenMesh::Vec3f> pts;
		for (const auto& vh : m.fv_range(f))
			pts.push_back(m.point(vh));

		if (pts.size() < 3) continue;

		// a)
		{
			const OpenMesh::Vec3f& p0 = pts[0];
			for (size_t i = 1; i + 1 < pts.size(); ++i)
			{
				OpenMesh::Vec3f v1 = pts[i] - p0;
				OpenMesh::Vec3f v2 = pts[i + 1] - p0;
				OpenMesh::Vec3f cr = OpenMesh::cross(v1, v2);
				area_a += 0.5f * OpenMesh::norm(cr);
			}
		}

		// b)
		{
			OpenMesh::Vec3f sumCross(0.0f, 0.0f, 0.0f);
			const size_t n = pts.size();
			for (size_t i = 0; i < n; ++i)
			{
				const OpenMesh::Vec3f& p0 = pts[i];
				const OpenMesh::Vec3f& p1 = pts[(i + 1) % n];
				sumCross += OpenMesh::cross(p0, p1);
			}
			area_b += 0.5f * OpenMesh::norm(sumCross);
		}
	}

	return area_b;
}