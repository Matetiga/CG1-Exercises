// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <gui/AbstractViewer.h>
#include <gui/SliderHelper.h>
#include <util/OpenMeshUtils.h>

#include "AABBTree.h"
#include "HashGrid.h"
#include "Point.h"
#include "LineSegment.h"
#include "Triangle.h"

#include <gui/ShaderPool.h>

class Viewer : public nse::gui::AbstractViewer
{
public:
	Viewer();

	void drawContents();

private:

	enum PrimitiveType
	{
		Vertex, Edge, Tri
	};

	void SetupGUI();
	void MeshUpdated();

	void FindClosestPoint(const Eigen::Vector3f& p);
	void BuildGridVBO();
	void BuildRayVBOs();

	template <typename Grid>
	void BuildGridVBO(const Grid& grid)
	{
		std::vector<Eigen::Vector4f> positions;
		for (auto it = grid.NonEmptyCellsBegin(); it != grid.NonEmptyCellsEnd(); ++it)
		{
			auto box = grid.CellBounds(it->first);
			AddBoxVertices(box, positions);
		}

		ShaderPool::Instance()->simpleShader.bind();
		gridVAO.bind();
		gridPositions.uploadData(positions).bindToAttribute("position");
		gridVAO.unbind();
		gridIndices = (GLuint)positions.size();
	}

	void AddBoxVertices(const Box& box, std::vector<Eigen::Vector4f>& positions);

	nanogui::ComboBox* shadingBtn;
	nanogui::CheckBox* chkRenderMesh;
	nanogui::CheckBox* chkRenderGrid;
	nanogui::CheckBox* chkRenderRay;
	int raySteps;

	nse::gui::VectorInput* sldQuery, *sldRayOrigin, *sldRayDir;
	nanogui::ComboBox* cmbPrimitiveType;
	
	HEMesh polymesh;
	float bboxMaxLength;
	MeshRenderer renderer;
	
	AABBTree<Point> vertexTree;
	AABBTree<LineSegment> edgeTree;
	AABBTree<Triangle> triangleTree;
	
	HashGrid<Point> vertexGrid;
	HashGrid<LineSegment> edgeGrid;
	HashGrid<Triangle> triangleGrid;

	nse::gui::GLBuffer closestPositions;
	nse::gui::GLVertexArray closestVAO;

	nse::gui::GLBuffer gridPositions;
	nse::gui::GLVertexArray gridVAO;
	GLuint gridIndices;

	nse::gui::GLBuffer rayPositions, rayCellsPositions;
	nse::gui::GLVertexArray rayVAO, rayCellsVAO;
	GLuint rayCellsIndices;
};
