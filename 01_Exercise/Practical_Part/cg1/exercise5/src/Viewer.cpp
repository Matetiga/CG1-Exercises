// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Viewer.h"

#include <nanogui/window.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/messagedialog.h>
#include <nanogui/layout.h>
#include <nanogui/combobox.h>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <iostream>

#include <gui/SliderHelper.h>

#include <chrono>

#include <gui/ShaderPool.h>
#include "GridTraverser.h"

Viewer::Viewer()
	: AbstractViewer("CG1 Exercise 5"),
	renderer(polymesh),
	closestPositions(nse::gui::VertexBuffer),
	gridPositions(nse::gui::VertexBuffer),
	rayPositions(nse::gui::VertexBuffer), rayCellsPositions(nse::gui::VertexBuffer)
{ 
	SetupGUI();	

	closestVAO.generate();
	gridVAO.generate();
	rayVAO.generate();
	rayCellsVAO.generate();
}

void Viewer::SetupGUI()
{
	auto mainWindow = SetupMainWindow();	

	auto loadFileBtn = new nanogui::Button(mainWindow, "Load Mesh");
	loadFileBtn->setCallback([this]() {
		std::vector<std::pair<std::string, std::string>> fileTypes;
		fileTypes.push_back(std::make_pair("obj", "OBJ File"));
		auto file = nanogui::file_dialog(fileTypes, false);
		if (!file.empty())
		{
			polymesh.clear();
			if (!OpenMesh::IO::read_mesh(polymesh, file))
			{
				new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Warning, "Load Mesh",
					"The specified file could not be loaded");
			}
			else
				MeshUpdated();
		}
	});	

	cmbPrimitiveType = new nanogui::ComboBox(mainWindow, { "Use Vertices", "Use Edges", "Use Triangles" });
	cmbPrimitiveType->setCallback([this](int) { FindClosestPoint(sldQuery->Value()); BuildGridVBO(); });
	
	sldQuery = new nse::gui::VectorInput(mainWindow, "Query", Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), [this](const Eigen::Vector3f& p) { FindClosestPoint(p); });

	sldRayOrigin = new nse::gui::VectorInput(mainWindow, "Ray Origin", Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), [this](const Eigen::Vector3f& p) { BuildRayVBOs(); });
	sldRayDir = new nse::gui::VectorInput(mainWindow, "Ray Direction", Eigen::Vector3f::Constant(-1), Eigen::Vector3f::Constant(1), Eigen::Vector3f::Zero(), [this](const Eigen::Vector3f& p) { BuildRayVBOs(); });
	nanogui::TextBox* txtRaySteps;
	auto sldRaySteps = nse::gui::AddLabeledSlider(mainWindow, "Ray Steps", std::make_pair(1, 200), 80, txtRaySteps);
	sldRaySteps->setCallback([this, txtRaySteps](float value) {
		raySteps = (int)std::round(value);
		txtRaySteps->setValue(std::to_string(raySteps));
		BuildRayVBOs();
	});
	sldRaySteps->callback()(sldRaySteps->value());	

	chkRenderMesh = new nanogui::CheckBox(mainWindow, "Render Mesh"); chkRenderMesh->setChecked(true);
	chkRenderGrid = new nanogui::CheckBox(mainWindow, "Render Non-Empty Grid Cells"); chkRenderGrid->setChecked(false);
	chkRenderRay = new nanogui::CheckBox(mainWindow, "Render Ray"); chkRenderRay->setChecked(false);

	shadingBtn = new nanogui::ComboBox(mainWindow, { "Flat Shading", "Smooth Shading" });

	performLayout();
}

void Viewer::FindClosestPoint(const Eigen::Vector3f& p)
{
	if (polymesh.vertices_empty())
		return;
	Eigen::Vector3f closest;
	auto timeStart = std::chrono::high_resolution_clock::now();
	switch (cmbPrimitiveType->selectedIndex())
	{
	case Vertex:
		closest = vertexTree.ClosestPoint(p);
		break;
	case Edge:
		closest = edgeTree.ClosestPoint(p);
		break;
	case Tri:
		closest = triangleTree.ClosestPoint(p);
		break;
	}	
	auto timeEnd = std::chrono::high_resolution_clock::now();
	std::cout << std::fixed << "Closest point query took " << std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() << " microseconds." << std::endl;

	Eigen::Matrix4Xf points(4, 2);
	points.block<3, 1>(0, 0) = p;
	points.block<3, 1>(0, 1) = closest;
	points.row(3).setConstant(1);
		
	closestVAO.bind();
	ShaderPool::Instance()->simpleShader.bind();
	closestPositions.uploadData(points).bindToAttribute("position");
	closestVAO.unbind();
}

void Viewer::MeshUpdated()
{
	//calculate the bounding Box of the mesh
	nse::math::BoundingBox<float, 3> bbox;
	for (auto v : polymesh.vertices())
		bbox.expand(ToEigenVector(polymesh.point(v)));
	camera().FocusOnBBox(bbox);		
	bboxMaxLength = bbox.diagonal().maxCoeff();

	polymesh.triangulate();
	
	BuildAABBTreeFromVertices(polymesh, vertexTree);
	BuildAABBTreeFromEdges(polymesh, edgeTree);
	BuildAABBTreeFromTriangles(polymesh, triangleTree);

	Eigen::Vector3f cellSize = Eigen::Vector3f::Constant(bbox.diagonal().maxCoeff() / 50);
	BuildHashGridFromVertices(polymesh, vertexGrid, cellSize);
	BuildHashGridFromEdges(polymesh, edgeGrid, cellSize);
	BuildHashGridFromTriangles(polymesh, triangleGrid, cellSize);		

	sldQuery->SetBounds(bbox.min, bbox.max);
	sldQuery->SetValue(bbox.max);
	FindClosestPoint(bbox.max);

	sldRayOrigin->SetBounds(bbox.min, bbox.max);
	sldRayOrigin->SetValue(bbox.min);
	sldRayDir->SetValue(bbox.diagonal().normalized());

	BuildGridVBO();
	
	renderer.Update();
}

void Viewer::BuildGridVBO()
{
	switch (cmbPrimitiveType->selectedIndex())
	{
	case Vertex:
		BuildGridVBO(vertexGrid);
		break;
	case Edge:
		BuildGridVBO(edgeGrid);
		break;
	case Tri:
		BuildGridVBO(triangleGrid);
		break;
	}
}

void Viewer::BuildRayVBOs()
{
	if (polymesh.vertices_empty())
		return;

	//Ray line indicator
	Eigen::Matrix4Xf rayPoints(4, 2);
	rayPoints.block<3, 1>(0, 0) = sldRayOrigin->Value();
	rayPoints.block<3, 1>(0, 1) = sldRayOrigin->Value() + sldRayDir->Value().normalized() * 0.2f * bboxMaxLength;
	rayPoints.row(3).setConstant(1);

	rayVAO.bind();
	ShaderPool::Instance()->simpleShader.bind();
	rayPositions.uploadData(rayPoints).bindToAttribute("position");
	rayVAO.unbind();

	//Ray cells
	std::vector<Eigen::Vector4f> cellPositions;
	GridTraverser trav(sldRayOrigin->Value(), sldRayDir->Value(), vertexGrid.CellExtents());
	for (int i = 0; i < raySteps; ++i, trav++)
	{
		auto bounds = vertexGrid.CellBounds(*trav);
		AddBoxVertices(bounds, cellPositions);
	}

	rayCellsVAO.bind();
	rayCellsPositions.uploadData(cellPositions).bindToAttribute("position");
	rayCellsVAO.unbind();
	rayCellsIndices = (GLuint)cellPositions.size();
}

void Viewer::AddBoxVertices(const Box & box, std::vector<Eigen::Vector4f>& positions)
{
	auto& lb = box.LowerBound();
	auto& ub = box.UpperBound();
	Eigen::Vector4f o; o << lb, 1.0f;
	Eigen::Vector4f x; x << ub.x() - lb.x(), 0, 0, 0;
	Eigen::Vector4f y; y << 0, ub.y() - lb.y(), 0, 0;
	Eigen::Vector4f z; z << 0, 0, ub.z() - lb.z(), 0;
	positions.push_back(o);
	positions.push_back(o + x);
	positions.push_back(o + x);
	positions.push_back(o + x + y);
	positions.push_back(o + x + y);
	positions.push_back(o + y);
	positions.push_back(o + y);
	positions.push_back(o);

	positions.push_back(o + z);
	positions.push_back(o + z + x);
	positions.push_back(o + z + x);
	positions.push_back(o + z + x + y);
	positions.push_back(o + z + x + y);
	positions.push_back(o + z + y);
	positions.push_back(o + z + y);
	positions.push_back(o + z);

	positions.push_back(o);
	positions.push_back(o + z);
	positions.push_back(o + x);
	positions.push_back(o + x + z);
	positions.push_back(o + y);
	positions.push_back(o + y + z);
	positions.push_back(o + x + y);
	positions.push_back(o + x + y + z);
}

void Viewer::drawContents()
{
	glEnable(GL_DEPTH_TEST);

	if (!polymesh.vertices_empty())
	{
		Eigen::Matrix4f view, proj;
		camera().ComputeCameraMatrices(view, proj);
		Eigen::Matrix4f mvp = proj * view;

		if(chkRenderMesh->checked())
			renderer.Render(view, proj, shadingBtn->selectedIndex() == 0);

		ShaderPool::Instance()->simpleShader.bind();
		ShaderPool::Instance()->simpleShader.setUniform("mvp", mvp);

		//Draw line between query point and its closest position
		closestVAO.bind();				
		ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(1, 1, 1, 1));
		glDrawArrays(GL_LINES, 0, 2);
		ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(1, 0, 0, 1));
		glPointSize(3.0f);
		glDrawArrays(GL_POINTS, 0, 2);
		closestVAO.unbind();

		//Draw non-empty grid cells
		if (gridIndices > 0 && chkRenderGrid->checked())
		{
			gridVAO.bind();
			ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1));
			glDrawArrays(GL_LINES, 0, gridIndices);
			gridVAO.unbind();
		}

		if (chkRenderRay->checked())
		{
			//Draw line for ray
			rayVAO.bind();
			ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(1, 1, 1, 1));
			glDrawArrays(GL_LINES, 0, 2);
			ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(0, 1, 0, 1));
			glPointSize(3.0f);
			glDrawArrays(GL_POINTS, 0, 1);
			rayVAO.unbind();

			//Draw ray cells
			rayCellsVAO.bind();
			ShaderPool::Instance()->simpleShader.setUniform("color", Eigen::Vector4f(0.0f, 0.8f, 0.0f, 1));
			glDrawArrays(GL_LINES, 0, rayCellsIndices);
			rayCellsVAO.unbind();
		}
	}
}