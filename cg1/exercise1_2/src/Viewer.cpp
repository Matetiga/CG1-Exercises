// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Viewer.h"

#include <nanogui/window.h>
#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/messagedialog.h>
#include <nanogui/popupbutton.h>
#include <nanogui/layout.h>
#include <nanogui/combobox.h>

#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include <gui/SliderHelper.h>

#include "Primitives.h"
#include "SurfaceArea.h"
#include "Volume.h"
#include "ShellExtraction.h"
#include "Smoothing.h"
#include "Stripification.h"

const int segmentColorCount = 12;
const float segmentColors[segmentColorCount][3] =
{
	{ 0.651f, 0.808f, 0.890f },
	{ 0.122f, 0.471f, 0.706f },
	{ 0.698f, 0.875f, 0.541f },
	{ 0.200f, 0.627f, 0.173f },
	{ 0.984f, 0.604f, 0.600f },
	{ 0.890f, 0.102f, 0.110f },
	{ 0.992f, 0.749f, 0.435f },
	{ 1.000f, 0.498f, 0.000f },
	{ 0.792f, 0.698f, 0.839f },
	{ 0.416f, 0.239f, 0.604f },
	{ 1.000f, 1.000f, 0.600f },
	{ 0.694f, 0.349f, 0.157f },

};

Viewer::Viewer()
	: AbstractViewer("CG1 Exercise 1+2"),
	renderer(polymesh)
{ 
	SetupGUI();	

	polymesh.add_property(bboxProperty, "bbox");
	polymesh.add_property(avgEdgeLengthProperty, "avgEdgeLen");
	polymesh.add_property(faceIdProperty, "fid");
	polymesh.add_property(faceColorProperty, "fcolor");
	const auto valenceProps = AddValenceProperties(polymesh);
	vertexVertexValenceProperty = valenceProps.vextexValences;
	vertexFaceValenceProperty = valenceProps.faceValences;
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
				MeshUpdated(true);
		}
	});

	auto primitiveBtn = new nanogui::PopupButton(mainWindow, "Create Primitive");
	primitiveBtn->popup()->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 4, 4));

	auto quadBtn = new nanogui::Button(primitiveBtn->popup(), "Quad");
	quadBtn->setCallback([this]() { CreateQuad(polymesh); MeshUpdated(true); });

	auto diskBtn = new nanogui::Button(primitiveBtn->popup(), "Disk");
	diskBtn->setCallback([this]() { CreateDisk(polymesh, 1, 20); MeshUpdated(true); });

	auto tetBtn = new nanogui::Button(primitiveBtn->popup(), "Tetrahedron");
	tetBtn->setCallback([this]() { CreateTetrahedron(polymesh); MeshUpdated(true); });

	auto octaBtn = new nanogui::Button(primitiveBtn->popup(), "Octahedron");
	octaBtn->setCallback([this]() { CreateOctahedron(polymesh, 1); MeshUpdated(true); });

	auto cubeBtn = new nanogui::Button(primitiveBtn->popup(), "Cube");
	cubeBtn->setCallback([this]() { CreateCube(polymesh); MeshUpdated(true); });

	auto icoBtn = new nanogui::Button(primitiveBtn->popup(), "Icosahedron");
	icoBtn->setCallback([this]() { CreateIcosahedron(polymesh, 1); MeshUpdated(true); });

	auto cylBtn = new nanogui::Button(primitiveBtn->popup(), "Cylinder");
	cylBtn->setCallback([this]() { CreateCylinder(polymesh, 0.3f, 1, 20, 10); MeshUpdated(true); });

	auto sphereBtn = new nanogui::Button(primitiveBtn->popup(), "Sphere");
	sphereBtn->setCallback([this]() { CreateSphere(polymesh, 1, 20, 20); MeshUpdated(true); });

	auto torusBtn = new nanogui::Button(primitiveBtn->popup(), "Torus");
	torusBtn->setCallback([this]() { CreateTorus(polymesh, 0.4f, 1, 20, 20); MeshUpdated(true); });

	auto arrowBtn = new nanogui::Button(primitiveBtn->popup(), "Arrow");
	arrowBtn->setCallback([this]() { CreateUnitArrow(polymesh); MeshUpdated(true); });

	auto conicFanBtn = new nanogui::Button(primitiveBtn->popup(), "Conic Fan");
	conicFanBtn->setCallback([this]() {
		// ToDo: move into Primitives.cpp/.h after the end of the semester
		/* CreateConicFan(HEMesh &mesh, float radius, float aspect) */ {
			const float radius = 1.f, aspect = 1.f;
			constexpr const unsigned slices = 7;
			polymesh.clear();
			std::vector<float> circleFracts = {.0f, .075f, .175f, .3f, .45f, .625f, .825f};
			std::vector<OpenMesh::VertexHandle> vh; vh.reserve(slices+1);
			vh.emplace_back(polymesh.add_vertex(OpenMesh::Vec3f(-radius/4.f, -radius/5.f, aspect*radius)));
			for (int i=0; i<slices; i++)
			{
				const float angle = float(2.*M_PI)*circleFracts[i];
				vh.emplace_back(polymesh.add_vertex(
					OpenMesh::Vec3f(std::cos(angle)*radius, std::sin(angle)*radius, .0f ))
				);
			}
			for (int i=0; i<slices; i++)
				polymesh.add_face(vh[0], vh[1 + i%slices], vh[1 + (i+1)% slices]);
		}

		MeshUpdated(true);
	});

	auto calcAreaBtn = new nanogui::Button(mainWindow, "Calculate Mesh Area");
	calcAreaBtn->setCallback([this]() {
		auto area = ComputeSurfaceArea(polymesh);
		std::stringstream ss;
		ss << "The mesh has an area of " << area << ".";
		new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Surface Area",
			ss.str());
	});

	auto calcVolBtn = new nanogui::Button(mainWindow, "Calculate Mesh Volume");
	calcVolBtn->setCallback([this]() {
		//Triangulate the mesh if it is not a triangle mesh
		for (auto f : polymesh.faces()) if (polymesh.valence(f) > 3)
		{
			std::cout << "Triangulating mesh." << std::endl;
			polymesh.triangulate();
			MeshUpdated();
			break;
		}
		
		auto vol = ComputeVolume(polymesh);
		std::stringstream ss;
		ss << "The mesh has a volume of " << vol << ".";
		new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Volume",
			ss.str());
	});

	auto calcValenceBtn = new nanogui::Button(mainWindow, "Compute Vertex Valences");
	calcValenceBtn->setCallback([this]() {
		// compute
		const std::chrono::duration
			vfTime = [this] {
				const auto timeStart = std::chrono::high_resolution_clock::now();
				ComputeVertexFaceValences(polymesh, vertexFaceValenceProperty);
				const auto timeEnd = std::chrono::high_resolution_clock::now();
				return timeEnd - timeStart;
			}(),
			vvTime = [this] {
				const auto timeStart = std::chrono::high_resolution_clock::now();
				ComputeVertexVertexValences(polymesh, vertexVertexValenceProperty);
				const auto timeEnd = std::chrono::high_resolution_clock::now();
				return timeEnd - timeStart;
			}();
		// output timing info
		std::cout << std::fixed << "Vertex valence computation timings for " << polymesh.n_vertices()
		          << " vertices in "<<polymesh.n_faces()<<" faces:" << std::endl
		          << " - vertex-face incidences took "
		          << std::chrono::duration_cast<std::chrono::milliseconds>(vfTime).count()<<"ms" << std::endl
		          << " - vertex-vertex adjacencies took "
		          << std::chrono::duration_cast<std::chrono::milliseconds>(vvTime).count()<<"ms" << std::endl;
		// compute histograms
		const ValenceHistogram
			histVF = ComputeValenceHistogram(polymesh, vertexFaceValenceProperty),
			histVV = ComputeValenceHistogram(polymesh, vertexVertexValenceProperty);
		// format output
		std::cout << "Histogram:"<<std::endl
		          << std::right<<std::setw(20)<<"[Face incidences]"
		          <<    "  "   <<std::setw(20)<<"[Vertex adjacencies]" << std::endl;
		if (histVF.empty())
			std::cout << std::setw(20)<<"<empty> " << "  ";
		else
			std::cout << std::setw(9)<<"valence"<<std::setw(10)<<"num" << "   ";
		if (histVV.empty())
			std::cout << std::setw(20)<<"<empty> " << std::endl;
		else
			std::cout << std::setw(9)<<"valence"<<std::setw(10)<<"num" << std::endl;
		for (auto itVF=histVF.begin(), itVV=histVV.begin();
		     itVF!=histVF.end() || itVV != histVV.end();)
		{
			if (itVF!=histVF.end())
			{
				std::cout << std::setw(9)<<itVF->first<<std::setw(10)<<itVF->second << "   ";
				itVF++;
			}
			else
				std::cout << std::setw(20)<<" " << "  ";
			if (itVV != histVV.end())
			{
				std::cout << std::setw(9)<<itVV->first<<std::setw(10)<<itVV->second;
				itVV++;
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
		new nanogui::MessageDialog(
			this, nanogui::MessageDialog::Type::Information, "Compute Vertex Valences", "Done! Check console output."
		);
	});

	auto extractShellsBtn = new nanogui::Button(mainWindow, "Extract Shells");
	extractShellsBtn->setCallback([this]() {
		auto count = ExtractShells(polymesh, faceIdProperty);
		std::stringstream ss;
		ss << "The mesh has " << count << " shells.";
		new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Shell Extraction",
			ss.str());

		ColorMeshFromIds();
	});

	auto noiseBtn = new nanogui::Button(mainWindow, "Add Noise");
	noiseBtn->setCallback([this]() { AddNoise(polymesh, bboxProperty); MeshUpdated(); });

	nanogui::TextBox* txtSmoothingIterations;
	auto sldSmoothingIterations = nse::gui::AddLabeledSlider(mainWindow, "Smoothing Iterations", std::make_pair(1, 100), 20, txtSmoothingIterations);
	sldSmoothingIterations->setCallback([this, txtSmoothingIterations](float value)
	{
		smoothingIterations = (unsigned int)std::round(value);
		txtSmoothingIterations->setValue(std::to_string(smoothingIterations));
	});
	sldSmoothingIterations->callback()(sldSmoothingIterations->value());

	sldSmoothingStrength = nse::gui::AddLabeledSliderWithDefaultDisplay(mainWindow, "Smoothing Strength", std::make_pair(0.0f, 1.0f), 0.1f, 2);

	
	auto smoothLBtn = new nanogui::Button(mainWindow, "Uniform Laplacian Smoothing");
	smoothLBtn->setCallback([this]() {
		for (unsigned i=0; i<smoothingIterations; i++)
			SmoothUniformLaplacian(polymesh, sldSmoothingStrength->value());
		MeshUpdated();
	});

	auto smoothLBBtn = new nanogui::Button(mainWindow, "Cotangent Laplacian Smoothing");
	smoothLBBtn->setCallback([this]() {
		//Triangulate the mesh if it is not a triangle mesh
		for (auto f : polymesh.faces()) if (polymesh.valence(f) > 3)
		{
			std::cout << "Triangulating mesh." << std::endl;
			polymesh.triangulate();
			MeshUpdated();
			break;
		}
		for (unsigned i=0; i<smoothingIterations; i++)
			SmoothCotanLaplacian(polymesh, sldSmoothingStrength->value());
		MeshUpdated();
	});

	nanogui::TextBox* txtStripificationTrials;
	auto sldStripificationTrials = nse::gui::AddLabeledSlider(mainWindow, "Stripification Trials", std::make_pair(1, 50), 20, txtStripificationTrials);
	sldStripificationTrials->setCallback([this, txtStripificationTrials](float value)
	{
		stripificationTrials = (unsigned int)std::round(value);
		txtStripificationTrials->setValue(std::to_string(stripificationTrials));
	});
	sldStripificationTrials->callback()(sldStripificationTrials->value());

	auto stripifyBtn = new nanogui::Button(mainWindow, "Extract Triangle Strips");
	stripifyBtn->setCallback([this]() {
		//Triangulate the mesh if it is not a triangle mesh
		for (auto f : polymesh.faces())
		{
			if (polymesh.valence(f) > 3)
			{
				std::cout << "Triangulating mesh." << std::endl;
				polymesh.triangulate();
				MeshUpdated();
				break;
			}
		}

		auto count = ExtractTriStrips(polymesh, faceIdProperty, stripificationTrials);
		std::stringstream ss;
		ss << "The mesh has " << count << " triangle strips.";
		new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Stripification",
			ss.str());

		ColorMeshFromIds();
	});

	shadingBtn = new nanogui::ComboBox(mainWindow, { "Flat Shading", "Smooth Shading" });

	performLayout();
}

void Viewer::ColorMeshFromIds()
{
	//Set face colors
	for (auto f : polymesh.faces())
	{
		auto shell = polymesh.property(faceIdProperty, f);
		if (shell < 0)
			polymesh.property(faceColorProperty, f) = Eigen::Vector4f(0, 0, 0, 1);
		else
		{
			auto& color = segmentColors[shell % segmentColorCount];
			polymesh.property(faceColorProperty, f) = Eigen::Vector4f(color[0], color[1], color[2], 1);
		}
	}
	hasColors = true;
	MeshUpdated();
}

void Viewer::MeshUpdated(bool initNewMesh)
{
	if (initNewMesh)
	{
		hasColors = false;

		//calculate the bounding box of the mesh
		nse::math::BoundingBox<float, 3> bbox;
		for (const auto v : polymesh.vertices())
			bbox.expand(ToEigenVector(polymesh.point(v)));
		polymesh.property(bboxProperty) = bbox;

		// calculate average edge length
		polymesh.property(avgEdgeLengthProperty) = .0f;
		for (const auto e : polymesh.edges())
			polymesh.property(avgEdgeLengthProperty) +=
				(polymesh.point(e.v1()) - polymesh.point(e.v0())).length();
		polymesh.property(avgEdgeLengthProperty) /= polymesh.n_edges();
		camera().FocusOnBBox(bbox);
	}	

	if (hasColors)
		renderer.UpdateWithPerFaceColor(faceColorProperty);
	else
		renderer.Update();
}

void Viewer::drawContents()
{
	glEnable(GL_DEPTH_TEST);

	Eigen::Matrix4f view, proj;
	camera().ComputeCameraMatrices(view, proj);

	renderer.Render(view, proj, shadingBtn->selectedIndex() == 0);
}
