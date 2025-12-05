// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Viewer.h"

#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/checkbox.h>

#include <gui/SliderHelper.h>

#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>

#include "glsl.h"


Viewer::Viewer()
	: AbstractViewer("CG1 Exercise 3"), vertex_shader_id(0), fragment_shader_id(0), program_id(0)
{ 
	SetupGUI();

	CreateShaders();
	CreateVertexBuffers();

	modelViewMatrix.setIdentity();
	projectionMatrix.setIdentity();

	camera().FocusOnBBox(nse::math::BoundingBox<float, 3>(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1)));
}

void Viewer::SetupGUI()
{
	auto mainWindow = SetupMainWindow();

	//Create GUI elements for the various options
	chkHasDepthTesting = new nanogui::CheckBox(mainWindow, "Perform Depth Testing");
	chkHasDepthTesting->setChecked(true);

	chkHasFaceCulling = new nanogui::CheckBox(mainWindow, "Perform backface Culling");
	chkHasFaceCulling->setChecked(true);

	sldJuliaCX = nse::gui::AddLabeledSliderWithDefaultDisplay(mainWindow, "JuliaC.X", std::make_pair(-1.0f, 1.0f), 0.45f, 2);
	sldJuliaCY = nse::gui::AddLabeledSliderWithDefaultDisplay(mainWindow, "JuliaC.Y", std::make_pair(-1.0f, 1.0f), -0.3f, 2);
	sldJuliaZoom = nse::gui::AddLabeledSliderWithDefaultDisplay(mainWindow, "Julia Zoom", std::make_pair(0.01f, 10.0f), 1.0f, 2);

	performLayout();
}

// Create and define the vertex array and add a number of vertex buffers
void Viewer::CreateVertexBuffers()
{
	/*** Begin of task 3.2.3 ***
	Fill the positions-array and your color array with 12 rows, each
	containing 4 entries, to define a tetrahedron. */

	// Define 3 vertices for one face
	GLfloat positions[] = {
		// (x,y,z,w)
		0, 1, 0, 1,
		-1, -1, 0, 1,
		1, -1, 0, 1
	};

	// Generate the vertex array 
	glGenVertexArrays(1, &vertex_array_id);
	// after the vertex array object is bound, all subsequent command will use this VAO
	glBindVertexArray(vertex_array_id);

	// Generate a position buffer to be appended to the vertex array
	glGenBuffers(1, &position_buffer_id);
	// Bind the buffer for subsequent settings
	glBindBuffer(GL_ARRAY_BUFFER, position_buffer_id);
	// Supply the position data
	glBufferData(GL_ARRAY_BUFFER, sizeof(positions), positions, GL_STATIC_DRAW);
	// The buffer shall now be linked to the shader attribute
	// "in_position". First, get the location of this attribute in 
	// the shader program
	GLuint vid = glGetAttribLocation(program_id, "in_position");
	
	// Enable this vertex attribute array
	glEnableVertexAttribArray(vid);
	// Set the format of the data to match the type of "in_position"
	glVertexAttribPointer(vid, 4, GL_FLOAT, GL_FALSE, 0, 0);

	/*** Begin of task 3.2.2 (a) ***
	Create another buffer that will store color information. This works nearly
	similar to the code above that creates the position buffer. Store the buffer
	id into the variable "color_buffer_id" and bind the color buffer to the
	shader variable "in_color". */
	GLfloat colors[] = {
		// (r,g,b,a)
		1, 0, 0, 1,
		0, 1, 0, 1,
		0, 0, 1, 1
	};
	glGenVertexArrays(1, &color_buffer_id);
	glGenBuffers(1, &color_buffer_id);
	glBindBuffer(GL_ARRAY_BUFFER, color_buffer_id);
	glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_STATIC_DRAW);
	GLuint color_id = glGetAttribLocation(program_id, "in_color");
	glEnableVertexAttribArray(color_id);
	glVertexAttribPointer(color_id, 4, GL_FLOAT, GL_FALSE, 0, 0);
	
	
	/*** End of task 3.2.2 (a) ***/
	
	

	// Unbind the vertex array to leave OpenGL in a clean state
	glBindVertexArray(0);
}

//Checks if the given shader has been compiled successfully. Otherwise, prints an
//error message and throws an exception.
//  shaderId - the id of the shader object
//  name - a human readable name for the shader that is printed together with the error
void CheckShaderCompileStatus(GLuint shaderId, std::string name)
{
	GLint status;
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &status);

	if (status != GL_TRUE)
	{
		char buffer[512];
		std::cerr << "Error while compiling shader \"" << name << "\":" << std::endl;
		glGetShaderInfoLog(shaderId, 512, nullptr, buffer);
		std::cerr << "Error: " << std::endl << buffer << std::endl;
		throw std::runtime_error("Shader compilation failed!");
	}
}


// this will Open the respective Shader file for CreateShaders
std::string openFile(std::string filename) {
	std::ifstream file(filename, std::ios::binary);
	if (!file.is_open()){
		std::cout << "Could not open Shader file: " << filename << std::endl;
		return nullptr;
	}

	std::stringstream file_buffer;
	// rdbuf is a pointer to the file and "<<" reads the file until end and puts it into the stringstream
	file_buffer << file.rdbuf();
	std::string file_str = file_buffer.str();
	std::cout << file_str << std::endl;
	return file_str;
}

// Read, Compile and link the shader codes to a shader program
void Viewer::CreateShaders()
{
	// Store strings to keep them alive otherwise dangling pointer 
	std::string vertex_str = openFile("../../exercise3/glsl/shader.vert");
	std::string fragment_str = openFile("../../exercise3/glsl/shader.frag");
	// c_str() gives a pointer to the char array of the string until \0
	const char *vertex_content = vertex_str.c_str();
	const char *fragment_content = fragment_str.c_str();

	/*** Begin of task 3.2.1 ***
	Use the appropriate OpenGL commands to create a shader object for
	the vertex shader, set the source code and let it compile. Store the
	ID of this shader object in the variable "vertex_shader_id". Repeat
	for the fragment shader. Store the ID in the variable "fragment_shader_id.
	Finally, create a shader program with its handle stored in "program_id",
	attach both shader objects and link them. For error checking, you can
	use the method "CheckShaderCompileStatus()" after the call to glCompileShader().
	*/
	/*** End of task 3.2.1 ***/

	// The sequence of commands are shown in the vorlesung 
	// These IDs are already defined in the header, therefore only assign 
	program_id = glCreateProgram();

	vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader_id, 1, &vertex_content, nullptr);
	glCompileShader(vertex_shader_id);
	CheckShaderCompileStatus(vertex_shader_id, "Vertex Shader");
	glAttachShader(program_id, vertex_shader_id);

	fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader_id, 1, &fragment_content, nullptr);
	glCompileShader(fragment_shader_id);
	CheckShaderCompileStatus(fragment_shader_id, "Fragment Shader");
	glAttachShader(program_id, fragment_shader_id);

	glLinkProgram(program_id);
	// glValidateProgram(program);
	glUseProgram(program_id);

}

void Viewer::drawContents()
{
	Eigen::Vector2f juliaC(sldJuliaCX->value(), sldJuliaCY->value());
	float juliaZoom = sldJuliaZoom->value();

	//Get the transform matrices
	camera().ComputeCameraMatrices(modelViewMatrix, projectionMatrix);

	// If has_faceculling is set then enable backface culling
	// and disable it otherwise
	if (chkHasFaceCulling->checked())
		glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);

	// If has_depthtesting is set then enable depth testing
	// and disable it otherwise
	if (chkHasDepthTesting->checked())
		glEnable(GL_DEPTH_TEST);
	else
		glDisable(GL_DEPTH_TEST);

	// Activate the shader program
	glUseProgram(program_id);

	/*** Begin of task 3.2.4 (b) ***
	Set the shader variables for the modelview and projection matrix.
	First, find the location of these variables using glGetUniformLocation and
	then set them with the command glUniformMatrix4fv. 
	*/

	// Bind the vertex array 
	glBindVertexArray(vertex_array_id);
	// Draw the bound vertex array. Start at element 0 and draw 3 vertices
	glDrawArrays(GL_TRIANGLES, 0, 3);

	/*** End of task 3.2.4 (b) ***/
	
	// Unbind the vertex array
	glBindVertexArray(0);
	// Deactivate the shader program
	glUseProgram(0);
}
