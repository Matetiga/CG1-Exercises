#version 330 core

in vec4 position;
in vec4 normal;
in vec4 color;
in vec2 texCoords;

out vec4 normalViewSpace;
out vec4 posViewSpace;
out vec4 vertexColor;
out vec2 vertexTexCoords;

uniform mat4 view;
uniform mat4 proj;

void main()
{
	posViewSpace = view * position;
	normalViewSpace = view * normal;

	vertexColor = color;
	vertexTexCoords = texCoords;
	
	gl_Position = proj * posViewSpace;
}