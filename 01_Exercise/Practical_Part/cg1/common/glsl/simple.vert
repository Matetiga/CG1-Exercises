#version 330 core

in vec4 position;

uniform mat4 mvp;

void main()
{
	gl_Position = mvp * position;
}