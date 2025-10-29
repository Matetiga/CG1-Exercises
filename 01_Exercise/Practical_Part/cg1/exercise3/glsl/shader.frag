#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

in vec4 fragment_color;

out vec4 color;



void main(void)
{
	/**** Begin of tasks ***
	 - 2.2.5
	 Implement the pseudo-code for calculating the julia fractal at a point.
	 For this point you can just use the X- and Y-component of the fragment
	 position in model space, which you can receive from the vertex shader
	 via another "in" variable. */

	color = fragment_color;

	/**** End of tasks ***/
}
