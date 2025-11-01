#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

in vec4 in_position;

out vec4 fragment_color;




void main(void)
{
	gl_Position = in_position;
	fragment_color = in_position;

	/* - 2.2.2 (b)
	 * Declare a new "in" variable with the name "in_color". Instead of setting
	 * "fragment_color" to the position, set it to "in_color. */

	/* - 2.2.4 (a)
	 * Declare two new "uniform" variables with the type "mat4" (above the main function)
	 * that store the modelview and projection matrix. To apply the transformations
	 * multiply the value of "in_position" before setting "gl_Position". */

	/* - 2.2.5
	 * The algorithm to calculate the julia fractal needs a position as input.
	 * Declare another "out" variable and set it to the untransformed input
	 * position. */
}