#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

in vec4 fragment_color;
in vec4 model_position;

out vec4 color;

uniform vec2 juliaC;
uniform float juliaZoom;


void main(void)
{
	/**** Begin of tasks ***
	 - 2.2.5
	 Implement the pseudo-code for calculating the julia fractal at a point.
	 For this point you can just use the X- and Y-component of the fragment
	 position in model space, which you can receive from the vertex shader
	 via another "in" variable. */

	vec2 z = model_position.xy * juliaZoom;

	int imax = 200;
	int i;
	for (i=1;i<=imax;i++) {
		float x = (z.x * z.x - z.y * z.y) + juliaC.x;
		float y = (z.y * z.x + z.x * z.y) + juliaC.y;

		if (x*x+y*y > 4.0) break;

		z.x = x;
		z.y = y;	
	}
	
	float alpha = 0.0;
	if (i<imax) {
		alpha = float(i) / float(imax);
	}

	color = vec4(alpha, alpha, alpha, alpha) * 10.0 * fragment_color;
	//color = fragment_color;

	/**** End of tasks ***/
}
