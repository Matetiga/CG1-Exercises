#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

in vec4 fragment_color;
in vec4 point_position;

out vec4 color;

uniform vec2 c;
uniform float m;


void main(void)
{
	/**** Begin of tasks ***
	 - 2.2.5
	 Implement the pseudo-code for calculating the julia fractal at a point.
	 For this point you can just use the X- and Y-component of the fragment
	 position in model space, which you can receive from the vertex shader
	 via another "in" variable. */

	vec2 z = point_position.xy * m;

	int n = 200; // n -> i_max
	int i;

	for (i=1;i<=n;i++) {
		float x = (z.x * z.x - z.y * z.y) + c.x;
		float y = (z.y * z.x + z.x * z.y) + c.y;

		if (x*x+y*y > 4.0) break;

		z.x = x;
		z.y = y;	
	}
	
	float alpha = 0.0;
	if (i<n) {
		alpha = float(i) / float(n);
	}

	color = vec4(alpha, alpha, alpha, alpha) * 10.0 * fragment_color;

	/**** End of tasks ***/
}
