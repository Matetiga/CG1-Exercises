#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved



out vec4 color;

uniform vec3 cameraPos;


uniform sampler2D background;
uniform vec2 screenSize;

const vec3 dirToLight = normalize(vec3(1, 3, 1));	

//Calculates the visible surface color based on the Blinn-Phong illumination model
vec4 calculateLighting(vec4 materialColor, float specularIntensity, vec3 normalizedNormal, vec3 directionToViewer)
{
	vec4 color = materialColor;
	vec3 h = normalize(dirToLight + directionToViewer);
	color.xyz *= 0.9 * max(dot(normalizedNormal, dirToLight), 0) + 0.1;
	color.xyz += specularIntensity * pow(max(dot(h, normalizedNormal), 0), 50);
	return color;
}

vec4 getBackgroundColor()
{
	return texture(background, gl_FragCoord.xy / screenSize);
}

void main()
{
	//surface geometry
	vec3 n = vec3(0, 1, 0);
	vec3 dirToViewer = vec3(0, 1, 0);

	//material properties	
	color = vec4(0.6, 0.6, 0.6, 1);
	float specular = 0;

	

	//Calculate light
	color = calculateLighting(color, specular, n, dirToViewer);

	
}