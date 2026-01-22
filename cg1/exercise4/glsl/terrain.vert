#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

in vec4 position;
in vec2 offset;

uniform mat4 mvp;

out vec3 tangent;
out vec3 binormal;
out vec3 normal;
out vec4 worldPos;

//Returns the height of the procedural terrain at a given xz position
float getTerrainHeight(vec2 p);


void main()
{
	// float currentHeight = getTerrainHeight(position.xz + offset.xy);
	float currentHeight = getTerrainHeight(position.xz );
	worldPos = vec4(position.x + offset.x, currentHeight, position.z + offset.y, 1.0);
	gl_Position = mvp * worldPos;


	// calculate the normal
	// first two neighbors
	vec2 neighborOffset = vec2(1.0, 0.0);
	float hightX = getTerrainHeight(worldPos.xz + neighborOffset.xy);
	float hightZ = getTerrainHeight(worldPos.xz + neighborOffset.yx);
	
	// using height difference
	tangent = vec3(1.0 , hightX - currentHeight, 0.0);
	binormal = vec3(0.0 , hightZ - currentHeight, 1.0);

	// Crossproduct of both neighbors 
	//vec3 crossProduct = cross(binormal, tangent);
	//normal = normalize(crossProduct);
	normal = normalize(vec3(-(hightX-currentHeight), 1.0, -(hightZ-currentHeight))); // this is the result of the crosproduct operation
}

//source: https://gist.github.com/patriciogonzalezvivo/670c22f3966e662d2f83
float rand(vec2 c)
{
	return 2 * fract(sin(dot(c.xy ,vec2(12.9898,78.233))) * 43758.5453) - 1;
}

float perlinNoise(vec2 p )
{
	vec2 ij = floor(p);
	vec2 xy = p - ij;
	//xy = 3.*xy*xy-2.*xy*xy*xy;
	xy = .5*(1.-cos(3.1415926 * xy));
	float a = rand((ij+vec2(0.,0.)));
	float b = rand((ij+vec2(1.,0.)));
	float c = rand((ij+vec2(0.,1.)));
	float d = rand((ij+vec2(1.,1.)));
	float x1 = mix(a, b, xy.x);
	float x2 = mix(c, d, xy.x);
	return mix(x1, x2, xy.y);
}

//based on https://www.seedofandromeda.com/blogs/58-procedural-heightmap-terrain-generation
float getTerrainHeight(vec2 p)
{
	float total = 0.0;
	float maxAmplitude = 0.0;
	float amplitude = 1.0;
	float frequency = 0.02;
	for (int i = 0; i < 11; i++) 
	{
		total +=  ((1.0 - abs(perlinNoise(p * frequency))) * 2.0 - 1.0) * amplitude;
		frequency *= 2.0;
		maxAmplitude += amplitude;
		amplitude *= 0.45;
	}
	return 15 * total / maxAmplitude;
}