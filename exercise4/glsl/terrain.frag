#version 330 core
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
#define M_PI 3.1415926535897932384626433832795


in vec3 tangent;
in vec3 binormal;
in vec3 normal;
in vec4 worldPos;
out vec4 color;


uniform sampler2D grassTexture;
uniform sampler2D rockTexture;
uniform sampler2D roadColorTexture;
uniform sampler2D roadSpecularMap;
uniform sampler2D roadNormalMap;
uniform sampler2D alphaMap;
uniform sampler2D backgroundTexture;

uniform vec3 cameraPos;
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
	// this uses the camera position which causes the error that the texture follows the camera
	//	return texture(background, gl_FragCoord.xy / screenSize);
	// the screenSize makes the terrain to pixelated

	
	float degr = 40.0;
	float omega = acos(dot(vec3(1.0,0.0,1.0) , normal)); // in radians

	// both conditions make sure the rock Texture is active on every direction,
	// otherwise (-1, 0, -1) view direction would be only made of grass 
	vec4 endTexColor ;
	if(omega < degr*M_PI / 180 ||M_PI - omega < degr*M_PI/180 ) // the angle between the normal and the slope should be greater than 70 degrees
	{
		endTexColor = texture(rockTexture, worldPos.xz / 25.5);
	} else{
		endTexColor = texture(grassTexture, worldPos.xz / 25.5);
	}

	return endTexColor;
	//return mix(backgroundColor, endTexColor, fogFactor);
}

// this function is taken from learn openGL
float LinearizeDepth(float depth){

	float z = depth * 2.0 - 1.0; // back to NDC 
	// clipping planes are defined in Viewer::Viewer
	float near = 0.1; 
	float far = 1000.0; 
	// inverse transformation to get linear depth. Projection Matrix had non-linearized the depth 
	return (2.0 * near * far) / (far + near - z * (far - near));	
}

// this should remap the value from one range to another
float map(float value, float min1, float max1, float min2, float max2) {
  return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

void main()
{
	//surface geometry
	// vec3 n = vec3(0, 1, 0);
	vec3 dirToViewer = cameraPos - worldPos.xyz;

	//material properties	
	color = vec4(0.6, 0.6, 0.6, 1);
	float specular = texture(roadSpecularMap, worldPos.xz / 255).r ;

	vec4 backgroundColor = getBackgroundColor();
	vec4 road = texture(alphaMap, worldPos.xz / 255);
	vec4 roadColor = texture(roadColorTexture, worldPos.xz / 25.5);
	vec4 roadNorMap  = texture(roadNormalMap, worldPos.xz / 255);

	backgroundColor = mix(backgroundColor, roadColor, road.r);

	// transform normal from tangent space to world space
	//roadNorMap.y = - roadNorMap.y; // invert y component ------------> Check if here or in n_tan!!! (scene looks darker)
	vec3 n_tan = 2.0 * roadNorMap.xyz - 1.0; // map from [0,1] to [-1,1]
	n_tan.y = - n_tan.y; // with this inverted, the scene looks brighter 
	mat3 TBN = mat3(normalize(tangent), normalize(binormal), normal);
	vec3 normal_eye = normalize(TBN * n_tan);
	

	// for fog 
	vec4 background = texture(backgroundTexture, worldPos.xz / 255);
	float dist = length(cameraPos -worldPos.xyz); 
	float fogStart = 500.0f; 
	float fogEnd = 1000.0f;


	vec4 endTerrainColor = backgroundColor * calculateLighting(color, specular, normal_eye, dirToViewer);
	//color = vec4(vec3(depthFragment), 1.0);
	float depth =1.0- LinearizeDepth(gl_FragCoord.z) / 1000.0; // divide by far plane
	float fogValue = clamp((gl_FragCoord.z - 500.0)/1000.0, 0.0, 1.0);
	//depth = map(depth, 0.0, 1.0, 0.0, 1.0);
	//depth = clamp((depth*1000-fogStart)/(fogEnd), 0.0, 1.0);
//	color = vec4(endTerrainColor.xyz * depth, 1.0); 
	color = mix( vec4(0.5, 0.5, 0.5, 1.0), endTerrainColor, depth); 
	//color = mix(vec4(background.xyz, 1.0), endTerrainColor, depth);
	//color = vec4(endTerrainColor.xyz, endTerrainColor.w * depth);

	

}