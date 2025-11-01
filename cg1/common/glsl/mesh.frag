#version 330 core

in vec4 normalViewSpace;
in vec4 posViewSpace;
in vec4 vertexColor;
in vec2 vertexTexCoords;

out vec4 outColor;

uniform vec4 color;

uniform bool flatShading;
uniform bool perVertexColor;
uniform bool visualizeTexCoords;

void main()
{
	vec3 normal = normalize(normalViewSpace.xyz);
	if(flatShading)
		normal = normalize(cross(dFdx(posViewSpace.xyz), dFdy(posViewSpace.xyz)));
	vec3 toLight = normalize(-posViewSpace.xyz);
	
	float diffuse = max(dot(normal, toLight), 0);
	float specular = pow(diffuse, 20);
	
	if(!gl_FrontFacing)
		diffuse *= 0.5;
	
	vec4 materialColor = perVertexColor ? vertexColor : color;

	if(visualizeTexCoords)
	{
		int cellId = (int(20 * vertexTexCoords.x) + int(20 * vertexTexCoords.y)); 
  		if(cellId % 2 != 0)   			
  			materialColor.xyz *= 0.3; 
	}
	
	outColor = diffuse * materialColor + specular * vec4(0.5, 0.5, 0.5, 0);
	outColor.a = 1;
}