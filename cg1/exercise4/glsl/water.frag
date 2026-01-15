#version 330 core

in vec3 normal;
in vec4 worldPos;

uniform vec3 cameraPos;

out vec4 color;

void main(){
	vec3 specularLlightPosition = vec3(25.0, 14.0, 1.0); // this has to be checked
	vec4 lightColor = vec4(1.0, 1.0, 1.0, 1.0);
	float specularFactor = 74.7f;
	vec3 lightDirection = vec3(0.0f, 1.0f, 1.0f);
	vec4 waterColor =vec4(0.1, 0.3, 0.7, 0.7);

	vec3 specDir = normalize(specularLlightPosition - worldPos.xyz);
	vec3 viewDir = normalize( cameraPos - worldPos.xyz);
	vec3 halfwayVec = normalize(specDir + viewDir);
	float specAngle = max(dot(halfwayVec, normal), 0.0);
	float specular = pow(specAngle, specularFactor);

	float NdotL = clamp(dot(normal, lightDirection), 0.0, 1.0);

	vec4 endColor = waterColor * (NdotL * 0.9 + 1.0) + specular * lightColor;
	color = vec4(endColor.x, endColor.y, endColor.z, 0.5); // to make it a bit transparent
}
