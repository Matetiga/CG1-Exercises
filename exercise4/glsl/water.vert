# version 330 core 

in vec4 waterPosition;
in vec2 offset;

out vec4 worldPos;
out vec3 normal;

uniform float time;
uniform mat4 mvp;

vec2 rotate(vec2 v, float angle){
	float s = sin(angle );
	float c = cos(angle);
	vec2 r;
	// formula for 2D rotation
	r.x = c * v.x - s* v.y;
	r.y = s * v.x + c * v.y;
	return r;
}

float calculateHight(vec2 position)
{
	vec2 currentDir = vec2(1.0, 0.0);
	float dx = 0.0;
	float dz = 0.0;
	
	// this values have been fine tuned for this 
	float waveFrequency = 0.15f;
	float waveAmplitude = 1.0f;
	float waveRotation = 1.28;
	float waveSpeed = 1.0f;
	int waveNumber = 32;
	float frequencyDampner = 1.15;
	float amplitudeDampner = 0.77;
	float speedDampner = 0.39;

	float height = 0.0f;
	// 32 will represent the number of waves in the sum
	for(int i = 1; i < waveNumber; i++){
		vec2 D = normalize(currentDir);
		float direction = dot(D, position);
	
		float phase = direction * waveFrequency + time * waveSpeed;  
		float eulerWave = exp(sin(phase)) -1.0 ; // waves with euler formula are pointier at the edge than sin waves
		height += eulerWave * waveAmplitude;

		// we calculate here the derivative (rate of change) which is important for the normal 
		float derivative = waveFrequency * waveAmplitude * eulerWave *cos(phase);
		dx += derivative * D.x;
		dz += derivative * D.y;

		waveFrequency *= frequencyDampner; 
		waveAmplitude *= amplitudeDampner; 
		waveSpeed *= speedDampner;
		currentDir = rotate(currentDir, waveRotation);
		
	}

	// this is the result of doing the cross product between T x B (for efficiency, it is directly calculated)
	normal = normalize(vec3(-dx, 1.0, -dz));
	return height;
}

void main(){

	float height = calculateHight(waterPosition.xz + offset);	
	worldPos = vec4(waterPosition.x + offset.x, height, waterPosition.z + offset.y, 1.0);
	gl_Position =  mvp * worldPos;
}