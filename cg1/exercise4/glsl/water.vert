# version 330 core 

in vec4 waterPosition;
in vec4 offset;

out vec4 worldPos;

uniform mat4 mvp;

void main(){

	
	worldPos = vec4(waterPosition.x + offset.x, 1.5f, waterPosition.z + offset.y, 1.0);
	gl_Position = mvp * waterPosition;
}
