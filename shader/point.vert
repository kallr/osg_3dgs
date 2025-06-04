#version 430 core

layout (location = 0) in vec2 inPos;
uniform samplerBuffer indexBuffer;
uniform samplerBuffer dataBuffer;

uniform mat4 view;
uniform mat4 proj;
uniform vec2 viewport_size;

out vec4 PassColor;
out vec2 PassPosition;
 

void main() {
	int instanceAddress = int(texelFetch(indexBuffer,gl_InstanceID).x)*5;		
	vec4 pos     = texelFetch(dataBuffer, instanceAddress+0);
	vec4 color   = texelFetch(dataBuffer, instanceAddress+1);
	vec3 sigma10  = texelFetch(dataBuffer, instanceAddress+2).xyz;
	vec3 sigma20  = texelFetch(dataBuffer, instanceAddress+3).xyz;
	vec3 sigma30  = texelFetch(dataBuffer, instanceAddress+4).xyz;

    vec4 p = proj * view * pos;
    gl_Position = vec4(p.xyz / p.w, 1);
    gl_PointSize = 2;
    PassColor = color;

}
