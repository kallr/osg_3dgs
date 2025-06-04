#version 430 core
layout (location = 0) in vec2 inPos;

uniform samplerBuffer indexBuffer;
uniform samplerBuffer dataBuffer;

uniform mat4 view;
uniform mat4 proj;
uniform vec2 viewport_size;

out vec4 PassColor;
out vec2 PassPosition;

vec4 get_basis(mat2 sigma) {
    float a = sigma[0][0];
    float b = sigma[0][1];
    float c = sigma[1][0];
    float d = sigma[1][1];

    float tr = a + d;
    float det = a * d - b * c;

    // eigenvalues
    float s = sqrt((tr * tr) - (4 * det));
    float lambda1 = 0.5 * (tr + s);
    float lambda2 = 0.5 * (tr - s);

    // eigenvectors
    const float epsilon = 0.00001;

    vec2 e1 = vec2(1, 0);
    if (abs(c) > epsilon) {
        e1 = vec2(lambda1 - d, c);
    } else if (abs(b) > epsilon) {
        e1 = vec2(b, lambda1 - a);
    }
    e1 = normalize(e1);

    vec2 e2 = vec2(e1.y, -e1.x);

    const float max_size = 32 * 2048;
    lambda1 = min(max_size, lambda1);
    lambda2 = min(max_size, lambda2);

    // basis vectors
    vec2 b1 = sqrt(2 * lambda1) * e1;
    vec2 b2 = sqrt(2 * lambda2) * e2;

    return vec4(b1, b2);
}

void main() 
{
	int instanceAddress = int(texelFetch(indexBuffer,gl_InstanceID).x)*5;		
	vec4 pos      = texelFetch(dataBuffer, instanceAddress+0);
	vec4 color    = texelFetch(dataBuffer, instanceAddress+1);
	vec3 sigma10  = texelFetch(dataBuffer, instanceAddress+2).xyz;
	vec3 sigma20  = texelFetch(dataBuffer, instanceAddress+3).xyz;
	vec3 sigma30  = texelFetch(dataBuffer, instanceAddress+4).xyz;

    mat3 sigma = mat3(sigma10, sigma20, sigma30);
    //
    vec4 u = view*pos;
    u /= u.w;

    float focal =proj[0][0] * viewport_size.x * 0.5;

    mat3 jacobian = mat3(
       focal/u.z, 0.0,         -(focal * u.x) / (u.z * u.z), 
        0.0,         focal/u.z, -(focal * u.y) / (u.z * u.z), 
        0.0,         0.0,         0.0
    );

    // Calculate 2D covariance matrix
    mat3 t = jacobian * mat3(view);
    mat3 sigma_prime = t * sigma * transpose(t);
    mat2 sigma2 = mat2(sigma_prime);  // take upper left

    // Get basis vectors of the splatted 2D Gaussian
    vec4 bases =get_basis(sigma2);
    vec2 b1 = bases.xy;
    vec2 b2 = bases.zw;

    //position in screen space
    vec4 pos2d = proj * u;
    vec2 center = pos2d.xy / pos2d.w;

    gl_Position = vec4(center
        + (inPos.x * b1) / (0.5 * viewport_size)
        + (inPos.y * b2) / (0.5 * viewport_size),
        -1,1 );
    gl_Position.z = pos2d.z / pos2d.w;

    PassColor    = color;
    PassPosition = inPos;	
};
