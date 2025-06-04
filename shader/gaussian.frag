#version 430 core

in vec4 PassColor;
in vec2 PassPosition;

out vec4 FragColor;

void main() {
    int f = 1;
    float A = -dot(f * PassPosition, f * PassPosition);
    if (A < -4.0) discard;
    float B = exp(A) * PassColor.a;
    FragColor = B * vec4(PassColor.rgb, 1);
}
