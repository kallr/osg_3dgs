#version 430 core

in vec4 PassColor;
in vec2 PassPosition;

out vec4 FragColor;

void main() {
    FragColor = PassColor;
}
