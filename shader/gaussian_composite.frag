#version 430 core

in vec2 TexCoord;

uniform sampler2D tileOutput;

out vec4 FragColor;

void main()
{
    vec4 color = texture(tileOutput, TexCoord);
    FragColor = color;
}
