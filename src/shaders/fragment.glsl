#version 330 core

out vec4 FragColor;
in vec3 ourColor;
in vec2 texCoord;

uniform sampler2DArray ourTexture;
uniform int layer = 0;

void main()
{
    FragColor = texture(ourTexture, vec3(texCoord, layer));
}
