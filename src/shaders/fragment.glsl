#version 450 core

out vec4 FragColor;
in vec2 tex_coord;

uniform sampler2D tex;

void main()
{
    FragColor = texture2D(tex, tex_coord);
}
