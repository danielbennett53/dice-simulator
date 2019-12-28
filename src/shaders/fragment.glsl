#version 450 core

out vec4 FragColor;
in vec2 tex_coord;

uniform sampler2D tex;
uniform vec4 color;

void main()
{
    FragColor = color*texture2D(tex, tex_coord);
}
