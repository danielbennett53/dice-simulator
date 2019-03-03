#version 330 core

out vec4 FragColor;
in vec3 color;
in vec2 tex_coord;
flat in int tex_num;

uniform sampler2DArray tex;

void main()
{
    FragColor = texture(tex, vec3(tex_coord, tex_num));
}
