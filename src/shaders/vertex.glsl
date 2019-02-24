#version 330 core

layout (location = 0) in vec3 pos_in;
layout (location = 1) in vec3 color_in;
layout (location = 2) in vec2 tex_coord_in;
layout (location = 3) in int tex_num_in;

out vec3 color;
out vec2 tex_coord;
out int tex_num;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(pos_in, 1.0);
    color = color_in;
    tex_coord = tex_coord_in;
    tex_num = tex_num_in;
}
