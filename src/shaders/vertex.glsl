#version 330 core

layout (location = 0) in vec3 pos_in;
layout (location = 1) in vec2 tex_coord_in;

//attribute vec3 pos_in;
//attribute vec2 tex_coord_in;

out vec2 tex_coord;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(pos_in, 1.0);
    tex_coord = tex_coord_in;
}
