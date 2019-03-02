
#include <GLFW/glfw3.h>
#include "Visualizer.h"
#include <vector>
#include "Mesh.h"
#include <memory>
#include "Dice.h"
#include <iostream>

int main()
{
    Visualizer vis(600, 800);

    // Initialize floor vertices
    std::vector<Vertex> floor_vertices = {
        {.position = {-100.0f, 0.0f, -100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {0.0, 0.0},
         .texNum = 0},
        {.position = {-100.0f, 0.0f,  100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {0.0, 10.0},
         .texNum = 0},
        {.position = {100.0f, 0.0f,  100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {10.0, 10.0},
         .texNum = 0},
        {.position = {100.0f, 0.0f, -100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {10.0, 10.0},
         .texNum = 0}
    };
    std::vector<unsigned int> floor_indices = {
            0, 1, 2,
            0, 2, 3
    };
    Texture floor_texture;
    floor_texture.id = 0;
    floor_texture.file_paths.emplace_back(PROJECT_DIR "/resources/checkerboard.png");

    vis.meshes_.emplace_back(std::make_shared<Mesh>(floor_vertices, floor_indices, floor_texture, 0));


    while(!glfwWindowShouldClose(vis.window_))
    {
        vis.draw();
    }
    return 0;
}
