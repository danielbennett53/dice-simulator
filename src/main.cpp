
#include <GLFW/glfw3.h>
#include <nlopt.hpp>
#include <math.h>
#include "Visualizer.h"
#include <vector>
#include "Mesh.h"
#include <memory>
#include "Dice.h"
#include <iostream>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/glm.hpp"
#include "SolidBody.h"
#include <unistd.h>

int main()
{

    Visualizer vis(1024, 1500);

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
        {.position = {100.0f, 0.0f,  -100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {10.0, 0.0},
         .texNum = 0},
        {.position = {100.0f, 0.0f, 100.0f},
         .color = {1.0f, 1.0f, 1.0f},
         .texCoords = {10.0, 10.0},
         .texNum = 0}
    };
    std::vector<unsigned int> floor_indices = {
            0, 1, 2,
            1, 2, 3
    };
    Texture floor_texture;
    floor_texture.id = 0;
    floor_texture.file_paths.emplace_back(PROJECT_DIR "/resources/checkerboard.png");


    vis.meshes_.emplace_back(std::make_shared<Mesh>(floor_vertices, floor_indices, floor_texture, 0));
    auto die_mesh = Dice::generateMesh(Dice::d6, 1);
    vis.meshes_.emplace_back(die_mesh);
    die_mesh->updateModelTF(Eigen::Vector3d(0, 5, 0), Eigen::Quaterniond(0,0,0,0));
    die_mesh = Dice::generateMesh(Dice::d6, 1);
    vis.meshes_.emplace_back(die_mesh);
    auto rigid_body = Dice::generateSolidBody(Dice::d6, die_mesh);

    die_mesh->updateModelTF(Eigen::Vector3d(5, 5, 0), Eigen::Quaterniond(0,0,0,0));
    rigid_body->COM_ = Eigen::Vector3d(5, 10, 0);
    rigid_body->orientation_ = Eigen::AngleAxisd(.9, Eigen::Vector3d(1, 1, 1));

    while(!glfwWindowShouldClose(vis.window_))
    {
//        auto time = (float)glfwGetTime();
        for (int i=0;i<20;++i) {
            rigid_body->step();
        }
        usleep(1000);

        vis.draw();
    }
    return 0;
}
