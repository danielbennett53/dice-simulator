
#include <GLFW/glfw3.h>
#include <nlopt.hpp>
#include <math.h>
#include "Visualizer.h"
#include <vector>
#include "Mesh.h"
#include <memory>
//#include "Dice.h"
#include <iostream>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/glm.hpp"
#include "SolidBody.h"
#include <unistd.h>
#include <fstream>

int main()
{

    Visualizer vis(1024, 1500);
    vis.meshes_.emplace_back(std::make_shared<Mesh>(PROJECT_DIR "/resources/d4.obj"));
    vis.draw();
//
//    // Initialize floor vertices
//    std::vector<Vertex> floor_vertices = {
//        {.position = {-100.0f, 0.0f, -100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {0.0, 0.0},
//         .texNum = 0},
//        {.position = {-100.0f, 0.0f,  100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {0.0, 10.0},
//         .texNum = 0},
//        {.position = {100.0f, 0.0f,  -100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {10.0, 0.0},
//         .texNum = 0},
//        {.position = {100.0f, 0.0f, 100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {10.0, 10.0},
//         .texNum = 0}
//    };
//    std::vector<unsigned int> floor_indices = {
//            0, 1, 2,
//            1, 2, 3
//    };
//    Texture floor_texture;
//    floor_texture.id = 0;
//    floor_texture.file_paths.emplace_back(PROJECT_DIR "/resources/checkerboard.png");
//
//
//    vis.meshes_.emplace_back(std::make_shared<Mesh>(floor_vertices, floor_indices, floor_texture, 0));
//    auto die_mesh = Dice::generateMesh(Dice::d6, 1);
//    vis.meshes_.emplace_back(die_mesh);
//    auto rigid_body = Dice::generateSolidBody(Dice::d6, die_mesh);
//    rigid_body->COM_ = Eigen::Vector3d(5, 10, 0);
//    rigid_body->vel_ << 1.0, 0, 2.0, 0, 10.0, -5.0;
//    rigid_body->orientation_ = Eigen::AngleAxisd(1, Eigen::Vector3d(1, 1, 0));
//
//    vis.meshes_.push_back(Dice::generateMesh(Dice::d4, 1));
//    auto rigid_body2 = Dice::generateSolidBody(Dice::d4, vis.meshes_[2]);
//    rigid_body2->COM_ = Eigen::Vector3d(-10, 10, 0);
//    rigid_body2->vel_ << 1.0, 2.0, 0.0, 7.0, 10.0, 0.0;
//    rigid_body2->orientation_ = Eigen::AngleAxisd(1, Eigen::Vector3d(0, 1, 1));
//
//
    while(!glfwWindowShouldClose(vis.window_))
    {
////        auto time = (float)glfwGetTime();
//        for (int i=0;i<20;++i) {
//            rigid_body->step();
//            rigid_body2->step();
//        }
//        usleep(10000);
        vis.draw();
    }
    return 0;
}
