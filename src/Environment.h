#pragma once

#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "shaders/Shader.h"
#include <vector>
#include <memory>
#include "Mesh.h"
#include "SolidBody.h"

class Environment
{
public:
    // Constructor, takes in window size
    Environment(unsigned int height, unsigned int width);

    // Add body to visualizer
    void addSolidBody(SolidBody body);
    SolidBody& getSolidBody(int idx);

    // Add mesh to visualizer
    void addMesh(Mesh mesh);
    void addMesh(std::string objFile);

    // Update window
    void update();

    // Window size
    unsigned int win_width_;
    unsigned int win_height_;
    // Distance from camera position to camera target
    float cam_radius_ = 40.0f;
    // Private class properties
    GLFWwindow* window_;


private:
    // Private methods
    void processInput();
    void updateCameraView(double cursorPosX, double cursorPosY);
    // Callbacks
    static void windowResizeCallback(GLFWwindow* window, int width, int height);
    static void scrollCallback(GLFWwindow* window, double x, double y);

    // Custom OpenGL shaders
    Shader* shader_;
    // Camera transform
    glm::mat4 cam_view_;

    // List of solid bodies subject to physics
    std::vector<SolidBody> bodies_;

    // List of meshes not subject to physics
    std::vector<std::shared_ptr<Mesh>> meshes_;
};
