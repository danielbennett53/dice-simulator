#pragma once

#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "shaders/Shader.h"
#include <vector>
#include <memory>
#include "Mesh.h"

class Visualizer
{
public:
    // Constructor, takes in window size
    Visualizer(unsigned int height, unsigned int width);

    // Update window
    void draw();

    // Window size
    unsigned int win_width_;
    unsigned int win_height_;
    // Distance from camera position to camera target
    float cam_radius_ = 40.0f;
    // Private class properties
    GLFWwindow* window_;
    // List of meshes to draw
    std::vector<std::shared_ptr<Mesh>> meshes_;

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
};
