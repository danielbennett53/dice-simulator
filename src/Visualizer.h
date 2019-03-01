#pragma once

#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "shaders/Shader.h"
#include <vector>

class Visualizer
{
public:
    // Constructor, takes in window size
    Visualizer(unsigned int height, unsigned int width);
    ~Visualizer();

    // Update window
    void update(glm::mat4 die_tf);

    // Window size
    unsigned int win_width_;
    unsigned int win_height_;
    // Distance from camera position to camera target
    float cam_radius_ = 10.0f;
    // Private class properties
    GLFWwindow* window_;

private:
    // Private methods
    void processInput();
    void updateCameraView(double cursorPosX, double cursorPosY);
    static unsigned int loadTexture(const char* image_path);
    // Callbacks
    static void windowResizeCallback(GLFWwindow* window, int width, int height);
    static void scrollCallback(GLFWwindow* window, double x, double y);

    // Custom textures
    std::vector<unsigned int> textures_;
    // Custom OpenGL shaders
    Shader* shader1_;
    Shader* shader2_;
    // OpenGL buffers
    unsigned int VBO_, VAO_, EBO_;
    // Camera transform
    glm::mat4 cam_view_;
};
