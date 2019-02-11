#pragma once

#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "shader.h"

class Visualizer
{
public:
    // Constructor, takes in window size
    Visualizer(unsigned int height, unsigned int width);
    ~Visualizer();

    // Update window
    void update();


private:
    // Private methods
    void processInput();
    void updateCameraView(double cursorPosX, double cursorPosY);
    // Bind callback functions
    void bindCallbacks();

    // Callbacks
    void windowResizeCallback(GLFWwindow* window, int width, int height);
    void scrollCallback(GLFWwindow* window, double x, double y);

    // Private class properties
    GLFWwindow* window_;
    // Window size
    unsigned int win_width_;
    unsigned int win_height_;
    // Custom OpenGL shaders
    Shader* shader_ = new Shader(PROJECT_DIR "/src/shaders/vertex.glsl",
            PROJECT_DIR "/src/shaders/fragment.glsl");
    // OpenGL buffers
    unsigned int VBO_, VAO_, EBO_;
    // Camera transform
    glm::mat4 cam_view_;
    // Distance from camera position to camera target
    float cam_radius_ = 3.0f;
};

