#include "visualizer.h"
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#include <iostream>
#include <cmath>
//#include "shader.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


Visualizer::Visualizer(unsigned int height, unsigned int width)
{
    // Initialize GLFW window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window with specified size
    win_width_ = width;
    win_height_ = height;
    window_ = glfwCreateWindow(width, height, "Dice Simulator", nullptr, nullptr);

    // Check for successful creation
    if (window_ == nullptr)
    {
        std::cout << "Failed to create simulator window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window_);

    // Load openGL/GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Initialize scene with floor
    float vertices[] = {
            -100.0f, 0.0f, -100.0f,  0.0f, 0.75f, 0.75f,
            -100.0f, 0.0f,  100.0f,  0.0f, 0.75f, 0.75f,
             100.0f, 0.0f,  100.0f,  0.0f, 0.75f, 0.75f,
             100.0f, 0.0f, -100.0f,  0.0f, 0.75f, 0.75f
    };

    unsigned int indices[] = {
            0, 1, 2,
            0, 2, 3
    };

    // Generate buffers
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &EBO_);

    // Bind buffers
    glBindVertexArray(VAO_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Define position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    //color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    // Bind callbacks
    this->bindCallbacks();

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Initialize camera view
    cam_view_ = glm::mat4(1.0f);
}

Visualizer::~Visualizer()
{
    // Unbind buffers and terminate glfw
    glDeleteVertexArrays(1, &VAO_);
    glDeleteBuffers(1, &VBO_);
    glDeleteBuffers(1, &EBO_);
    glfwTerminate();
}

void Visualizer::updateCameraView(double cursorPosX, double cursorPosY)
{
    // Store last position of mouse
    static double lastPosX, lastPosY = 0;
    // Difference of current and last position
    double diffX = cursorPosX - lastPosX;
    double diffY = cursorPosY - lastPosY;

    // Rotations about X and Y axes
    static double xAng, yAng = 0;

    // Vectors for defining camera view
    static glm::vec3 cam_target = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 cam_pos, up, right;

    // Rotation using middle mouse button
    if(glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        xAng += diffX / 5;
        yAng += diffY / 5;
    }

    // Keep angles within +- 180 degrees
    if (yAng > 180)
        yAng -= 360;
    if (yAng < -180)
        yAng += 360;
    if (xAng > 180)
        xAng -= 360;
    if (xAng < -180)
        xAng += 360;

    // Flip up vector when y angle goes beyond +/- 90
    if (yAng < 90 && yAng > -90) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
        up = glm::vec3(0.0f, -1.0f, 0.0f);
    }

    // Spherical coordinates calculation for rotation:
    // Radius of projected circle
    auto r = (float) cos(yAng * M_PI / 180.0f);
    // Calculate x, y, z coords of camera position
    cam_pos = glm::vec3(r * sin(xAng * M_PI / 180.0f),
                        (float) sin(yAng * M_PI / 180.0f),
                        r * cos(xAng * M_PI / 180.0f));

    // Move camera target with right-click
    if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        right = glm::normalize(glm::cross(cam_pos, up));
        cam_target += (right * (float) (diffX/200)) +
                  (glm::cross(right, cam_pos) * (float) (diffY/200));
    }

    // Calculate absolute camera position
    cam_pos = cam_pos * cam_radius_ + cam_target;

    // Calculate camera view
    cam_view_ = glm::lookAt(cam_pos, cam_target, up);

    // Propagate mouse position history
    lastPosX = cursorPosX;
    lastPosY = cursorPosY;
}

void Visualizer::bindCallbacks()
{
    auto size_cb_fun = [this] (GLFWwindow* window, int width, int height)
            { this->windowResizeCallback(window, width, height); };
    glfwSetFramebufferSizeCallback(window_,
            *(GLFWframebuffersizefun*) (void*) &size_cb_fun);

    auto scroll_cb_fun = [this] (GLFWwindow* window, double x, double y)
            { this->scrollCallback(window, x, y); };
    glfwSetScrollCallback(window_,
            *(GLFWscrollfun*) (void*) &scroll_cb_fun);
}


void Visualizer::windowResizeCallback(GLFWwindow* window, int width, int height)
{
    win_height_ = (unsigned int) height;
    win_width_ = (unsigned int) width;
    glViewport(0, 0, width, height);
}

void Visualizer::scrollCallback(GLFWwindow* window, double x, double y)
{
    cam_radius_ -= y/10;
}