#include "visualizer.h"
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#include <iostream>
#include <cmath>
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
    // Set user pointer for callbacks
    glfwSetWindowUserPointer(window_, this);

    // Load openGL/GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }

    shader_ = new Shader(PROJECT_DIR "/src/shaders/vertex.glsl",
                         PROJECT_DIR "/src/shaders/fragment.glsl");

    // Initialize scene with floor
    float vertices[] = {
            -100.0f, 0.0f, -100.0f,  0.0f, 0.75f, 0.75f,
            -100.0f, 0.0f,  100.0f,  0.0f, 0.75f, 0.75f,
             100.0f, 0.0f,  100.0f,  0.0f, 0.75f, 0.75f,
             100.0f, 0.0f, -100.0f,  0.0f, 0.75f, 0.75f,
            -0.5f,  -0.5f,   -0.5f,  1.0f,  0.0f,  0.0f,
            -0.5f,  -0.5f,    0.5f,  1.0f,  0.0f,  0.0f,
            -0.5f,   0.5f,   -0.5f,  0.0f,  1.0f,  0.0f,
            -0.5f,   0.5f,    0.5f,  0.0f,  1.0f,  0.0f,
            0.5f,   -0.5f,   -0.5f,  0.0f,  0.0f,  1.0f,
            0.5f,   -0.5f,    0.5f,  0.0f,  0.0f,  1.0f,
            0.5f,    0.5f,   -0.5f,  1.0f,  0.0f,  1.0f,
            0.5f,    0.5f,    0.5f,  1.0f,  0.0f,  1.0f
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
    glfwSetFramebufferSizeCallback(window_, windowResizeCallback);
    glfwSetScrollCallback(window_, scrollCallback);

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

void Visualizer::update(glm::mat4 die_tf)
{
    this->processInput();
    glClearColor(0.2f, 0.3f, 0.7f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader_->use();
    auto model = glm::mat4(1.0f);
    auto projection = glm::perspective(glm::radians(45.0f),
            ((float)win_width_ / win_height_), 0.1f, 100.0f);

    shader_->setMat4f("model", model);
    shader_->setMat4f("view", cam_view_);
    shader_->setMat4f("projection", projection);

    unsigned int indices[] = {
            0, 1, 2,
            0, 2, 3
    };

    glBindVertexArray(VAO_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, nullptr);

    unsigned int indices_2[] = {  // note that we start from 0!
            4, 5, 6,
            5, 6, 7,
            4, 6, 8,
            6, 8, 10,
            4, 5, 8,
            5, 8, 9,
            5, 7, 9,
            7, 9, 11,
            6, 7, 10,
            7, 10, 11,
            8, 9, 10,
            9, 10, 11
    };

    shader_->setMat4f("model", die_tf);
    shader_->setMat4f("view", cam_view_);
    shader_->setMat4f("projection", projection);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_2), indices_2, GL_STATIC_DRAW);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void Visualizer::processInput()
{
    // Exit when escape is pressed
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window_, true);

    // Get mouse position
    double mouse_x, mouse_y;
    glfwGetCursorPos(window_, &mouse_x, &mouse_y);

    // Update current camera view
    this->updateCameraView(mouse_x, mouse_y);
}

void Visualizer::updateCameraView(double cursorPosX, double cursorPosY)
{
    // Store last position of mouse
    static double lastPosX, lastPosY = 0;
    // Difference of current and last position
    double diffX = cursorPosX - lastPosX;
    double diffY = cursorPosY - lastPosY;

    // Rotations about X and Y axes
    static double xAng;
    static double yAng = 10;

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

void windowResizeCallback(GLFWwindow* window, int width, int height)
{
    auto vis = (Visualizer*) glfwGetWindowUserPointer(window);
    vis->win_height_ = (unsigned int) height;
    vis->win_width_ = (unsigned int) width;
    glViewport(0, 0, width, height);
}

void scrollCallback(GLFWwindow* window, double x, double y)
{
    auto vis = (Visualizer*) glfwGetWindowUserPointer(window);
    vis->cam_radius_ -= y/10;
}