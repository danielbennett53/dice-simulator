
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#include <iostream>
#include <cmath>
#include "shader.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/quaternion.hpp"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// Settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

static auto projection = glm::mat4(1.0f);
static auto view = glm::mat4(1.0f);
static float cam_radius = 3;

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);


    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // build and compile our shader program
    // ------------------------------------
    // vertex shader
    Shader ourShader(PROJECT_DIR "/src/shaders/vertex.glsl",
                     PROJECT_DIR "/src/shaders/fragment.glsl");

    float vertices[] = {
            -0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
            -0.5f, -0.5f, 0.5f,   1.0f, 0.0f, 0.0f,
            -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
            -0.5f, 0.5f, 0.5f,    0.0f, 1.0f, 0.0f,
            0.5f, -0.5f, -0.5f,   0.0f, 0.0f, 1.0f,
            0.5f, -0.5f, 0.5f,    0.0f, 0.0f, 1.0f,
            0.5f, 0.5f, -0.5f,    1.0f, 0.0f, 1.0f,
            0.5f, 0.5f, 0.5f,     1.0f, 0.0f, 1.0f,
            -0.5f, -(float) sin(M_PI/3)/2, -(float) sin(M_PI / 3)/2,     0.0f, 1.0f, 0.0f,
            0.5f, -(float) sin(M_PI/3)/2, -(float) sin(M_PI / 3)/2,      1.0f, 0.0f, 0.0f,
            0.0f, -(float) sin(M_PI/3)/2, (float) sin(M_PI / 3)/2,       0.0f, 0.0f, 1.0f,
            0.0f, (float) sin(M_PI/3)/2, 0.0f, 1.0f, 1.0f, 0.0f
    };

    unsigned int indices[] = {  // note that we start from 0!
            0, 1, 2,
            1, 2, 3,
            0, 2, 4,
            2, 4, 6,
            0, 1, 4,
            1, 4, 5,
            1, 3, 5,
            3, 5, 7,
            2, 3, 6,
            3, 6, 7,
            4, 5, 6,
            5, 6, 7
    };

    unsigned int indices_2[] = {
            8, 9, 10,
            8, 9, 11,
            8, 10, 11,
            9, 10, 11
    };
    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    //color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);

    //glViewport(0, 0, 800, 600);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetScrollCallback(window, scroll_callback);

    glEnable(GL_DEPTH_TEST);

    glm::mat4 model = glm::mat4(1.0f);
    //glm::mat4 projection = glm::mat4(1.0f);
    projection = glm::perspective(glm::radians(45.0f),  ((float)SCR_WIDTH / SCR_HEIGHT), 0.1f, 100.0f);

    while(!glfwWindowShouldClose(window))
    {
        processInput(window);
        glClearColor(0.2f, 0.3f, 0.7f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.use();

        //auto time = (float)glfwGetTime();
        model = glm::mat4(1.0f);
        //model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f));
        //model = glm::rotate(model, time, glm::vec3(0.0f, 0.0f, 0.0f));



//        auto modelLoc = glGetUniformLocation(ourShader.ID, "model");
//        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
//        auto viewLoc = glGetUniformLocation(ourShader.ID, "view");
//        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
//        auto projectionLoc = glGetUniformLocation(ourShader.ID, "projection");
//        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
        ourShader.setMat4f("model", model);
        ourShader.setMat4f("view", view);
        ourShader.setMat4f("projection", projection);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        model = glm::translate(model, glm::vec3(2.0f, 0.0f, 0.0f));
        ourShader.setMat4f("model", model);
//        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        model = glm::translate(model, glm::vec3(-4.0f, 0.0f, 0.0f));
        ourShader.setMat4f("model", model);
//        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_2), indices_2, GL_STATIC_DRAW);
        glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_INT, nullptr);



        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glfwTerminate();
    return 0;

}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    projection = glm::perspective(glm::radians(45.0f),  ((float)width / height), 0.1f, 100.0f);
}

void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    static float mouse_x_last, mouse_y_last, x_ang, y_ang = 0;
    static auto target = glm::vec3(0.0f, 0.0f, 0.0f);
    double mouse_x, mouse_y = 0;
    glm::vec3 cam_pos, up, right;
    glfwGetCursorPos(window, &mouse_x, &mouse_y);
    double diff_x = mouse_x - mouse_x_last;
    double diff_y = mouse_y - mouse_y_last;
    // Check mouse button state
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        x_ang += diff_x / 5;
        y_ang += diff_y / 5;
    }

    if (y_ang > 180)
        y_ang -= 360;
    if (y_ang < -180)
        y_ang += 360;
    if (x_ang > 180)
        x_ang -= 360;
    if (x_ang < -180)
        x_ang += 360;

    if (y_ang < 90 && y_ang > -90) {
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
        up = glm::vec3(0.0f, -1.0f, 0.0f);
    }

    auto r = (float) cos(y_ang * M_PI / 180.0f);
    cam_pos = glm::vec3(r * sin(x_ang * M_PI / 180.0f),
                        (float) sin(y_ang * M_PI / 180.0f),
                        r * cos(x_ang * M_PI / 180.0f));
    right = glm::normalize(glm::cross(cam_pos, up));

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        target += (right * (float) (diff_x/200)) +
                (glm::cross(right, cam_pos) * (float) (diff_y/200));
    }

    cam_pos = cam_pos * cam_radius + target;

    view = glm::lookAt(cam_pos,
                       target,
                       up);

    mouse_x_last = (float) mouse_x;
    mouse_y_last = (float) mouse_y;

}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    cam_radius -= yoffset/10;
}