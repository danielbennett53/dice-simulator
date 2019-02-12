
#include <GLFW/glfw3.h>
#include "visualizer.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

int main()
{
    Visualizer vis(600, 800);

    while(!glfwWindowShouldClose(vis.window_))
    {
        auto time = (float)glfwGetTime();
        auto model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 2.0f, 0.0f));
        model = glm::rotate(model, time, glm::vec3(0.0f, 1.0f, 1.0f));
        vis.update(model);
    }
    return 0;
}
