
#include <GLFW/glfw3.h>
#include "visualizer.h"


int main()
{
    Visualizer vis(600, 800);

    while(!glfwWindowShouldClose(vis.window_))
    {
        vis.update();
    }
    return 0;
}
