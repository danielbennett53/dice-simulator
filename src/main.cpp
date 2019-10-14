
#include <GLFW/glfw3.h>
#include <nlopt.hpp>
#include <math.h>
#include "Environment.h"
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

    Environment env(1024, 1500);
    env.addMesh(PROJECT_DIR "/resources/floor.obj");

    Mesh d4 = Mesh(PROJECT_DIR "/resources/d4.obj");
    Mesh d6 = Mesh(PROJECT_DIR "/resources/d6.obj");
    Mesh d20 = Mesh(PROJECT_DIR "/resources/d20.obj");

    env.addSolidBody(SolidBody(d6));
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    orientation = Eigen::AngleAxisd(1.5, Eigen::Vector3d(1, 0, 1));
    orientation.normalize();
    position << 1.0, 10, 1.0;
    env.getSolidBody(0).COM_ = position;
    env.getSolidBody(0).orientation_ = orientation;

    env.update();

    while(!glfwWindowShouldClose(env.window_))
    {
//        auto time = (float)glfwGetTime();
        for (int i=0;i<20;++i) {
            env.getSolidBody(0).step();
        }
        usleep(100);
        env.update();
    }
    return 0;
}
