
#include <GLFW/glfw3.h>
#include <nlopt.hpp>
#include <math.h>
#include "Visualizer.h"
#include <vector>
#include "Mesh.h"
#include <memory>
#include "Dice.h"
#include <iostream>
#include "glm/gtc/matrix_transform.hpp"
#include "glm/glm.hpp"
#include "SolidBody.h"

Eigen::VectorXd phi(Eigen::VectorXd f)
{
    if (f(0) < 0)
        f(0) = 0;

//    if ((0.2*0.2*f(0)*f(0) - (f(1)*f(1) + f(2)*f(2))) < 0) {
//        auto x = 0.2*f(0) * f(1)/sqrt(pow(f(1), 2) + pow(f(2), 2));
//        auto y = 0.2*f(0) * f(2)/sqrt(pow(f(1), 2) + pow(f(2), 2));
//        f(1) = x;
//        f(2) = y;
//    }
    double mu = 0.2;
    if ((mu*f(0) - f(1) < 0) || (mu*f(0) + f(1) < 0)) {
        f(1) = mu*f(0) * (f(1) < 0 ? -1 : 1);
    }
    if ((mu*f(0) - f(2) < 0) || (mu*f(0) + f(2) < 0)) {
        f(2) = mu*f(0) * (f(2) < 0 ? -1 : 1);
    }

    return f;
}

double myfunc(unsigned int n, const double *x, double *grad, void *my_func_data)
{
    Eigen::MatrixXd A(3,3);
    A << -1,0,-4,
            2,1,0,
            5,-6,1;
    Eigen::VectorXd b(3);
    b << 0,0,0;
    Eigen::VectorXd X(3);
    X << x[0], x[1], x[2];

    if (grad) {
        Eigen::VectorXd g = 0.5*(A + A.transpose())*X + b;
        grad[0] = g(0);
        grad[1] = g(1);
        grad[2] = g(2);
    }


    double out = X.dot(A*X) + b.dot(X);
    return out;
}

double friction_cone_constraint(unsigned int n, const double *x, double *grad, void *data)
{
    double mu = 0.2;
    if (grad) {
        grad[0] = -2*mu*mu*x[0];
        grad[1] = 2*x[1];
        grad[2] = 2*x[2];
    }

    return (x[1]*x[1] + x[2]*x[2] - mu*mu*x[0]*x[0]);
}

int main()
{

    double lb[3] = {0, -HUGE_VAL, -HUGE_VAL};
    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, 3);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_min_objective(opt, myfunc, NULL);
    nlopt_add_inequality_constraint(opt, friction_cone_constraint, NULL, 1e-8);
    nlopt_set_xtol_rel(opt, 1e-4);
    double x[3] = {0, 0, 0};
    double minf;
    if (nlopt_optimize(opt, x, &minf) < 0) {
        std::cout << "failed" << std::endl;
    }
    else {
        std::cout << "Min: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
        std::cout << "Cost: " << minf << std::endl;
    }
//    Visualizer vis(1024, 1500);
//
//    // Initialize floor vertices
//    std::vector<Vertex> floor_vertices = {
//        {.position = {-100.0f, 0.0f, -100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {0.0, 0.0},
//         .texNum = 0},
//        {.position = {-100.0f, 0.0f,  100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {0.0, 10.0},
//         .texNum = 0},
//        {.position = {100.0f, 0.0f,  -100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {10.0, 0.0},
//         .texNum = 0},
//        {.position = {100.0f, 0.0f, 100.0f},
//         .color = {1.0f, 1.0f, 1.0f},
//         .texCoords = {10.0, 10.0},
//         .texNum = 0}
//    };
//    std::vector<unsigned int> floor_indices = {
//            0, 1, 2,
//            1, 2, 3
//    };
//    Texture floor_texture;
//    floor_texture.id = 0;
//    floor_texture.file_paths.emplace_back(PROJECT_DIR "/resources/checkerboard.png");
//
//
//    vis.meshes_.emplace_back(std::make_shared<Mesh>(floor_vertices, floor_indices, floor_texture, 0));
//    auto die_mesh = Dice::generateMesh(Dice::d6, 1);
//    vis.meshes_.emplace_back(die_mesh);
//    die_mesh->updateModelTF(Eigen::Vector3d(0, 5, 0), Eigen::Quaterniond(0,0,0,0));
//    die_mesh = Dice::generateMesh(Dice::d6, 1);
//    vis.meshes_.emplace_back(die_mesh);
//
//    die_mesh->updateModelTF(Eigen::Vector3d(5, 5, 0), Eigen::Quaterniond(0,0,0,0));
//
//    Eigen::MatrixXd A(3,3);
//    A << 11,2,-2,
//         4,5,6,
//         0,-4,9;
//    Eigen::VectorXd b(3);
//    b << -2,-4,0.5;
//    std::function<Eigen::VectorXd (Eigen::VectorXd)> in = phi;
//    auto f = SolidBody::pgsSolve(A, b, in);
//
//    std::cout << f << std::endl;

    return 0;

//    while(!glfwWindowShouldClose(vis.window_))
//    {
//        auto time = (float)glfwGetTime();
//        auto model = glm::mat4(1.0f);
//        model = glm::translate(model, glm::vec3(0.0f, 5.0f, 0.0f));
//        model = glm::rotate(model, time, glm::vec3(0.0f, 1.0f, 1.0f));
//        vis.meshes_[1]->updateModelTF(model);
//        model = glm::translate(model, glm::vec3(0.0f, 5.0f, 5.0f));
//        vis.meshes_[2]->updateModelTF(model);
//
//
//        vis.draw();
//    }
//    return 0;
}
