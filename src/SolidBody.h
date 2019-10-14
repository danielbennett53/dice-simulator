#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Geometry>
#include "Mesh.h"


class SolidBody {

public:
    explicit SolidBody(Mesh &mesh, double density = 1.0);
    SolidBody(const std::string& objFile, double density = 1.0);
    void step();
    void updatePosition();
    void draw(Shader shader);
    Eigen::Vector3d COM_;
    Eigen::Quaterniond orientation_;
    Eigen::Matrix<double, 6, 1> vel_;

private:
    // Vertices defined relative to COM
    std::vector<Eigen::Vector3d> vertices_;
    // Vector of vertex numbers that make up a specific face
    std::vector<std::vector<unsigned int>> faces_;

    double Ts_ = 0.001;

    std::shared_ptr<Mesh> mesh_;

    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Vector3d COM_offset_;

    void calculatePhysicalProperties(float density);
};

