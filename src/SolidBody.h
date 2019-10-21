#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Geometry>
#include "Mesh.h"


class SolidBody {

public:
    explicit SolidBody(Mesh::meshType mesh_idx, double density = 1.0);
    SolidBody(const std::string& objFile, double density = 1.0);
    void step();
    void updatePosition();

    Eigen::Transform<double, 3, Eigen::Affine> tf_;
    Eigen::Matrix<double, 6, 1> vel_;
    Mesh::meshType mesh_idx_;

private:
    // Vertices defined relative to COM
    std::vector<Eigen::Vector3d> vertices_;
    // Vector of vertex numbers that make up a specific face
    std::vector<std::vector<unsigned int>> faces_;

    double Ts_ = 0.001;

    Eigen::Vector3d COM_;
    Eigen::Quaterniond orientation_;

    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Vector3d COM_offset_;

    void calculatePhysicalProperties(float density);
};

