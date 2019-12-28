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
    void setPosition(Eigen::Transform<double, 3, Eigen::Affine> tf);

    Eigen::Transform<double, 3, Eigen::Affine> tf_;
    Eigen::Matrix<double, 6, 1> vel_;
    Mesh::meshType mesh_idx_;
    bool selected_ = false;

private:
    double Ts_ = 0.001;

    Eigen::Vector3d COM_;
    Eigen::Quaterniond orientation_;

    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Vector3d COM_offset_;

    void calculatePhysicalProperties(float density);
};

