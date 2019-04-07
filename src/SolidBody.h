#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <memory>
#include "Mesh.h"


class SolidBody {

public:
    SolidBody(std::vector<Eigen::Vector3d> vertices,
            std::vector<std::vector<unsigned int>> faces,
            std::shared_ptr<Mesh> mesh);
    void step();
    Eigen::Vector3d COM_;
    Eigen::Quaterniond orientation_;

private:
    // Vertices defined relative to COM
    std::vector<Eigen::Vector3d> vertices_;
    // Vector of vertex numbers that make up a specific face
    std::vector<std::vector<unsigned int>> faces_;

    double Ts_ = 0.0001;


    Eigen::Matrix<double, 6, 1> vel_;
    std::shared_ptr<Mesh> mesh_;

    Eigen::Matrix<double, 6, 6> M_;

    Eigen::Matrix<double, Eigen::Dynamic, 6> getContactJacobian();
    Eigen::Matrix<double, 6, 1> getContactForce();


    void updatePosition();
//    void groundIntersection(void);
};

