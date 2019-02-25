#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <memory>
#include "mesh.h"


class SolidBody {

public:
    SolidBody()
    void updateState(double Ts);
    Mesh::Mesh generateMesh(void);

private:
    // Vertices defined relative to COM
    std::vector<Eigen::Vector3d> vertices_;
    // Vector of vertex numbers that make up a specific face
    std::vector<std::vector<int>> faces_;

    Eigen::Vector3d COM_;
    Eigen::Quaterniond orientation_;
    std::shared_ptr<Mesh::Mesh> mesh_;

    double m_;
    Eigen::Matrix3d I_;
    struct {
        double radius_; // Radius of bounding circle
        double stiffness_;
        double damping_;
    } intersection_properties_;

    void groundIntersection(void);
};

