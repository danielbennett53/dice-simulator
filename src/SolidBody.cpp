#include "SolidBody.h"
#include <iostream>

SolidBody::SolidBody(std::vector<Eigen::Vector3d> vertices,
                     std::vector<std::vector<unsigned int>> faces,
                     std::shared_ptr<Mesh> mesh)
{

    double m = 0.1; //kg
    M_.topLeftCorner(3,3) = (1.0/6.0) * m * 4.0 * Eigen::Matrix3d::Identity();
    M_.bottomRightCorner(3,3) = m * Eigen::Matrix3d::Identity();
    mesh_ = std::move(mesh);
    vertices_ = std::move(vertices);
    faces_ = std::move(faces);
    intersection_properties_ = {
            .radius = 1.414,
            .stiffness = 3000.0,
            .damping = 500.0
    };
}


void SolidBody::updatePosition(double Ts)
{
    // Rotate orientation vector by current velocity
    Eigen::Quaterniond ang_vel;
    ang_vel.x() = vel_(0) * Ts/2;
    ang_vel.y() = vel_(1) * Ts/2;
    ang_vel.z() = vel_(2) * Ts/2;
    orientation_ *= ang_vel;
    orientation_.normalize();

    // Translate center of mass
    COM_(0) += vel_(3) * Ts;
    COM_(1) += vel_(4) * Ts;
    COM_(2) += vel_(5) * Ts;
}


Eigen::Matrix<double, Eigen::Dynamic, 6> SolidBody::getContactJacobian()
{
    Eigen::Matrix<double, Eigen::Dynamic, 6> Jc;
    long nrows = 0;
    for (const auto &vertex : vertices_) {
        auto tfVertex = orientation_ * vertex * orientation_.inverse() + COM_;

        if (tfVertex(2) <= 0) {
            nrows += 3;
            Jc.resize(nrows, Eigen::NoChange_t());
            Eigen::Matrix3d r_cross;
            r_cross << 0, -tfVertex(2), tfVertex(1),
                       tfVertex(2), 0, -tfVertex(0),
                       -tfVertex(1), tfVertex(0), 0;

            Jc.bottomLeftCorner(3,3) = r_cross;
            Jc.bottomRightCorner(3,3) = Eigen::Matrix3d::Identity();
        }
    }
    return Jc;
}
