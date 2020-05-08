#pragma once

#include <vector>
#include <memory>
#include <QMatrix>
#include <QTransform>
#include <Eigen/Geometry>
#include "ConvexPolytope.h"


class SolidBody {

public:
    explicit SolidBody(std::unique_ptr<geometry::ConvexPolytope> shape, double density = 1.0);
    SolidBody(const std::string& objFile, double density = 1.0);
    void step();
    void updatePosition();
    void setPosition(Eigen::Isometry3d& tf);

    Eigen::Matrix<double, 6, 1> vel_;
    std::unique_ptr<geometry::ConvexPolytope> shape_;
    bool selected_ = false;

private:
    double Ts_ = 0.001;

    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Vector3d COM_offset_;

    void calculatePhysicalProperties(float density);
};

