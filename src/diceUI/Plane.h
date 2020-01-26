#pragma once

#include "ObjReader.h"
#include "Shape.h"


namespace geometry {

class Plane : Shape
{
public:
    Plane(const ObjReader& obj);
    Plane(const Eigen::Vector3d normal, const Eigen::Vector3d center,
          const Eigen::Vector3d x_axis, double width, double height,
          const std::string tex_file = "");

    void draw() override { render_data_->draw(); };

    // Checks if point is within the specified distance of the bounding sphere
    bool interiorPoint(Eigen::Vector3d& point, double radius) override {
        return ((point - getCentroid()).dot(normal_) < (radius));
    };

    // Support function for GJK/EP algorithm
    bool support(Eigen::Vector3d& vector, Eigen::Vector3d& out_point) const override;

    Eigen::Vector3d normal_;
    std::vector<Eigen::Vector3d> vertices_;
private:
    double radius_ = 0;
    void updateCentroid() override;
};

} // Namespace geometry
