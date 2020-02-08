#pragma once

#include "ObjReader.h"
#include "Shape.h"


namespace geometry {

class Plane : public Shape
{
public:
    Plane(const ObjReader& obj);
    Plane(const Eigen::Vector3d normal, const Eigen::Vector3d center,
          const Eigen::Vector3d x_axis, double width, double height,
          const std::string tex_file = "");

    void draw() override { render_data_->draw(); };

    bool isectPossible(const Eigen::Vector3d& point, double radius) const override {
        return ((point - getCentroid()).dot(normal_) < (radius));
    };

    // Support function for GJK/EP algorithm
    Eigen::Vector3d support(const Eigen::Vector3d& dir) const override;

    bool rayIntersection(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                         Eigen::Vector3d& intersectionPoint) override;

    Eigen::Vector3d normal_;
    std::vector<Eigen::Vector3d> vertices_;
private:
    double radius_ = 0;
    void updateCentroid() override;
};

} // Namespace geometry
