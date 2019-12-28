#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace geometry {

double rayIntersectsTriangle(const Eigen::Vector3d &rayOrigin,
                             const Eigen::Vector3d &rayVector,
                             const std::vector<Eigen::Vector3d> &verts);

bool triTriIntersection3d(const std::vector<Eigen::Vector3d> &V,
                          const std::vector<Eigen::Vector3d> &U,
                          std::vector<Eigen::Vector3d> &intersectionPoints,
                          int &coplanar);

void sortCCW(std::vector<Eigen::Vector2d> &points);

std::vector<Eigen::Vector2d> convexHull2D(std::vector<Eigen::Vector2d> &points);

std::vector<Eigen::Vector3d> planeProjection(const std::vector<Eigen::Vector3d> &points,
                                             const Eigen::Vector3d &normal);

}

PYBIND11_MODULE(Geometry, m) {
    m.doc() = "Geometry Module";

    m.def("convexHull2D", &geometry::convexHull2D, pybind11::return_value_policy::copy, "Returns the convex hull from the set of points defined in input");
    m.def("planeProjection", &geometry::planeProjection, pybind11::return_value_policy::copy, "Returns the projection of the set of points onto the plane defined by normal");
    m.def("triTriIntersection3d", &geometry::triTriIntersection3d, pybind11::return_value_policy::copy, "Returns intersection of triangle");
}
