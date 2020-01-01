#pragma once

#include "../Mesh.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>
// Uncomment to build python module from code
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
//#include <pybind11/eigen.h>

namespace geometry {

double rayIntersectsTriangle(const Eigen::Vector3d &rayOrigin,
                             const Eigen::Vector3d &rayVector,
                             const std::vector<Eigen::Vector3d> &verts);

std::vector<Eigen::Vector3d> triTriIntersection3d(std::vector<Eigen::Vector3d> &A,
                                                  std::vector<Eigen::Vector3d> &B);

void sortCCW(std::vector<Eigen::Vector2d> &points);

std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> &points);

double polygonArea(std::vector<Eigen::Vector2d>& points);

std::vector<Eigen::Vector2d> planeProjection(const std::vector<Eigen::Vector3d> &points,
                                             const Eigen::Vector3d &normal);

bool meshIntersection(const Mesh& m1, const Eigen::Transform<double, 3, Eigen::Affine>& tf1,
                      const Mesh& m2, const Eigen::Transform<double, 3, Eigen::Affine>& tf2,
                      std::vector<Eigen::Vector3d>& iSectPoints, Eigen::Vector3d& iSectVector);
inline bool meshIntersection(const Mesh& m1, const Eigen::Transform<double, 3, Eigen::Affine>& tf1,
                      const Mesh& m2, const Eigen::Transform<double, 3, Eigen::Affine>& tf2)
{
    std::vector<Eigen::Vector3d> iS;
    Eigen::Vector3d iSV;
    return meshIntersection(m1, tf1, m2, tf2, iS, iSV);
}

}

// Uncomment to build python module
//PYBIND11_MODULE(Geometry, m) {
//    m.doc() = "Geometry Module";

//    m.def("convexHull2d", &geometry::convexHull2d, pybind11::return_value_policy::copy, "Returns the convex hull from the set of points defined in input");
//    m.def("planeProjection", &geometry::planeProjection, pybind11::return_value_policy::copy, "Returns the projection of the set of points onto the plane defined by normal");
//    m.def("triTriIntersection3d", &geometry::triTriIntersection3d, pybind11::return_value_policy::copy, "Returns intersection of triangle");
//    m.def("polygonArea", &geometry::polygonArea, pybind11::return_value_policy::copy, "Returns the area of the convex hull of the points defined in input");
//}

// Run the following command with the above sections uncommented to build the python module
// g++ -O3 -Wall -shared -std=c++11 -fpic `python3 -m pybind11 --includes` Geometry.cpp -o Geometry`python3-config --extension-suffix`
