#pragma once

//#include "../Mesh.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "Shape.h"
// Uncomment to build python module from code
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
//#include <pybind11/eigen.h>

namespace geometry {

typedef struct {
    Eigen::Vector3d point;
    std::vector<int> connectedFaces;
    std::vector<int> connectedVertices;
} vertex;

double rayIntersectsTriangle(const Eigen::Vector3d &rayOrigin,
                             const Eigen::Vector3d &rayVector,
                             const std::vector<Eigen::Vector3d> &verts);

std::vector<Eigen::Vector3d> triTriIntersection3d(std::vector<Eigen::Vector3d> &A,
                                                  std::vector<Eigen::Vector3d> &B);

Eigen::Vector3d support(const std::vector<vertex>& shape, const Eigen::Vector3d& D,
                        int startingIdx = 0);

void sortCCW(std::vector<Eigen::Vector2d> &points);

std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> &points);

double polygonArea(std::vector<Eigen::Vector2d>& points);

std::vector<Eigen::Vector2d> planeProjection(const std::vector<Eigen::Vector3d> &points,
                                             const Eigen::Vector3d &normal);

bool GJKIntersection(const Shape& s1, const Shape& s2);

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
