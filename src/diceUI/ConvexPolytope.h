#pragma once

#include "ObjReader.h"
#include "Shape.h"

#include <vector>
#include <Eigen/Geometry>
#include <list>
#include <memory>

namespace geometry {

class ConvexPolytope : Shape {
public:
    ConvexPolytope(const ObjReader& obj);
    ConvexPolytope(const std::vector<Eigen::Vector3d>& points);
    ConvexPolytope(const std::vector<Face>& faces) : faces_(faces) { updateCentroid(); };

    // Adds vertex to polytope while updating face and edge connections.
    void addVertex(const Eigen::Vector3d& point);

    // Adds vertex to simplex, keeping face_to_keep. Only valid if faces_.size() == 4
    void addVertexSimplex(const Eigen::Vector3d& point, unsigned int face_to_keep);

    // Draws polytope
    void draw() override;

    // Checks if point is within the specified distance of the bounding sphere
    bool interiorPoint(Eigen::Vector3d& point, double radius) override {
        return ((point - getCentroid()).norm() < (radius + getRadius()));
    };

    // Support function for GJK/EP algorithm
    bool support(Eigen::Vector3d& vector, Eigen::Vector3d& out_point) const override;

    // Fetches vertices list
    const std::vector<std::weak_ptr<Vertex>>& getVertices() {
        unsigned int i = 0;
        while (i < vertices_.size()) {
            if (vertices_[i].expired())
                remove_at(vertices_, i);
            else
                ++i;
        }
        return vertices_;
    }

    std::vector<Face> faces_;

private:
    std::vector<std::weak_ptr<Vertex>> vertices_;
    void updateCentroid() override;
};


} // namespace geometry
