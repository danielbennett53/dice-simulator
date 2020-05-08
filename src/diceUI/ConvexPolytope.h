#pragma once

#include "ObjReader.h"
#include "Shape.h"

#include <vector>
#include <QTransform>
#include <Eigen/Geometry>
#include <list>
#include <memory>

namespace geometry {

class ConvexPolytope : public Shape
{
public:
    ConvexPolytope(const ObjReader& obj);
    ConvexPolytope(const std::vector<Eigen::Vector3d>& points);
    ConvexPolytope(const std::vector<Face>& faces) : faces_(faces) { updateCentroid(); };

    // Adds vertex to polytope while updating face and edge connections.
    void addVertex(const Eigen::Vector3d& point);

    // Adds vertex to simplex, keeping face_to_keep. Only valid if faces_.size() == 4
    void addVertexSimplex(const Eigen::Vector3d& point, unsigned int face_to_keep);

    // Draws polytope
    void draw(QOpenGLShaderProgram& shader) override;

    bool isectPossible(const Eigen::Vector3d& point, double radius) const override {
        return ((point - getCentroid()).norm() < (radius + getRadius()));
    };

    bool rayIntersection(const Eigen::Vector3d& origin,
                         const Eigen::Vector3d& dir,
                         Eigen::Vector3d& intersectionPoint) override;

    void transform(const Eigen::Isometry3d &tf) override;

    // Support function for GJK/EP algorithm
    Eigen::Vector3d support(const Eigen::Vector3d& vector) const override;

    // Fetches vertices list
    const std::vector<std::shared_ptr<Vertex>> getVertices() {
        std::vector<std::shared_ptr<Vertex>> out;
        unsigned int i = 0;
        while (i < vertices_.size()) {
            if (vertices_[i].expired())
                remove_at(vertices_, i);
            else {
                out.emplace_back(vertices_[i]);
                ++i;
            }
        }
        return out;
    }

    std::vector<Face> faces_;

protected:
    ConvexPolytope() {}
    std::vector<std::weak_ptr<Vertex>> vertices_;
    void updateCentroid() override;
};


class Simplex : public ConvexPolytope
{
public:
    Simplex() : ConvexPolytope() {}
    Simplex(const std::vector<Eigen::Vector3d>& points) : ConvexPolytope(points) {}

    void addVertex(const Eigen::Vector3d& point, unsigned int face_to_keep = 0);

    // Given a simplex, returns the vector of the shortest distance between simplex and origin
    bool nearestSimplex(Eigen::Vector3d& new_dir, unsigned int& nearest_face);
};


} // namespace geometry
