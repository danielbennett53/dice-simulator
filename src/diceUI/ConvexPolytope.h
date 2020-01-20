#pragma once

#include "ObjReader.h"

#include <vector>
#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>

namespace geometry {

template <typename T>
inline void remove_at(std::vector<T>& v, typename std::vector<T>::size_type n)
{
    std::swap(v[n], v.back());
    v.pop_back();
}

class Vertex {
public:
    Vertex(Eigen::Vector3d point) : pos_(point) {};
    ~Vertex() {
        for (auto c : connections_)
            static_cast<std::shared_ptr<Vertex>>(c)->setUpdate(true);
    }

    bool operator==(const Vertex& v) const {
        static const double eps = 1e-6;
        return (pos_ - v.getPos()).norm() < eps;
    }

    void connect(const std::shared_ptr<Vertex>& vtx)
    {
        connections_.remove_if([&vtx](std::weak_ptr<Vertex> v) {
            return (v.expired() || (static_cast<std::shared_ptr<Vertex>>(v) == vtx));});
        connections_.emplace_back(vtx);
        update_connections_ = false;
    }

    Eigen::Vector3d operator-(const Vertex& v) const { return (pos_ - v.getPos()); }

    Eigen::Vector3d operator+(const Vertex& v) const { return (pos_ + v.getPos()); }

    Eigen::Vector3d operator-(void) const { return -1*pos_; }

    double& operator[](int i) { return pos_[i]; }
    double operator[](int i) const { return pos_[i]; }

    Eigen::Vector3d getPos() const { return pos_; }
    void setUpdate(bool update) { update_connections_ = update; }

    const std::list<std::weak_ptr<Vertex>>& getConnections() {
        if (update_connections_) {
            connections_.remove_if([](std::weak_ptr<Vertex> v){ return (v.expired()); });
            update_connections_ = false;
        }
        return connections_;
    }

private:
    Eigen::Vector3d pos_;
    // Pointers to every other connected vertex
    std::list<std::weak_ptr<Vertex>> connections_;
    // Flag to indicate whether the connections list is valid
    bool update_connections_{false};
};

class Edge {
public:
    Edge(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> end) :
        startPoint_(start), endPoint_(end)
    {
        diff_ = *end - *start;
        end->connect(start);
        start->connect(end);
    }

    ~Edge() {}
    std::shared_ptr<Vertex> startPoint_;
    std::shared_ptr<Vertex> endPoint_;
    Eigen::Vector3d diff_;

    void reverse()
    {
        auto temp = startPoint_;
        startPoint_ = endPoint_;
        endPoint_ = temp;
        diff_ = -diff_;
    }

    bool operator==(const Edge& e) const {
        return (*startPoint_ == *e.startPoint_) &&
               (*endPoint_ == *e.endPoint_);
    }
    Edge operator-(void) const { return Edge(endPoint_, startPoint_); }
    Eigen::Vector3d operator*(const Edge& e) const { return diff_.cross(e.diff_); }
};

class Face {
public:
    Face(const Edge& e1, const Edge& e2, const Edge& e3) :
        edges_({e1, e2, e3}) { normal_ = (e1*e2).normalized(); }

    void reverse()
    {
        edges_ = {edges_[2], edges_[1], edges_[0]};
        normal_ = -normal_;
    }

    Face operator-(void) const {return Face(edges_[2], edges_[1], edges_[0]); }

    std::vector<Edge> edges_;
    Eigen::Vector3d normal_;
};


class ConvexPolytope
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
    void draw();

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

    const Eigen::Vector3d& getCentroid() {
        if (centroid_valid_)
            return centroid_;
        updateCentroid();
        return centroid_;
    }

    double getRadius() {
        if (centroid_valid_)
            return radius_;
        updateCentroid();
        return radius_;
    }

    std::vector<Face> faces_;

private:
    // Define mesh centroid and largest convex dimension for intersection culling
    Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
    bool centroid_valid_ = false;
    double radius_ = 0.0;    
    std::vector<std::weak_ptr<Vertex>> vertices_;
    void updateCentroid();
};


typedef struct  {
    Eigen::Vector3d position;
    Eigen::Vector2d texCoords;
} drawVertex;


class OGLPolytope : ConvexPolytope
{
public:
    OGLPolytope(const ObjReader& obj);

    void draw();

private:
    std::vector<drawVertex> drawVertices_;
    std::vector<int> drawIndices_;

    // Render data
    QOpenGLVertexArrayObject VAO_;
    QOpenGLBuffer VBO_, EBO_;
    std::shared_ptr<QOpenGLTexture> tex_;
};

} // namespace geometry
