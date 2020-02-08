#pragma once

#include "ObjReader.h"

#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <list>

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
}; // Vertex


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
}; // Edge


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
}; // Face


class OGLRenderData {
public:
    struct drawVertex {
        Eigen::Vector3d position;
        Eigen::Vector2d texCoords;

        drawVertex(Eigen::Vector3d pos, Eigen::Vector2d tex_coord) : position{pos}, texCoords{tex_coord} {};
    };

    OGLRenderData(const ObjReader& obj);
    OGLRenderData(const std::vector<drawVertex>& vertices,
                  const std::vector<int>& indices,
                  const std::string tex_file) :
                  drawVertices_{vertices}, drawIndices_{indices} {setRenderData(tex_file); };

    ~OGLRenderData() {
        VAO_.destroy();
        VBO_.destroy();
        EBO_.destroy();
//        tex_->destroy();
    }

    std::vector<drawVertex> drawVertices_;
    std::vector<int> drawIndices_;

    void draw();

private:
    void setRenderData(const std::string tex_file);
    // Render data
    QOpenGLVertexArrayObject VAO_;
    QOpenGLBuffer VBO_{QOpenGLBuffer::VertexBuffer}, EBO_{QOpenGLBuffer::IndexBuffer};
    std::shared_ptr<QOpenGLTexture> tex_;
}; // OGLRenderData


class Shape {
public:
    Shape() {}
    ~Shape() {}

    virtual void draw() {};

    // Outputs true if the sphere given by point and radius intersects the simplified
    // geometry of this shape
    virtual bool isectPossible(const Eigen::Vector3d& point, double radius) const
    {
        return (point - centroid_).norm() < (radius + radius_);
    }

    virtual Eigen::Vector3d support(const Eigen::Vector3d& dir) const
    {
        return (centroid_ + radius_ * dir);
    }

    virtual bool rayIntersection(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                                 Eigen::Vector3d& intersectionPoint);

    const Eigen::Vector3d& getCentroid() const {
        return centroid_;
    }

    double getRadius() const {
        return radius_;
    }

protected:
    Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;

    virtual void updateCentroid() {};

    std::unique_ptr<OGLRenderData> render_data_;
}; // Shape

//Shape operator*(const Eigen::Transform<double, 3, Eigen::Affine> tf, const Shape& s)
//{

//}

} // namespace geometry
