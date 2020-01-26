#include "ConvexPolytope.h"
#include <cassert>
#include <algorithm>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

namespace geometry {


ConvexPolytope::ConvexPolytope(const ObjReader& obj)
{
    // Make pointers for each vertex
    std::vector<std::shared_ptr<Vertex>> vtxs;
    for (const auto& p : obj.points) {
        vtxs.push_back(std::make_shared<Vertex>(p));
        vertices_.push_back(vtxs.back());
    }
    for (const auto& f : obj.faces) {
        // Calculate obj file face normal
        Eigen::Vector3d norm = (obj.normals[f[0].normal_idx] +
                                obj.normals[f[1].normal_idx] +
                                obj.normals[f[2].normal_idx]) / 3;
        Face f_out{
            {vtxs[f[0].point_idx], vtxs[f[1].point_idx]},
            {vtxs[f[1].point_idx], vtxs[f[2].point_idx]},
            {vtxs[f[2].point_idx], vtxs[f[0].point_idx]} };

        // Make sure that the saved face normal is aligned with the obj file normal
        if (f_out.normal_.dot(norm) < 0)
            f_out.reverse();

        faces_.push_back(f_out);
    }

    updateCentroid();
}

ConvexPolytope::ConvexPolytope(const std::vector<Eigen::Vector3d>& points)
{
    // Only 3-simplexes can initialize a polytope
    assert(("Can only initialize ConvexPolytope with 2 or 3-simplexes",
            (points.size() == 4)));

    // Create vertices
    std::vector<std::shared_ptr<Vertex>> vertices;
    for (const auto& p : points) {
        vertices.push_back(std::make_shared<Vertex>(p));
        vertices_.push_back(vertices.back());
    }

    // Create faces
    for (unsigned int i = 0; i < points.size(); ++i) {
        const auto& rotated = std::rotate(vertices.begin(), vertices.begin() + i, vertices.end());

        // Determine direction of face normal
        Face f({rotated[0], rotated[1]},
               {rotated[1], rotated[2]},
               {rotated[2], rotated[0]});
        if ( (*rotated[3] - *rotated[0]).dot(f.normal_) > 0)
            faces_.emplace_back(f);
        else
            faces_.emplace_back(-f);
    }

    updateCentroid();
}


void ConvexPolytope::addVertex(const Eigen::Vector3d& point)
{
    // Find all faces that can "see" the point and remove them.
    std::list<Edge> edges;
    for (unsigned int i = 0; i < faces_.size(); ++i) {
        Eigen::Vector3d v = point - faces_[i].edges_[0].startPoint_->getPos();
        if (faces_[i].normal_.dot(v) > 0) {
            for (const auto& e : faces_[i].edges_) {
                auto el = std::find(edges.begin(), edges.end(), -e);
                if (el == edges.end())
                    edges.push_back(e);
                else
                    edges.erase(el);
            }
            // Remove face from vector
            remove_at(faces_, i--);
        }
    }

    // Add new faces for each border edge
    const auto new_vtx = std::make_shared<Vertex>(point);
    vertices_.push_back(new_vtx);
    for (const auto& e : edges) {
        faces_.emplace_back(e,
                            Edge(e.endPoint_, new_vtx),
                            Edge(new_vtx, e.startPoint_));
    }

    centroid_valid_ = false;
    render_data_.release();
}


void ConvexPolytope::addVertexSimplex(const Eigen::Vector3d& point, unsigned int face_to_keep)
{
    assert(("addVertexSimiplex is only valid on simplexes (4 points)", faces_.size() == 4));

    // Remove all other faces from shape
    for (unsigned int i = 0; i < faces_.size(); ++i) {
        if (i != face_to_keep)
            remove_at(faces_, i--);
    }

    // Make sure that the remaining face is in the right direction
    Eigen::Vector3d v = point - faces_[0].edges_[0].startPoint_->getPos();
    if (faces_[0].normal_.dot(v) > 0)
        faces_[0].reverse();

    // Add three remaining faces. Reverse the edges on the first face so the normal is facing out
    const auto new_vtx = std::make_shared<Vertex>(point);
    vertices_.push_back(new_vtx);
    for (const auto& e : faces_[0].edges_) {
        faces_.emplace_back(-e,
                            Edge(e.startPoint_, new_vtx),
                            Edge(new_vtx, e.endPoint_));
    }

    centroid_valid_ = false;
    render_data_.release();
}


void ConvexPolytope::updateCentroid()
{
    // Find all unique points on all faces
    std::vector<std::shared_ptr<Vertex>> vertices;
    for (const auto& f : faces_) {
        for (unsigned int i = 0; i < 3; ++i) {
            const auto loc = std::find(vertices.begin(), vertices.end(), f.edges_[i].startPoint_);
            if (loc == vertices.end())
                vertices.push_back(f.edges_[i].startPoint_);
        }
    }

    // Calculate the centroid and radius
    centroid_ = Eigen::Vector3d::Zero();
    radius_ = 0.0;
    for (const auto& v : vertices)
        centroid_ += v->getPos();
    centroid_ /= vertices.size();

    for (const auto& v : vertices) {
        double r = (v->getPos() - centroid_).norm();
        if (r > radius_)
            radius_ = r;
    }

    centroid_valid_ = true;
}


void ConvexPolytope::draw()
{
    if (render_data_) {
        render_data_->draw();
        return;
    }

    // Get vector of vertices
    std::vector<Eigen::Vector3d> vertices;
    for (const auto& f : faces_) {
        vertices.push_back(f.edges_[0].startPoint_->getPos());
        vertices.push_back(f.edges_[1].startPoint_->getPos());
        vertices.push_back(f.edges_[2].startPoint_->getPos());
    }

    // Create temporary OGL buffer to draw vertices
    QOpenGLBuffer buf;
    buf.create();
    buf.bind();
    buf.setUsagePattern(QOpenGLBuffer::StreamDraw);
    buf.allocate(&vertices[0], vertices.size() * sizeof(vertices[0]));
    QOpenGLFunctions* gl_fncs = QOpenGLContext::currentContext()->functions();
    gl_fncs->glEnableVertexAttribArray(0);
    gl_fncs->glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(vertices[0]), nullptr);
    gl_fncs->glDrawArrays(GL_TRIANGLES, 0, (GLsizei) vertices.size());
    buf.release();
    buf.destroy();
} // draw()


bool ConvexPolytope::support(Eigen::Vector3d &vector, Eigen::Vector3d& out_point) const
{
    unsigned int i = 0;
    auto curr_vtx = static_cast<std::shared_ptr<Vertex>>(vertices_[0]);
    double max_dot_product = vector.dot(curr_vtx->getPos());
    while (++i < vertices_.size()) {
        auto connections = curr_vtx->getConnections();
        bool changed = false;
        for (const auto& c : connections) {
            auto new_vtx = static_cast<std::shared_ptr<Vertex>>(c);
            double new_dot_product = new_vtx->getPos().dot(curr_vtx->getPos());
            if (new_dot_product > max_dot_product) {
                max_dot_product = new_dot_product;
                curr_vtx = new_vtx;
                changed = true;
            }
        }
        // Return if no adjacent vertices have higher dot product
        if (!changed) {
            out_point = curr_vtx->getPos();
            return true;
        }
    }
    return false;
}


} // namespace geometry
