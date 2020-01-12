#include "ConvexPolytope.h"
#include <cassert>
#include <algorithm>

namespace geometry {


ConvexPolytope::ConvexPolytope(const std::string& objFile)
{

}

ConvexPolytope::ConvexPolytope(const std::vector<Eigen::Vector3d>& points)
{
    // Only 3-simplexes can initialize a polytope
    assert(("Can only initialize ConvexPolytope with 2 or 3-simplexes",
            (points.size() == 4)));

    // Create vertices
    std::vector<std::shared_ptr<Vertex>> vertices;
    for (const auto& p : points) {
        vertices.emplace_back(p);
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
    for (const auto& e : edges) {
        faces_.emplace_back(e,
                            Edge(e.endPoint_, new_vtx),
                            Edge(new_vtx, e.startPoint_));
    }

    centroid_valid_ = false;
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
    for (const auto& e : faces_[0].edges_) {
        faces_.emplace_back(-e,
                            Edge(e.startPoint_, new_vtx),
                            Edge(new_vtx, e.endPoint_));
    }

    centroid_valid_ = false;
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

} // namespace geometry
