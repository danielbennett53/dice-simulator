#include "ConvexPolytope.h"
#include "Geometry.h"
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
    render_data_ = std::make_unique<OGLRenderData>(obj);
    updateCentroid();
}

ConvexPolytope::ConvexPolytope(const std::vector<Eigen::Vector3d>& points)
{
    // Only 3-simplexes can initialize a polytope
    assert(("Can only initialize ConvexPolytope with 3-simplexes",
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

    updateCentroid();
    render_data_.release();
}


void ConvexPolytope::updateCentroid()
{
    // Calculate the centroid and radius
    centroid_ = Eigen::Vector3d::Zero();
    radius_ = 0.0;
    for (const auto& v : getVertices())
        centroid_ += v->getPos();
    centroid_ /= getVertices().size();

    for (const auto& v : getVertices()) {
        double r = (v->getPos() - centroid_).norm();
        if (r > radius_)
            radius_ = r;
    }
}


void ConvexPolytope::draw(QOpenGLShaderProgram& shader)
{
    static bool on = false;
    on = !on;
    if (render_data_ && on) {
        render_data_->draw(shader);
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


Eigen::Vector3d ConvexPolytope::support(const Eigen::Vector3d &vector) const
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
            return curr_vtx->getPos();
        }
    }
    return curr_vtx->getPos();
}


void Simplex::addVertex(const Eigen::Vector3d& point, unsigned int face_to_keep)
{
    // Behavior changes based on number of vertices
    switch (getVertices().size()) {
    case 0:
    case 1:
        vertices_.push_back(std::make_shared<Vertex>(point));
        break;
    case 2:
    {
        vertices_.push_back(std::make_shared<Vertex>(point));
        auto vtxs = getVertices();
        faces_.emplace_back(Edge(vtxs[0], vtxs[1]),
                            Edge(vtxs[1], vtxs[2]),
                            Edge(vtxs[2], vtxs[0]));
        break;
    }

    // Case 4 is the same as case 3 once extra faces are removed
    case 4:
    {
        // Remove all other faces from shape
        for (unsigned int i = 0; i < faces_.size(); ++i) {
            if (i != face_to_keep)
                remove_at(faces_, i--);
        }
    }
    case 3:
    {
        vertices_.push_back(std::make_shared<Vertex>(point));
        auto vtxs = getVertices();
        if (faces_[0].normal_.dot(point - vtxs[0]->getPos()) > 0)
            faces_[0].reverse();
        for (const auto& e : faces_[0].edges_) {
            faces_.emplace_back(-e,
                                Edge(e.startPoint_, vtxs.back()),
                                Edge(vtxs.back(), e.endPoint_));
        }
        break;
    }

    default:
        break;
    }

    return;
}


bool Simplex::nearestSimplex(Eigen::Vector3d& new_dir, unsigned int& nearest_face)
{
    // Different checks depending on size of simplex
    auto vtxs = getVertices();
    nearest_face = 0;
    switch (vtxs.size()) {
    case 1:
        new_dir = -vtxs[0]->getPos();
        break;

    case 2:
    {
        Eigen::Vector3d edge = vtxs[1]->getPos() - vtxs[0]->getPos();
        new_dir = edge.cross(-vtxs[0]->getPos().cross(edge));
        break;
    }
    case 3:
    {
        Eigen::Vector3d norm = faces_[0].edges_[1].diff_.cross(faces_[0].normal_);
        Eigen::Vector3d diff = -vtxs[2]->getPos();
        if (norm.dot(diff) > 0)
            new_dir = norm;
        else {
            norm = faces_[0].edges_[2].diff_.cross(faces_[0].normal_);
            if (norm.dot(diff) > 0)
                new_dir = norm;
            else if (faces_[0].normal_.dot(diff) > 0)
                new_dir = faces_[0].normal_;
            else
                new_dir = -faces_[0].normal_;
        }
        break;
    }
    case 4:
    {
        Eigen::Vector3d diff = -vtxs.back()->getPos();
        // Iterate through faces, then average the normals of every face the point is outside of
        std::vector<unsigned int> face_idxs;
        for (unsigned int i = 1; i < faces_.size(); ++i) {
            if (faces_[i].normal_.dot(diff) > 0)
                face_idxs.push_back(i);
        }
        if (face_idxs.size() >= 3)
            std::cout << "nearestSimplex entered impossible state" << std::endl;
        else if (face_idxs.size() == 0)
            return true;
        else {
            new_dir.setZero();
            for (const auto& i : face_idxs)
                new_dir += faces_[i].normal_;
            new_dir /= face_idxs.size();
        }
        nearest_face = face_idxs[0];
        break;
    }
    default:
        std::cout << "Simplex is invalid size" << std::endl;
        break;
    }
    return false;
}


bool ConvexPolytope::rayIntersection(const Eigen::Vector3d& origin,
                                     const Eigen::Vector3d& dir,
                                     Eigen::Vector3d& intersectionPoint)
{

    // See if ray gets close enough to possibly intersect
    // Point of closest approach
    double t = (centroid_.dot(dir) - origin.dot(dir)) / dir.dot(dir);
    Eigen::Vector3d p = origin + t*dir;
    if ( (p - centroid_).norm() > radius_)
        return false;

    // Iterate through every face to find closest intersection
    t = -1;
    for (const auto &f : faces_) {
        auto new_t = rayIntersectsTriangle(origin, dir, {f.edges_[0].startPoint_->getPos(),
                              f.edges_[1].startPoint_->getPos(), f.edges_[2].startPoint_->getPos()});
        if ((new_t > 0) && ((t < 0) || (new_t < t))) {
            t = new_t;
        }
    }

    if (t < 0)
        return false;

    intersectionPoint = origin + t * dir;
    return true;
}

void ConvexPolytope::transform(const Eigen::Transform<double, 3, Eigen::Affine> &tf)
{
    auto change_idx = faces_[0].edges_[0].startPoint_->change_idx_ + 1;
    for (auto& f : faces_) {
        f.transform(tf, centroid_, change_idx);
    }
    centroid_ = tf * centroid_;
    render_data_->transform(tf);
}

//void ConvexPolytope::setTransform(const Eigen::Transform<double, 3, Eigen::Affine> &tf)
//{
//    auto new_tf = tf * render_data_->tf_.inverse();
//    transform(new_tf);
//}

} // namespace geometry
