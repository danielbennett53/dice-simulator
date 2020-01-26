#include "Plane.h"

namespace geometry {

Plane::Plane(const ObjReader& obj)
{
    // Check that all normals are the same
    auto last_norm = obj.normals[0];
    for (unsigned int i = 1; i < obj.normals.size(); ++i) {
        if ((last_norm - obj.normals[i]).norm() > 1e-6)
            return;
    }

    normal_ = last_norm;
    vertices_ = obj.points;

    render_data_ = std::make_unique<OGLRenderData>(obj);
}

Plane::Plane(const Eigen::Vector3d normal, const Eigen::Vector3d center,
             const Eigen::Vector3d x_axis, double width, double height,
             const std::string tex_file) : normal_(normal)
{
    centroid_ = center;
    Eigen::Vector3d y_axis = normal.cross(x_axis);
    vertices_.push_back(center - width * x_axis - height * y_axis);
    vertices_.push_back(center + width * x_axis - height * y_axis);
    vertices_.push_back(center - width * x_axis + height * y_axis);
    vertices_.push_back(center + width * x_axis + height * y_axis);

    std::vector<OGLRenderData::drawVertex> draw_vertices;
    draw_vertices.emplace_back(vertices_[0], Eigen::Vector2d(0, 0));
    draw_vertices.emplace_back(vertices_[1], Eigen::Vector2d(1, 0));
    draw_vertices.emplace_back(vertices_[2], Eigen::Vector2d(0, 1));
    draw_vertices.emplace_back(vertices_[3], Eigen::Vector2d(1, 1));

    std::vector<int> draw_indices = {
        0, 1, 3,
        0, 3, 2
    };

    render_data_ = std::make_unique<OGLRenderData>(draw_vertices,
                                                   draw_indices,
                                                   tex_file);
}


bool Plane::support(Eigen::Vector3d& vector, Eigen::Vector3d& out_point) const
{
    // Add depth to each vertex in the direction of the normal
    double max_dot_product = -HUGE_VAL;
    Eigen::Vector3d max_vertex;
    for (const auto& v : vertices_) {
        Eigen::Vector3d offset_vtx = v - normal_ * 100;
        double new_dot_product = vector.dot(offset_vtx);
        if (new_dot_product > max_dot_product) {
            max_vertex = offset_vtx;
            max_dot_product = new_dot_product;
        }
    }
    out_point = max_vertex;
    return true;
}

} // namespace geometry
