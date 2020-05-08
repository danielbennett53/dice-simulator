#include "Shape.h"

#include <QOpenGLFunctions>

namespace geometry {

OGLRenderData::OGLRenderData(const ObjReader& obj)
{
    // Iterate through faces and find unique vertices
    std::vector<ObjReader::objVertex> obj_vtxs;
    for (const auto& f : obj.faces) {
        for (const auto& v : f) {
            auto loc = std::find(obj_vtxs.begin(), obj_vtxs.end(), v);
            if (loc == obj_vtxs.end()) {
                drawIndices_.push_back(obj_vtxs.size());
                obj_vtxs.push_back(v);
            } else {
                drawIndices_.push_back(std::distance(obj_vtxs.begin(), loc));
            }
        }
    }

    // Convert obj_vtxs into drawVertices
    for (const auto& v : obj_vtxs)
        drawVertices_.push_back(drawVertex({obj.points[v.point_idx],
                                            obj.tex_coords[v.tex_idx]}));

    setRenderData(obj.tex_file);
}


void OGLRenderData::setRenderData(const std::string tex_file)
{
    // Create OGL objects
    auto a = QOpenGLContext::currentContext();
    QOpenGLFunctions* gl_fncs = a->functions();
    VAO_.create();
    VBO_.create();
    EBO_.create();

    VAO_.bind();
    VBO_.bind();
    EBO_.bind();

    // Allocate arrays
    VBO_.setUsagePattern(QOpenGLBuffer::StaticDraw);
    VBO_.allocate(&drawVertices_[0], drawVertices_.size() * sizeof(drawVertex));

    EBO_.setUsagePattern(QOpenGLBuffer::StaticDraw);
    EBO_.allocate(&drawIndices_[0], drawIndices_.size() * sizeof(int));

    // Enable attribute arrays (0 is vertex position, 1 is texture coord)
    gl_fncs->glEnableVertexAttribArray(0);
    gl_fncs->glEnableVertexAttribArray(1);

    // Define position attribute
    gl_fncs->glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(drawVertex), (void*) offsetof(drawVertex, position));

    // Define texture coordinates attribute
    gl_fncs->glVertexAttribPointer(1, 2, GL_DOUBLE, GL_FALSE, sizeof(drawVertex),
                                  (void*) offsetof(drawVertex, texCoords));

    // Load textures

    // Exit if no textures
    if (tex_file.empty()) {
        std::cout << "Texture file " << tex_file << " not found" << std::endl;
    } else {
        tex_ = std::make_shared<QOpenGLTexture>(QImage(QString::fromStdString(tex_file)).mirrored());
        tex_->create();
        tex_->setWrapMode(QOpenGLTexture::Repeat);
        tex_->setMinificationFilter(QOpenGLTexture::Nearest);
        tex_->setMagnificationFilter(QOpenGLTexture::Linear);
    }
    // Unbind vertex array
    VAO_.release();
    VBO_.release();
    EBO_.release();
}


void OGLRenderData::draw(QOpenGLShaderProgram& shader)
{
    QOpenGLFunctions* gl_fncs = QOpenGLContext::currentContext()->functions();
    VAO_.bind();
    VBO_.bind();
    EBO_.bind();
    tex_->bind();
    gl_fncs->glDrawElements(GL_TRIANGLES, (GLsizei) drawIndices_.size(), GL_UNSIGNED_INT,
                   nullptr);
    VAO_.release();
    VBO_.release();
    EBO_.release();
    tex_->release();
}

bool Shape::rayIntersection(const Eigen::Vector3d& origin,
                            const Eigen::Vector3d& dir,
                            Eigen::Vector3d& intersectionPoint)
{
    // Find closest approach point
    double t = (centroid_ - origin).dot(dir) / (dir.dot(dir));
    if (t < 0)
        return false;
    Eigen::Vector3d p = origin + t * dir;
    if (p.norm() > radius_)
        return false;

    // Find point of first intersection with sphere
    double a = dir.squaredNorm();
    double b = 2 * (origin - centroid_).dot(dir);
    double c = (origin - centroid_).squaredNorm() - radius_*radius_;

    double t1 = (-b + sqrt(b*b - 4*a*c) ) / (2 * a);
    double t2 = (-b - sqrt(b*b - 4*a*c) ) / (2 * a);

    t = (t1 < t2) && (t1 > 0) ? t1 : t2;

    intersectionPoint = origin + t * dir;
    return true;
}

} // namespace geometry
