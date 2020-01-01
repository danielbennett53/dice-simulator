#pragma once

#include <string>
#include <vector>
#include <map>
#include <QOpenGLVertexArrayObject>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>

//namespace geometry {

typedef struct  {
    Eigen::Vector3d position;
    Eigen::Vector2d texCoords;
} drawVertex;

typedef struct {
    std::vector<int> vertexIdxs;
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
    double radius;
} Face;

class Mesh
{
public:    
    // Geometry functions
    bool rayIntersectsMesh(const Eigen::Vector3d &rayOrigin,
                           const Eigen::Vector3d &rayVector,
                           Eigen::Vector3d &intersectionPoint);

    // Define mesh vertices and textures
    std::vector<drawVertex> drawVertices_;
    std::vector<int> drawIndices_;
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<Face> faces_;
    std::vector<std::vector<int>> facesLookup_;

    // Define mesh centroid and largest convex dimension for intersection culling
    Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;

    //  Render data
    QOpenGLVertexArrayObject VAO_;
    QOpenGLBuffer VBO_, EBO_;
    std::shared_ptr<QOpenGLTexture> tex_;

    std::string textureFilepath_;

    // Constructor
    Mesh(const std::string& objFile);

    // Copy constructor
    Mesh(const Mesh &obj) : VAO_(), VBO_(QOpenGLBuffer::VertexBuffer),
        EBO_(QOpenGLBuffer::IndexBuffer) {(void) obj;}

    typedef enum {
        FLOOR,
        D4,
        D6,
        D20
    } meshType;

    // Global list of meshes
    static std::map<Mesh::meshType, Mesh> options;
    static std::map<Mesh::meshType, Mesh> initMeshes();

protected:
    double rayIntersectsTriangle(const Eigen::Vector3d &rayOrigin,
                              const Eigen::Vector3d &rayVector,
                              const std::vector<Eigen::Vector3d> &verts);
};

//}
