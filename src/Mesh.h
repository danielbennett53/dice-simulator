#pragma once

#include "Geometry.h"
#include <string>
#include <vector>
#include <map>
#include <QOpenGLVertexArrayObject>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>



typedef struct {
    std::vector<int> vertexIdxs;
    Eigen::Vector3d normal;
} Face;


class Mesh
{
public:    
    // Geometry functions
    bool rayIntersectsMesh(const Eigen::Vector3d &rayOrigin,
                           const Eigen::Vector3d &rayVector,
                           Eigen::Vector3d &intersectionPoint);

    // Define mesh vertices and textures    
    std::vector<geometry::vertex> vertices_;
    std::vector<Face> faces_;

    // Define mesh centroid and largest convex dimension for intersection culling
    Eigen::Vector3d centroid_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;

    //  Render data
    

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
