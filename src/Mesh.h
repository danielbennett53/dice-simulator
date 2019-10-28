#pragma once

#include <string>
#include <vector>
#include <map>
#include <QOpenGLVertexArrayObject>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>

typedef struct  {
    Eigen::Vector3d position;
    Eigen::Vector2d texCoords;
} Vertex;

typedef struct {
    std::vector<int> meshIndices;
    Eigen::Vector3d normal;
} Face;

class Mesh
{
public:    
    // Define mesh vertices and textures
    std::vector<Vertex> vertices_;
    std::vector<int> indices_;
    std::vector<Face> faces_;

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
};

