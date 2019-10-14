#pragma once

#include "glm/glm.hpp"
#include "shaders/Shader.h"
#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>


typedef struct  {
    glm::vec3 position;
    glm::vec2 texCoords;
} Vertex;

typedef struct {
    std::vector<unsigned int> meshIndices;
    glm::vec3 normal;
} Face;

class Mesh
{
public:
    // Define mesh vertices and textures
    std::vector<Vertex> vertices_;
    std::vector<unsigned int> indices_;
    std::vector<Face> faces_;

    // Constructor for glm inputs
    explicit Mesh(const std::string& objFile);

    // Draw mesh
    void draw(Shader shader);

    // Update orientation
    void updateModelTF(const Eigen::Vector3d& position,
                       const Eigen::Quaterniond& orientation);

private:
    //  Render data
    unsigned int VAO_, VBO_, EBO_, texId_;
    // Orientation data
    glm::mat4 modelTF_;

    // Initialize all mesh data
    void setupMesh();
    // Load textures
    void loadTextures(const std::string& filepath, unsigned int textureUnit);

    void calculatePhysicalProperties();
};

