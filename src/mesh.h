#pragma once

#include "glm/glm.hpp"
#include "shaders/shader.h"
#include <string>
#include <vector>
#include <Eigen/Geometry>


typedef struct  {
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 texCoords;
    unsigned int texNum;
} glmVertex;

typedef struct {
    std::vector<float> position;
    std::vector<float> color;
    std::vector<float> texCoords;
    unsigned int texNum;
} Vertex;

typedef struct {
    unsigned int id;
    std::string file_path;
} Texture;

class Mesh
{
public:
    // Define mesh vertices and textures
    std::vector<glmVertex> vertices_;
    std::vector<unsigned int> indices_;
    std::vector<Texture> textures_;

    // Constructor for glm inputs
    Mesh(std::vector<glmVertex> vertices, std::vector<unsigned int> indices,
            std::vector<Texture> textures);
    // Constructor for standard datatypes
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
         std::vector<Texture> textures);

    // Draw mesh
    void draw(Shader shader);

    // Update orientation
    void updateModelTF(glm::mat4 transform);
    void updateModelTF(Eigen::Vector3d position, Eigen::Quaterniond orientation);

private:
    //  Render data
    unsigned int VAO_, VBO_, EBO_;
    // Orientation data
    glm::mat4 modelTF_;

    // Initialize all mesh data
    void setupMesh();
};

