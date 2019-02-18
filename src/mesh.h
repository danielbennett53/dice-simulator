#pragma once

#include "glm/glm.hpp"
#include <string>
#include <vector>
#include <Eigen/Dense>

typedef struct glmVertex {
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 texCoords;
    unsigned int textureNum;
};

typedef struct Vertex {
    std::vector<float> position;
    std::vector<float> color;
    std::vector<float> texCoords;
    unsigned int textureNum;
};

typedef struct Texture {
    unsigned int id;
    std::string file_path;
};

class Mesh
{
public:
    // Define mesh vertices and textures
    std::vector<glmVertex> vertices_;
    std::vector<unsigned int> indices_;
    std::vector<Texture> textures_;

    // Constructor for glm inputs
    Mesh(std::vector<glmVertex> vertices, std::vector<unsigned int> indices,
            std::vectore<Texture> textures);
    // Constructor for standard datatypes
    Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
         std::vectore<Texture> textures);

    draw();

    





};

