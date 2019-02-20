#include "mesh.h"
#include <glm/gtc/type_ptr.hpp>

Mesh::Mesh(std::vector<glmVertex> vertices, std::vector<unsigned int> indices,
           std::vector<Texture> textures)
{
    this->vertices_ = std::move(vertices);
    this->indices_ = std::move(indices);
    this->textures_ = std::move(textures);

    this->setupMesh();
}

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
           std::vector<Texture> textures)
{
    // Initialize GLM vertex array
    std::vector<glmVertex> vertices_out;

    // Iterate through input vector to convert Vertex arrray to glmVertex array
    for (Vertex vertex : vertices)
    {
        auto position = glm::vec3(vertex.position[0], vertex.position[1],
                                  vertex.position[2]);
        auto color = glm::vec3(vertex.color[0], vertex.color[1],
                               vertex.color[2]);
        auto texCoords = glm::vec3(vertex.texCoords[0], vertex.texCoords[1],
                                   vertex.texCoords[2]);

        glmVertex tempVertex = {
                .position = position,
                .color = color,
                .texCoords = texCoords,
                .texNum = vertex.texNum
        };

        vertices_out.push_back(tempVertex);
    }

    this->vertices_ = vertices_out;
    this->indices_ = std::move(indices);
    this->textures_ = std::move(textures);

    this->setupMesh();
}

void Mesh::setupMesh()
{

}

void Mesh::draw(Shader shader)
{

}

void Mesh::updateModelTF(glm::mat4 transform)
{

}

void Mesh::updateModelTF(Eigen::Vector3d position, Eigen::Quaterniond orientation)
{

}

