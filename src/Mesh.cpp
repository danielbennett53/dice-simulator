#include "Mesh.h"
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>

Mesh::Mesh(std::vector<glmVertex> vertices, std::vector<unsigned int> indices,
           Texture texture, unsigned int textureUnit)
{
    vertices_ = std::move(vertices);
    indices_ = std::move(indices);
    texture_ = std::move(texture);

    setupMesh();
    loadTextures(textureUnit);
}

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices,
           Texture texture, unsigned int textureUnit)
{
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

        vertices_.push_back(tempVertex);
    }

    indices_ = std::move(indices);
    texture_ = std::move(texture);

    setupMesh();
    loadTextures(textureUnit);
}

void Mesh::setupMesh()
{
    // Generate buffers
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &EBO_);

    // Bind buffers
    glBindVertexArray(VAO_);

    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(glmVertex),
            &vertices_[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int),
                 &indices_[0], GL_STATIC_DRAW);

    // Define position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glmVertex), nullptr);
    // Define color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glmVertex),
                          (void*)offsetof(glmVertex, color));
    // Define texture coordinates attribute
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glmVertex),
                          (void*)offsetof(glmVertex, texCoords));
    // Define texture number attribute
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_TRUE, sizeof(glmVertex),
                          (void*)offsetof(glmVertex, texNum));

    // Unbind vertex array
    glBindVertexArray(0);

    // Set transform to identity
    modelTF_ = glm::mat4(1.0f);
}


void Mesh::loadTextures(unsigned int textureUnit)
{
    // Exit if no textures
    if (texture_.file_paths.empty())
        return;

    texUnit_ = textureUnit;

    // Activate texture
    glActiveTexture(GL_TEXTURE0 + texUnit_);

    // Load texture array
    glGenTextures(1, &texture_.id);
    glBindTexture(GL_TEXTURE_2D_ARRAY, texture_.id);

    // Load the first image to get size
    int w, h, nrChannels;
    stbi_load(texture_.file_paths[0].c_str(), &w, &h, &nrChannels, 0);

    // Initialize the texture array
    auto depth = (GLsizei) texture_.file_paths.size();
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, w, h, depth, 0, GL_RGBA,
                 GL_UNSIGNED_INT, nullptr);

    // Set the texture wrapping/filtering options
//    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BASE_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Iterate through filepaths
    for (int i=0; i<depth; ++i) {
        unsigned char* data = stbi_load(texture_.file_paths[i].c_str(), &w, &h, &nrChannels, 0);
        glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, w, h, 1, GL_RGBA,
                        GL_UNSIGNED_BYTE, data);
        stbi_image_free(data);
    }

    // Unbind texture and free data
    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
}


void Mesh::draw(Shader shader)
{
    // Activate texture and bind it
    shader.setInt("tex", texUnit_);
    glActiveTexture(GL_TEXTURE0 + texUnit_);
    glBindTexture(GL_TEXTURE_2D_ARRAY, texture_.id);

    // Set model transform
    shader.setMat4f("model", modelTF_);
    // draw mesh
    glBindVertexArray(VAO_);
    glDrawElements(GL_TRIANGLES, (GLsizei) indices_.size(), GL_UNSIGNED_INT,
                   nullptr);
    glBindVertexArray(0);
}

void Mesh::updateModelTF(glm::mat4 transform)
{
    modelTF_ = transform;
}

void Mesh::updateModelTF(Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
    auto tf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    tf.rotate(orientation);
    tf.translate(position);

    modelTF_ = glm::make_mat4(tf.data());
}

