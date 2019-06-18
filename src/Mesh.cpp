#include "Mesh.h"
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include "glad/glad.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
#include <fstream>
#include <map>


Mesh::Mesh(const std::string& objFile)
{
    std::ifstream fd;
    fd.open(objFile, std::ios::in);
    if (!fd) {
        std::cerr << "Unable to open file " << objFile << std::endl;
        return;
    }

    // Variables for storing each element when scanning obj file
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> texCoords;
    std::vector<glm::vec3> normals;
    std::map<unsigned int, std::map<unsigned int, unsigned int>> uniqueIndices;
    std::string textureFilepath;

    std::string line;
    while (std::getline(fd, line)) {
        std::stringstream ls(line);
        std::string token;
        ls >> token;

        if (token == "v") { // Vertex
            glm::vec3 vertex;
            ls >>  vertex[0] >> vertex[1] >> vertex[2];
            vertices.push_back(vertex);
        } else if (token == "vt") { // Texture coordinates
            glm::vec2 texCoord;
            ls >> texCoord[0] >> texCoord[1];
            texCoords.push_back(texCoord);
        } else if (token == "vn") { // Normal vectors
            glm::vec3 normal;
            ls >> normal[0] >> normal[1] >> normal[2];
            normals.push_back(normal);
        } else if (token == "f") { // Triangle indices

            std::vector<unsigned int> faceIndices;
            glm::vec3 norm;
            while (ls) {
                std::vector<unsigned int> idxs;

                // Extract first set of indices
                std::string set;
                ls >> set;
                if (set.empty())
                    break;
                std::stringstream ss(set);
                std::string idx;
                while(std::getline(ss, idx, '/')) {
                    idxs.push_back(std::stoi(idx) - 1);
                }
                if (uniqueIndices.count(idxs[0]) && uniqueIndices[idxs[0]].count(idxs[1])) {
                    indices_.push_back(uniqueIndices[idxs[0]][idxs[1]]);
                } else {
                    vertices_.emplace_back(
                            Vertex{vertices[idxs[0]], texCoords[idxs[1]]});
                    indices_.push_back(vertices_.size() - 1);
                    uniqueIndices[idxs[0]][idxs[1]] = vertices_.size() - 1;
                }

                faceIndices.push_back(indices_.back());
                norm = normals[idxs[2]];
            }
            faces_.emplace_back(Face{faceIndices, norm});
        } else if (token == "mtllib") { // Material file
            // Get name of mtl file
            std::string filename;
            std::string ignore;
            ls >> filename;

            // Split input filename to get filepath
            size_t found;
            found = objFile.find_last_of("/\\");
            std::string filepath = objFile.substr(0, found+1);

            // Open material file
            std::ifstream mtlfd;
            mtlfd.open(filepath + filename, std::ios::in);
            if (!mtlfd) {
                std::cerr << "Unable to open file " << filepath + filename << std::endl;
                textureFilepath = "";
                continue;
            }

            // Scan mtl file for texture image filename
            while (mtlfd) {
                std::string mtlToken;
                mtlfd >> mtlToken;

                if (mtlToken == "map_Kd") {
                    std::string texFile;
                    mtlfd >> texFile;
                    textureFilepath = filepath + texFile;
                    break;
                }
            }
            mtlfd.close();
        }
    }
    fd.close();

    setupMesh();
    loadTextures(textureFilepath, 0);
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
    glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex),
            &vertices_[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int),
                 &indices_[0], GL_STATIC_DRAW);

    // Define position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    // Define texture coordinates attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(1);

    // Unbind vertex array
    glBindVertexArray(0);

    // Set transform to identity
    modelTF_ = glm::mat4(1.0f);
}


void Mesh::loadTextures(const std::string& filepath, unsigned int textureUnit)
{
    // Exit if no textures
    if (filepath.empty()) {
        std::cout << "No texture file found" << std::endl;
        return;
    }

    // Activate texture
    glActiveTexture(GL_TEXTURE0);

    // Load texture array
    glGenTextures(1, &texId_);
    glBindTexture(GL_TEXTURE_2D, texId_);

    // Load the first image to get size
    int w, h, nrChannels;

    // Set the texture wrapping/filtering options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Load texture data
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(filepath.c_str(), &w, &h, &nrChannels, 0);
    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        std::cout << "Failed to load texture" << std::endl;
    }

    // Unbind texture and free data
    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);
}


void Mesh::draw(Shader shader)
{
    // Activate texture and bind it
    shader.setInt("tex", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texId_);

    // Set model transform
    shader.setMat4f("model", modelTF_);
    // draw mesh
    glBindVertexArray(VAO_);
    glDrawElements(GL_TRIANGLES, (GLsizei) indices_.size(), GL_UNSIGNED_INT,
                   nullptr);
    glBindVertexArray(0);
}


void Mesh::updateModelTF(const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& orientation)
{
    auto tf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    tf.translate(position);
    tf.rotate(orientation);

    modelTF_ = glm::make_mat4(tf.data());
}

