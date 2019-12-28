#include "Mesh.h"
#include "Geometry.h"
#include <iostream>
#include <fstream>
#include <map>

Mesh::Mesh(const std::string& objFile) : VAO_(), VBO_(QOpenGLBuffer::VertexBuffer),
EBO_(QOpenGLBuffer::IndexBuffer)
{
    std::ifstream fd;
    fd.open(objFile, std::ios::in);
    if (!fd) {
        std::cerr << "Unable to open file " << objFile << std::endl;
        return;
    }

    // Variables for storing each element when scanning obj file
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector2d> texCoords;
    std::vector<Eigen::Vector3d> normals;
    std::map<int, std::map<int, int>> uniqueIndices;

    std::string line;
    while (std::getline(fd, line)) {
        std::stringstream ls(line);
        std::string token;
        ls >> token;

        if (token == "v") { // Vertex
            Eigen::Vector3d vertex;
            ls >>  vertex[0] >> vertex[1] >> vertex[2];
            vertices.push_back(vertex);
        } else if (token == "vt") { // Texture coordinates
            Eigen::Vector2d texCoord;
            ls >> texCoord[0] >> texCoord[1];
            texCoords.push_back(texCoord);
        } else if (token == "vn") { // Normal vectors
            Eigen::Vector3d normal;
            ls >> normal[0] >> normal[1] >> normal[2];
            normals.push_back(normal);
        } else if (token == "f") { // Triangle indices

            std::vector<int> faceIndices;
            Eigen::Vector3d norm;
            while (ls) {
                std::vector<int> idxs;

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
                textureFilepath_ = "";
                continue;
            }

            // Scan mtl file for texture image filename
            while (mtlfd) {
                std::string mtlToken;
                mtlfd >> mtlToken;

                if (mtlToken == "map_Kd") {
                    std::string texFile;
                    mtlfd >> texFile;
                    textureFilepath_ = filepath + texFile;
                    break;
                }
            }
            mtlfd.close();
        }
    }
    fd.close();

    for (const auto &v : vertices_) {
        centroid_ += v.position;
        for (int i = 0; i < 3; ++i) {
            lowerLims_[i] = v.position[i] < lowerLims_[i] ? v.position[i] : lowerLims_[i];
            upperLims_[i] = v.position[i] > upperLims_[i] ? v.position[i] : upperLims_[i];
        }
    }
    centroid_ /= vertices_.size();
}

std::map<Mesh::meshType, Mesh> Mesh::initMeshes()
{
    std::map<Mesh::meshType, Mesh> m;
    m.emplace(Mesh::FLOOR, "../../resources/floor.obj");
    m.emplace(Mesh::D4, "../../resources/d4.obj");
    m.emplace(Mesh::D6, "../../resources/d6.obj");
    m.emplace(Mesh::D20, "../../resources/d20.obj");

    return m;
}

std::map<Mesh::meshType, Mesh> Mesh::options = Mesh::initMeshes();

bool Mesh::rayIntersectsMesh(const Eigen::Vector3d &rayOrigin,
                             const Eigen::Vector3d &rayVector,
                             Eigen::Vector3d &intersectionPoint)
{
    // See if ray gets close enough to possibly intersect
    // Point of closest approach
    double t = (centroid_.dot(rayVector) - rayOrigin.dot(rayVector)) / rayVector.dot(rayVector);
    Eigen::Vector3d p = rayOrigin + t*rayVector;
    if ( (p[0] > upperLims_[0]) || (p[1] > upperLims_[1]) || (p[2] > upperLims_[2]) ||
         (p[0] < lowerLims_[0]) || (p[1] < lowerLims_[1]) || (p[2] < lowerLims_[2])) {
        return false;
    }

    // Iterate through every face to find closest intersection
    t = -1;
    for (const auto &f : faces_) {
        auto new_t = geometry::rayIntersectsTriangle(rayOrigin, rayVector, {vertices_[f.meshIndices[0]].position,
                              vertices_[f.meshIndices[1]].position, vertices_[f.meshIndices[2]].position});
        if ((new_t > 0) && ((t < 0) || (new_t < t))) {
            t = new_t;
        }
    }

    if (t < 0)
        return false;

    intersectionPoint = rayOrigin + t*rayVector;
    return true;
}
