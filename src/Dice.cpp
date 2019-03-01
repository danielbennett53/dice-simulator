#include <Eigen/Geometry>
#include "Dice.h"
#include <vector>
#include <Mesh.h>
#include "SolidBody.h"


namespace Dice {

    Mesh generateMesh(Die dieType, textureUnit)
    {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Texture texture;

        // Add indices for each face
        for (auto face : dieType.faces) {
            for (int i=0; i<face.indices.size(); ++i) {
                Vertex vertex = {
                    .position = dieType.vertices[face.indices[i]],
                    .color = face.color,
                    .texCoords = face.tex_coords[i],
                    .texNum = face.num
                };
                vertices.push_back(vertex);
            }
            // Generate indices for two triangles per face
            std::vector<unsigned int> triIndices{0, 1, 2, 1, 2, 3};
            for (auto idx : triIndices) {
                indices.push_back(face.indices[idx]);
            }
            // Save face's texture
            texture.file_paths.push_back(face.texture_file);
        }

        return Mesh::Mesh(vertices, indices, texture, textureUnit);
    }

    SolidBody generateSolidBody(Die dieType, std::shared_ptr<Mesh> mesh) {

        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::vector<unsigned int>> faces;

        for (auto vertex : dieType.vertices) {
            Eigen::Vector3d vec(vertex[0], vertex[1], vertex[2]);
            vertices.push_back(vec);
        }
        for (auto face : dieType.faces) {
            faces.push_back(face.indices);
        }

        return SolidBody(vertices, faces, mesh);
    }


    const Die d6 = {
            .vertices = {std::vector<double>(-1, -1, -1),
                         std::vector<double>(-1, -1, 1),
                         std::vector<double>(-1, 1, -1),
                         std::vector<double>(-1, 1, 1),
                         std::vector<double>(1, -1, -1),
                         std::vector<double>(1, -1, 1),
                         std::vector<double>(1, 1, -1),
                         std::vector<double>(1, 1, 1)},
            .faces = {
                    {.num = 1,
                            .indices = {0, 2, 4, 6},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_1.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},

                    {.num = 2,
                            .indices = {0, 1, 2, 3},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_2.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},

                    {.num = 3,
                            .indices = {0, 1, 4, 5},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_3.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},

                    {.num = 4,
                            .indices = {1, 3, 5, 7},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_4.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},

                    {.num = 5,
                            .indices = {4, 5, 6, 7},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_5.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},

                    {.num = 6,
                            .indices = {2, 3, 6, 7},
                            .tex_coords = {std::vector<double>(0.0, 0.0),
                                           std::vector<double>(0.0, 1.0),
                                           std::vector<double>(1.0, 0.0),
                                           std::vector<double>(1.0, 1.0)},
                            .texture_file = PROJECT_DIR "/resources/square_6.png",
                            .color = std::vector<double>(1.0, 1.0, 1.0)},
            }
    };
};