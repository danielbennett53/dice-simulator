#include <Eigen/Geometry>
#include "Dice.h"
#include <vector>
#include <memory>
#include "Mesh.h"
#include <iostream>
//#include "SolidBody.h"


namespace Dice {

    std::shared_ptr<Mesh> generateMesh(Die dieType, unsigned int textureUnit)
    {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Texture texture;
        texture.id = 0;

        // Add indices for each face
        for (auto face : dieType.faces) {
            for (unsigned int i=0; i<face.indices.size(); ++i) {
                Vertex vertex = {
                    .position = dieType.vertices[face.indices[i]],
                    .color = face.color,
                    .texCoords = face.tex_coords[i],
                    .texNum = face.num
                };
                vertices.emplace_back(vertex);
            }
            // Get offset of current face in vertex vector
            auto offset = vertices.size() - face.indices.size();
            // Generate indices for two triangles per face
            std::vector<unsigned int> triIndices{0, 1, 2, 1, 2, 3};
            for (auto idx : triIndices) {
                indices.emplace_back(offset + idx);
            }
            // Save face's texture
            texture.file_paths.emplace_back(face.texture_file);
        }

        // Return shared pointer to mesh object
        return std::make_shared<Mesh>(vertices, indices, texture, textureUnit);
    }

//    std::shared_ptr<SolidBody> generateSolidBody(Die dieType, std::shared_ptr<Mesh> mesh) {
//
//        std::vector<Eigen::Vector3d> vertices;
//        std::vector<std::vector<unsigned int>> faces;
//
//        for (auto vertex : dieType.vertices) {
//            Eigen::Vector3d vec(vertex[0], vertex[1], vertex[2]);
//            vertices.push_back(vec);
//        }
//        for (const auto &face : dieType.faces) {
//            faces.push_back(face.indices);
//        }
//
//        // Return shared pointer to mesh object
//        return std::make_shared<SolidBody>(vertices, faces, mesh);
//    }


    const Die d6 = {
            .vertices = { {-1.0f, -1.0f, -1.0f},
                          {-1.0f, -1.0f, 1.0f},
                          {-1.0f, 1.0f, -1.0f},
                          {-1.0f, 1.0f, 1.0f},
                          {1.0f, -1.0f, -1.0f},
                          {1.0f, -1.0f, 1.0f},
                          {1.0f, 1.0f, -1.0f},
                          {1.0f, 1.0f, 1.0f}},
            .faces = {
                    {.num = 0,
                    .indices = {0, 2, 4, 6},
                    .tex_coords = {{0.0f, 0.0f},
                                   {0.0f, 1.0f},
                                   {1.0f, 0.0f},
                                   {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_1.png",
                    .color = {1.0f, 1.0f, 1.0f} },

                    {.num = 1,
                    .indices = {0, 1, 2, 3},
                    .tex_coords = { {0.0f, 0.0f},
                                    {0.0f, 1.0f},
                                    {1.0f, 0.0f},
                                    {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_2.png",
                    .color = {1.0f, 1.0f, 1.0f} },

                    {.num = 2,
                    .indices = {0, 1, 4, 5},
                    .tex_coords = {{0.0f, 0.0f},
                                   {0.0f, 1.0f},
                                   {1.0f, 0.0f},
                                   {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_3.png",
                    .color = {1.0f, 1.0f, 1.0f}},

                    {.num = 3,
                    .indices = {2, 3, 6, 7},
                    .tex_coords = {{0.0f, 0.0f},
                                   {0.0f, 1.0f},
                                   {1.0f, 0.0f},
                                   {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_4.png",
                    .color = {1.0f, 1.0f, 1.0f}},

                    {.num = 4,
                    .indices = {4, 5, 6, 7},
                    .tex_coords = {{0.0f, 0.0f},
                                   {0.0f, 1.0f},
                                   {1.0f, 0.0f},
                                   {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_5.png",
                    .color = {1.0f, 1.0f, 1.0f}},

                    {.num = 5,
                    .indices = {1, 3, 5, 7},
                    .tex_coords = {{0.0f, 0.0f},
                                   {0.0f, 1.0f},
                                   {1.0f, 0.0f},
                                   {1.0f, 1.0f}},
                    .texture_file = PROJECT_DIR "/resources/square_6.png",
                    .color = {1.0f, 1.0f, 1.0f}},
            }
    };
};