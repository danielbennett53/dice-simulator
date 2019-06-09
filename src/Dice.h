#pragma once


#include <string>
#include "Mesh.h"
#include <memory>
#include "SolidBody.h"

namespace Dice {
    typedef struct {
        unsigned int num;
        std::vector<unsigned int> indices;
        std::vector<std::vector<float>> tex_coords;
        std::string texture_file;
        std::vector<float> color;
    } Face;

    typedef struct {
        std::vector<std::vector<float>> vertices;
        std::vector<Face> faces;
    } Die;


    // Generation functions
    std::shared_ptr<Mesh> generateMesh(Die type, unsigned int textureUnit);
    std::shared_ptr<SolidBody> generateSolidBody(Die type, std::shared_ptr<Mesh> mesh);

    // Prototypes of different dice types
    extern const Die d6;
    extern const Die d4;
};