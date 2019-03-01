#pragma once


#include <string>
#include "Mesh.h"
#include "SolidBody.h"

namespace Dice {
    typedef struct {
        int num;
        std::vector<int> indices;
        std::vector<std::vector<double>> tex_coords;
        std::string texture_file;
        std::vector<double> color;
    } Face;

    typedef struct {
        std::vector<std::vector<double>> vertices;
        std::vector<Face> faces;
    } Die;


    // Generation functions
    Mesh generateMesh(Die type);
    SolidBody generateSolidBody(Die type);

    // Prototypes of different dice types
    extern Die d6;
};