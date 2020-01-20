#include "ObjReader.h"

#include <fstream>
#include <iostream>

ObjReader::ObjReader(const std::string& obj_file)
{
    // Open OBJ file
    std::ifstream fd;
    fd.open(obj_file, std::ios::in);
    if (!fd) {
        std::cerr << "Unable to open file " << obj_file << std::endl;
        return;
    }

    std::string line;
    while (std::getline(fd, line)) {
        std::stringstream ls(line);
        std::string token;
        ls >> token;

        if (token == "v") { // Vertex
            Eigen::Vector3d point;
            ls >>  point[0] >> point[1] >> point[2];
            points.push_back(point);
        } else if (token == "vt") { // Texture coordinates
            Eigen::Vector2d tex_coord;
            ls >> tex_coord[0] >> tex_coord[1];
            tex_coords.push_back(tex_coord);
        } else if (token == "vn") { // Normal vectors
            Eigen::Vector3d normal;
            ls >> normal[0] >> normal[1] >> normal[2];
            normals.push_back(normal);
        } else if (token == "f") { // Faces/Triangle indices
            std::vector<objVertex> vtxs;
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
                vtxs.emplace_back(idxs);
            }
            faces.push_back(vtxs);

        } else if (token == "mtllib") { // Material file
            // Get name of mtl file
            std::string filename;
            std::string ignore;
            ls >> filename;

            // Split input filename to get filepath
            size_t found;
            found = obj_file.find_last_of("/\\");
            std::string filepath = obj_file.substr(0, found+1);

            // Open material file
            std::ifstream mtlfd;
            mtlfd.open(filepath + filename, std::ios::in);
            if (!mtlfd) {
                std::cerr << "Unable to open file " << filepath + filename << std::endl;
                tex_file = "";
                continue;
            }

            // Scan mtl file for texture image filename
            while (mtlfd) {
                std::string mtlToken;
                mtlfd >> mtlToken;

                if (mtlToken == "map_Kd") {
                    std::string texFile;
                    mtlfd >> texFile;
                    tex_file = filepath + texFile;
                    break;
                }
            }
            mtlfd.close();
        }
    }
}
