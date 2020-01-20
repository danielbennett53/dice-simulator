#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Geometry>


class ObjReader
{
public:
    ObjReader(const std::string& obj_file);

    struct objVertex {
        unsigned int point_idx;
        unsigned int tex_idx;
        unsigned int normal_idx;

        objVertex(std::vector<int>& idxs) {
            point_idx = idxs[0];
            tex_idx = idxs[1];
            normal_idx = idxs[2];
        }

        bool operator==(const objVertex& obj) const {
            return ((point_idx == obj.point_idx) && (tex_idx == obj.tex_idx));
        }
    };

    struct objFace {
        std::vector<unsigned int> point_idxs;
        std::vector<unsigned int> tex_idxs;
        std::vector<unsigned int> normal_idxs;
    };

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2d> tex_coords;
    std::vector<Eigen::Vector3d> normals;
    std::vector<std::vector<objVertex>> faces;

    std::string tex_file;

    friend std::ostream& operator<<(std::ostream& os, const ObjReader obj)
    {
        os << "points: " << std::endl;
        for (const auto& p : obj.points) {
            os << "v " << std::endl << p << std::endl;
        }

        os << "tex coords: " << std::endl;
        for (const auto& t : obj.tex_coords) {
            os << "vt " << std::endl << t << std::endl;
        }

        os << "normals: " << std::endl;
        for (const auto& n : obj.normals) {
            os << "n " << std::endl << n << std::endl;
        }

        os << "faces: " << std::endl;
        for (const auto& f : obj.faces) {
            os << "face: ";
            for (const auto& i : f){
                os << i.point_idx << "/" << i.tex_idx << "/" << i.normal_idx << " ";
            }
            os << std::endl;
        }

        os << "Tex File: " << obj.tex_file << std::endl;
        return os;
    }
};
