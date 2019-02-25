#pragma once

#include <Eigen/Geometry>
#include <string>

namespace Dice {

    typedef struct {
        std::vector<int> indices;
        int num
        std::string texture_file;
    } Face;

    typdef struct {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<std::vector<int>> faces;
        std::vector
    } Die;



}